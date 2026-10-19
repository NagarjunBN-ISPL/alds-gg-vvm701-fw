#!/usr/bin/env python3
"""
bluesky_sim.py - BlueSky dispenser slave for alds-gg-vvm701-fw bench tests.

Stands in for BlueSky pumps on the gateway's RS485 line (a USB-RS485 adapter on
the gateway's A/B terminals), 9600 8E1. It answers the gateway's polls
byte-exact to what src/dispenser.c validates, and takes RSP writes from the
cloud the way the station pump does: the new price is kept and served on every
later RSP read, so it shows up in the gateway's telemetry.

    request : F5 <addr> A2 <cmd> <chk>                 chk = addr ^ cmd ^ 0xD7
    reply   : F5 <addr> A<n> <data..> <cmd> <chk>      n = bytes after byte 2
                                                       chk = XOR of all prior bytes & 0x7F
    D5 status   : 1 byte   0x80 idle, 0x81 delivering
    C5 total    : 10 BCD   5 bytes litres x100, 5 bytes amount x100
    B6 RSP      : 2 BCD    price x100          (65.82 -> 65 82)
    E2 k-factor : 2 bytes  raw, echoed as set  (default 23 60)
    writes      : any frame longer than 5 bytes, built by the cloud and forwarded
                  unread by the gateway's rsp/kfactor tunnel (src/rsp_tunnel.c).
                  B2 sets RSP, the layout proven at the station (2026-09-29):
                      664: F5 <addr> A4 <RR> <PP> B2 <chk>       (default)
                      886: F5 <addr> A5 00 <RR> <PP> B2 <chk>    (--variant 886)
                  E3 sets k-factor (GUESS, --kf-write-cmd). A write is checked
                  the way a pump would - checksum, layout, BCD price - and one
                  that fails gets NO reply (the cloud's .../out says 404), with
                  the reason in the log. A good one is answered like the
                  station pump:  F5 <addr> A3 59 <cmd> <chk>

A BlueSky dispenser N owns nozzle addresses 2N-1 and 2N; the gateway polls it
from a slot set to du<k>Make "bluesky", du<k>Sfx "0N".

While it runs, type:
    show                              price, totals and state of every nozzle
    sale <nz> [litres] [seconds]      start a sale (status 0x81 until it ends)
    price <nz> <price>                change the price at the pump: price 1 68.80
    help, quit

    python bluesky_sim.py --list                               # serial ports
    python bluesky_sim.py --port COM13                         # DU1 (nz 1,2)
    python bluesky_sim.py --port COM13 --du 1 2 --log sim.txt  # nz 1..4, log also in sim.txt
    python bluesky_sim.py --port COM13 --sale-every 90         # random sale, delivering 25-60 s
    python bluesky_sim.py --port COM13 --fault 5               # 5% corrupted replies
"""

import argparse
import queue
import random
import sys
import threading
import time
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial is missing: python -m pip install pyserial")

PRE = 0xF5
CMD_STATUS, CMD_TOTAL, CMD_RSP, CMD_KFACT = 0xD5, 0xC5, 0xB6, 0xE2
CMD_RSP_WRITE = 0xB2
WRITE_ACK = b"\x59"          # the station pump's ack data byte: F5 nz A3 59 <cmd> chk
FRAME_GAP_S = 0.1            # silence that ends a frame cut short: the pump drops it
QUIET_HINT_S = 60            # nothing heard for this long: print what to check
PRICE_MAX_C = 9999           # 2 BCD bytes hold 99.99

HELP = """commands:
  show                          price, totals and state of every nozzle
  sale <nz> [litres] [seconds]  start a sale on nozzle <nz> (default: random litres, 40 s)
  price <nz> <price>            change the price at the pump, e.g. price 1 68.80
  help                          this list
  quit                          stop (Ctrl+C works too)"""

_log_file = None             # --log: every line is also appended here


def chk(b: bytes) -> int:
    x = 0
    for v in b:
        x ^= v
    return x & 0x7F


def reply(addr: int, cmd: int, data: bytes) -> bytes:
    body = bytes([PRE, addr, 0xA0 | (len(data) + 2)]) + data + bytes([cmd])
    return body + bytes([chk(body)])


def bcd(value: int, nbytes: int) -> bytes:
    s = f"{value:0{nbytes * 2}d}"[-nbytes * 2:]
    return bytes(int(s[i:i + 2], 16) for i in range(0, len(s), 2))


def is_bcd(b: bytes) -> bool:
    return all((v >> 4) <= 9 and (v & 0x0F) <= 9 for v in b)


def hexs(b: bytes) -> str:
    return ",".join(f"{v:02x}" for v in b)


def log(msg: str):
    line = f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {msg}"
    print(line, flush=True)
    if _log_file:
        _log_file.write(line + "\n")
        _log_file.flush()


class Nozzle:
    def __init__(self, addr: int, price: float, kf: bytes):
        self.addr = addr
        self.price_c = int(round(price * 100))        # paise
        self.kf = kf
        self.vol_c = 2_151_826 + addr * 100_000       # litres x100
        self.amt_c = self.vol_c * self.price_c // 100 # rupees x100
        self.sale_end = 0.0                           # delivering until this time
        self.sale_vol_c = 0

    def start_sale(self, litres: float, seconds: float):
        self.sale_vol_c = int(litres * 100)
        self.sale_end = time.time() + seconds

    def delivering(self) -> bool:
        return time.time() < self.sale_end

    def on_status(self) -> int:
        if self.sale_end:
            if self.delivering():
                return 0x81
            self.sale_end = 0.0                       # sale settles on this poll
            self.vol_c += self.sale_vol_c
            self.amt_c += self.sale_vol_c * self.price_c // 100
            log(f"nz{self.addr} sale done: +{self.sale_vol_c / 100:.2f} L "
                f"-> total {self.vol_c / 100:.2f} L / Rs {self.amt_c / 100:.2f}")
        return 0x80


class Sim:
    def __init__(self, a, ser=None):
        self.nz = {}
        kf = bytes.fromhex(a.kfactor)
        for du in a.du:
            for addr in (2 * du - 1, 2 * du):
                self.nz[addr] = Nozzle(addr, a.price, kf)
        self.variant = a.variant
        self.kf_write_cmd = int(a.kf_write_cmd, 16)
        self.fault = a.fault / 100.0
        self.sale_every = a.sale_every
        self.next_sale = time.time() + a.sale_every if a.sale_every else None
        self.delay = a.delay / 1000.0
        self.last_tx = b""                            # to drop our own echo (RS485 adapters)
        self.buf = bytearray()                        # bytes not yet a whole frame
        self.last_rx = 0.0                            # when the last byte arrived
        self.last_heard = time.time()                 # when the last whole frame arrived
        self.noise = 0                                # bytes that were not a frame
        self.cmds = queue.Queue()                     # lines typed at the console
        self.ser = ser or serial.Serial(a.port, a.baud, bytesize=8, parity=serial.PARITY_EVEN,
                                        stopbits=1, timeout=0.02)
        log(f"{a.port} {a.baud} 8E1, BlueSky {self.variant}, nozzles {sorted(self.nz)}, "
            f"RSP {a.price:.2f}, kfact {kf.hex()}")
        for du in a.du:
            log(f"  dispenser {du:02d} = nozzles {2 * du - 1} and {2 * du}; on the gateway: "
                f"du<k>Make \"bluesky\", du<k>Sfx \"{du:02d}\"")
        log("type 'help' for commands")

    def maybe_sale(self):
        if self.next_sale and time.time() >= self.next_sale:
            self.next_sale = time.time() + self.sale_every
            idle = [n for n in self.nz.values() if not n.sale_end]
            if not idle:
                return
            n = random.choice(idle)
            litres = round(random.uniform(2, 25), 2)
            secs = random.uniform(25, 60)             # outlasts one 20 s poll + 3 status retries
            n.start_sale(litres, secs)
            log(f"nz{n.addr} sale started: {litres:.2f} L, delivering for {secs:.0f} s")

    def send(self, frame: bytes, what: str):
        if self.fault and random.random() < self.fault:
            frame = bytearray(frame)
            i = random.randrange(3, len(frame))
            frame[i] ^= 1 << random.randrange(8)
            frame = bytes(frame)
            what += "  ** FAULT: bit flipped **"
        time.sleep(self.delay)
        self.last_tx = frame
        self.ser.write(frame)
        self.ser.flush()
        log(f"TX {hexs(frame)}  {what}")

    def handle(self, f: bytes):
        if f == self.last_tx:                         # our own reply echoed back
            self.last_tx = b""
            return
        self.last_heard = time.time()
        addr = f[1]
        n = self.nz.get(addr)
        if n is None:
            log(f"RX {hexs(f)}  nz{addr} not simulated - silent")
            return
        if len(f) > 5:
            log(f"RX {hexs(f)}  <- WRITE to nz{addr}")
            self.write(n, f)
            return
        if len(f) < 5 or f[4] != (addr ^ f[3] ^ 0xD7):
            log(f"RX {hexs(f)}  bad request (length or checksum) - silent")
            return
        cmd = f[3]
        log(f"RX {hexs(f)}")

        if cmd == CMD_STATUS:
            st = n.on_status()
            self.send(reply(addr, cmd, bytes([st])),
                      f"nz{addr} status {'idle' if st == 0x80 else 'DELIVERING'}")
        elif cmd == CMD_TOTAL:
            self.send(reply(addr, cmd, bcd(n.vol_c, 5) + bcd(n.amt_c, 5)),
                      f"nz{addr} total {n.vol_c / 100:.2f} L / Rs {n.amt_c / 100:.2f}")
        elif cmd == CMD_RSP:
            self.send(reply(addr, cmd, bcd(n.price_c, 2)),
                      f"nz{addr} RSP {n.price_c / 100:.2f}")
        elif cmd == CMD_KFACT:
            self.send(reply(addr, cmd, n.kf), f"nz{addr} k-factor {n.kf.hex()}")
        else:
            log(f"nz{addr} unknown read 0x{cmd:02x} - silent")

    def write(self, n: Nozzle, f: bytes):
        """A frame from the cloud's tunnel, checked the way a pump checks it. One
        that fails gets no reply at all, so the gateway answers the cloud 404."""
        addr, cmd, data = f[1], f[-2], f[3:-2]
        if f[-1] != chk(f[:-1]):
            log(f"   IGNORED: checksum {f[-1]:02x}, should be {chk(f[:-1]):02x} (XOR of every "
                "byte before it, & 0x7F) - no reply, the cloud gets 404")
            return
        if cmd == CMD_RSP_WRITE:
            want, layout = (3, "A5 00 RR PP") if self.variant == "886" else (2, "A4 RR PP")
            if len(data) != want:
                log(f"   IGNORED: not a {self.variant} RSP write (F5 nz {layout} B2 chk) "
                    "- no reply, the cloud gets 404")
                return
            if not is_bcd(data[-2:]):
                log(f"   IGNORED: price {hexs(data[-2:])} is not BCD (68.80 is sent as 68,80) "
                    "- no reply, the cloud gets 404")
                return
            if n.delivering():
                log(f"   !! nz{addr} is DELIVERING: the cloud wrote mid-sale without reading "
                    "the status (D5) first")
            old = n.price_c
            n.price_c = int(data[-2:].hex())
            self.send(reply(addr, cmd, WRITE_ACK), f"nz{addr} write ACK")
            log(f"*** nz{addr} RSP {old / 100:.2f} -> {n.price_c / 100:.2f}, served from the "
                "next RSP read on ***")
        elif cmd == self.kf_write_cmd and len(data) >= 2:
            old = n.kf
            n.kf = bytes(data[-2:])
            self.send(reply(addr, cmd, WRITE_ACK), f"nz{addr} write ACK")
            log(f"*** nz{addr} k-factor {old.hex()} -> {n.kf.hex()} ***")
        else:
            log(f"   IGNORED: command 0x{cmd:02x} is not an RSP write (B2) or a k-factor write "
                f"({self.kf_write_cmd:02x}) - no reply, the cloud gets 404")

    def feed(self, chunk: bytes, now: float):
        """Bytes off the line -> whole frames -> handle()."""
        if chunk:
            self.buf += chunk
            self.last_rx = now
        while True:
            i = self.buf.find(PRE)
            if i < 0:
                self.noise += len(self.buf)
                self.buf.clear()
                return
            self.noise += i
            del self.buf[:i]
            if len(self.buf) < 3:
                break
            if (self.buf[2] & 0xF0) != 0xA0:
                self.noise += 1
                del self.buf[0]                       # not a header, resync
                continue
            ln = (self.buf[2] & 0x0F) + 3
            if len(self.buf) < ln:
                break
            frame = bytes(self.buf[:ln])
            del self.buf[:ln]
            self.handle(frame)
        # The line went quiet before the frame its length byte promised was
        # complete. A pump resets its receiver on the gap, so drop it rather than
        # glue it to the next frame.
        if self.buf and now - self.last_rx > FRAME_GAP_S:
            want = (self.buf[2] & 0x0F) + 3 if len(self.buf) > 2 else "?"
            log(f"RX {hexs(self.buf)}  cut short (its length byte says {want} bytes) - ignored")
            self.buf.clear()

    def tick(self, now: float):
        self.maybe_sale()
        while not self.cmds.empty():
            self.command(self.cmds.get())
        if now - self.last_heard > QUIET_HINT_S:
            seen = f"; {self.noise} stray bytes arrived (A/B swapped?)" if self.noise else ""
            log(f"nothing from the gateway for {QUIET_HINT_S} s - check: gateway online and set "
                f"for these dispensers, adapter on the RS485 A/B terminals, right COM port{seen}")
            self.last_heard = now
            self.noise = 0

    def console(self):
        for line in sys.stdin:
            self.cmds.put(line.strip())

    def nozzle(self, arg: str) -> Nozzle:
        if not arg.isdigit():
            raise ValueError(f"'{arg}' is not a nozzle number")
        n = self.nz.get(int(arg))
        if n is None:
            raise ValueError(f"nozzle {arg} is not simulated (nozzles {sorted(self.nz)})")
        return n

    def command(self, line: str):
        p = line.split()
        if not p:
            return
        c = p[0].lower()
        try:
            if c == "show":
                self.show()
            elif c == "sale":
                n = self.nozzle(p[1])
                litres = float(p[2]) if len(p) > 2 else round(random.uniform(2, 25), 2)
                secs = float(p[3]) if len(p) > 3 else 40.0
                n.start_sale(litres, secs)
                log(f"nz{n.addr} sale started by hand: {litres:.2f} L, delivering for {secs:.0f} s")
            elif c == "price":
                n = self.nozzle(p[1])
                new = int(round(float(p[2]) * 100))
                if not 0 <= new <= PRICE_MAX_C:
                    raise ValueError("the price must be 0.00 to 99.99")
                log(f"nz{n.addr} RSP {n.price_c / 100:.2f} -> {new / 100:.2f}, set at the pump")
                n.price_c = new
            elif c == "help":
                print(HELP, flush=True)
            elif c in ("quit", "exit"):
                raise KeyboardInterrupt
            else:
                print(f"unknown command '{c}' - type help", flush=True)
        except IndexError:
            print(f"'{line}': a value is missing - type help", flush=True)
        except ValueError as e:
            print(f"'{line}': {e} - type help", flush=True)

    def show(self):
        log("nozzle   RSP      total L      total Rs  kfact  state")
        for a, n in sorted(self.nz.items()):
            log(f"  {a:<5} {n.price_c / 100:6.2f}  {n.vol_c / 100:11.2f}  {n.amt_c / 100:12.2f}  "
                f"{n.kf.hex()}   {'DELIVERING' if n.delivering() else 'idle'}")

    def run(self):
        threading.Thread(target=self.console, daemon=True).start()
        try:
            while True:
                self.feed(self.ser.read(64), time.time())
                self.tick(time.time())
        except KeyboardInterrupt:
            log("stopped")
        finally:
            self.ser.close()


def main():
    global _log_file
    p = argparse.ArgumentParser(description="BlueSky dispenser slave simulator")
    p.add_argument("--port", "-p", help="serial port, e.g. COM13")
    p.add_argument("--baud", "-b", type=int, default=9600)
    p.add_argument("--du", type=int, nargs="+", default=[1],
                   help="BlueSky dispenser numbers to simulate (1..6), default 1")
    p.add_argument("--price", type=float, default=65.82, help="unit price (RSP)")
    p.add_argument("--kfactor", default="2360", help="k-factor raw bytes, hex (default 2360)")
    p.add_argument("--variant", choices=["664", "886"], default="664",
                   help="pump model, sets the RSP write layout it accepts (default 664)")
    p.add_argument("--kf-write-cmd", default="E3",
                   help="opcode treated as a k-factor write (hex, default E3 - unconfirmed)")
    p.add_argument("--sale-every", type=float, default=0,
                   help="start a random sale every N seconds (0 = off)")
    p.add_argument("--fault", type=float, default=0, help="percent of replies with a bit flipped")
    p.add_argument("--delay", type=float, default=20, help="reply delay in ms (default 20)")
    p.add_argument("--log", help="also append every line to this file")
    p.add_argument("--list", "-l", action="store_true", help="list serial ports")
    a = p.parse_args()

    if a.list or not a.port:
        for port in serial.tools.list_ports.comports():
            print(f"{port.device:10} {port.description}")
        if not a.port:
            print("\nuse --port <PORT>")
        return
    if any(d < 1 or d > 6 for d in a.du):
        sys.exit("--du values must be 1..6")
    if not 0 <= round(a.price * 100) <= PRICE_MAX_C:
        sys.exit("--price must be 0.00 to 99.99 (2 BCD bytes)")
    if a.log:
        _log_file = open(a.log, "a", encoding="utf-8")
    try:
        Sim(a).run()
    finally:
        if _log_file:
            _log_file.close()


if __name__ == "__main__":
    main()
