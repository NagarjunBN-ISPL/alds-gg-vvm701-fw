#!/usr/bin/env python3
"""
yenen_sim.py - Yenen dispenser slave for alds-gg-vvm701-fw bench tests.

Acts as one or more Yenen dispensers on a serial port (USB-RS485 adapter on
the gateway's RS485 A/B), 9600 8N1, byte-exact to what src/yenen.c sends and
validates. Default: dispensers 1 and 2, both sides = 4 nozzles.

    Command 2 (read)  : 02 '0' '8' '0' <head> <nz> 'A' 03 <ck_hi> <ck_lo>      10 bytes
    reply             : 02 <47 ASCII chars> 03 <ck_hi> <ck_lo>                 51 bytes
    Command K (RSP)   : 02 '0' '8' '0' <head> <nz> 'K' p1 p2 p3 p4 03 <ck_hi> <ck_lo>
                        p1..p4 = price x100 as 4 ASCII digits (65.32 -> '6','5','3','2')
    reply             : 06 30 31 (ACK)  or  15 (NAK: price not 4 digits, or --nak)

    checksum : XOR of STX..ETX, each nibble + 0x30 (0x71 -> '7' '1')
    <head>   : '0' + head (1..5)
    <nz>     : side A '0'+(head-1), side B '5'+(head-1)

A frame with a bad checksum, for a nozzle not simulated, or with an unknown
command gets NO reply, so the gateway's rsp tunnel answers the cloud 404. A
write while the nozzle is dispensing is accepted but flagged with "!!": the
cloud should have read the status (Command 2) first.

47-char reply payload (CLOUD_INTEGRATION_GUIDE.md appendix):
    "49" "0"<head><nz> "A" price4 preset_amt4 preset_vol4 deliv_l3 deliv_amt6
    total_l6 total_amt8 status2 "0990"          status "00" idle, "11" dispensing
Totals are kept x100, as in YENEN_TEST_VECTORS.md. The delivery field holds
9.99 L at most, so a sale is 9.99 L or less.

Yenen dispenser N is head N; its nozzles are numbered like the gateway's
telemetry labels: side A = 2N-1, side B = 2N (dispenser 2 = nozzles 3 and 4).

While it runs, type:
    show                              price, totals and state of every nozzle
    sale <nz> [litres] [seconds]      start a sale (status 11 until it ends)
    price <nz> <price>                change the price at the pump: price 1 68.80
    frame <nz> <price>                print the Command K frame to send from the cloud
    help, quit

    python yenen_sim.py --list                                 # serial ports
    python yenen_sim.py --port COM5                            # dispensers 1 and 2
    python yenen_sim.py --port COM5 --du 1 2 3 --log sim.txt   # 3 dispensers, log also in sim.txt
    python yenen_sim.py --port COM5 --sides A                  # single-sided, side B silent
    python yenen_sim.py --port COM5 --sale-every 90            # random sale, dispensing 25-60 s
    python yenen_sim.py --port COM5 --nak                      # refuse every RSP write
    python yenen_sim.py --port COM5 --fault 5                  # 5% corrupted replies
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

STX, ETX, ACK, NAK = 0x02, 0x03, 0x06, 0x15
ACK_FRAME = bytes([ACK, 0x30, 0x31])     # the pump's "accepted": 06 '0' '1'
FRAME_GAP_S = 0.1            # silence that ends a frame cut short: the pump drops it
FRAME_MAX = 64               # no request is longer; past this without ETX, resync
QUIET_HINT_S = 60            # nothing heard for this long: print what to check
PRICE_MAX_C = 9999           # 4 digits hold 99.99
SALE_MAX_C = 999             # the 3-digit delivery field holds 9.99 L

HELP = """commands:
  show                          price, totals and state of every nozzle
  sale <nz> [litres] [seconds]  start a sale on nozzle <nz> (default: random litres, 40 s)
  price <nz> <price>            change the price at the pump, e.g. price 1 68.80
  frame <nz> <price>            print the Command K frame the cloud sends to set <price>
  help                          this list
  quit                          stop (Ctrl+C works too)"""

_log_file = None             # --log: every line is also appended here


def cksum(frame: bytes) -> bytes:
    x = 0
    for b in frame:
        x ^= b
    return bytes([((x >> 4) & 0x0F) + 0x30, (x & 0x0F) + 0x30])


def nzcode(head: int, side: int) -> int:
    return (ord('5') if side else ord('0')) + head - 1


def request(head: int, side: int, cmd: str, data: bytes = b"") -> bytes:
    """A gateway/cloud request frame: Command 2 ('A') or Command K ('K')."""
    body = (bytes([STX]) + b"080" + bytes([ord('0') + head, nzcode(head, side), ord(cmd)])
            + data + bytes([ETX]))
    return body + cksum(body)


def hexs(b: bytes) -> str:
    return ",".join(f"{v:02x}" for v in b)


def log(msg: str):
    line = f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {msg}"
    print(line, flush=True)
    if _log_file:
        _log_file.write(line + "\n")
        _log_file.flush()


class Nozzle:
    def __init__(self, head: int, side: int, price: float):
        self.head, self.side = head, side
        self.label = (head - 1) * 2 + side + 1                 # the gateway's nozzle label
        self.name = f"nz{self.label} (head{head}/{'B' if side else 'A'})"
        self.price_c = int(round(price * 100))                 # paise
        self.tot_l = 874_333 + (head * 2 + side) * 10_000      # litres x100, 6 digits
        self.tot_amt = self.tot_l * self.price_c // 100        # rupees x100, 8 digits
        self.del_l = 0                                         # last delivery litres x100
        self.del_amt = 0
        self.sale = None             # (start, end, litres x100, base total L, base total Rs, price)

    def start_sale(self, litres_c: int, seconds: float):
        now = time.time()
        self.sale = (now, now + seconds, litres_c, self.tot_l, self.tot_amt, self.price_c)

    def dispensing(self) -> bool:
        return self.sale is not None and time.time() < self.sale[1]

    def update(self) -> str:
        """Advance any running sale; return the status field."""
        if not self.sale:
            return "00"
        t0, t1, lit, base_l, base_amt, price = self.sale
        frac = min(1.0, (time.time() - t0) / (t1 - t0))
        self.del_l = int(lit * frac)
        self.del_amt = self.del_l * price // 100
        self.tot_l = base_l + self.del_l
        self.tot_amt = base_amt + self.del_amt
        if frac < 1.0:
            return "11"
        self.sale = None
        log(f"{self.name} sale done: {self.del_l / 100:.2f} L / Rs {self.del_amt / 100:.2f} "
            f"-> total {self.tot_l / 100:.2f} L / Rs {self.tot_amt / 100:.2f}")
        return "00"

    def payload(self) -> str:
        st = self.update()
        s = (f"490{self.head}{chr(nzcode(self.head, self.side))}A"
             f"{self.price_c:04d}" "0000" "0000"
             f"{self.del_l % 1000:03d}{self.del_amt % 1_000_000:06d}"
             f"{self.tot_l % 1_000_000:06d}{self.tot_amt % 100_000_000:08d}"
             f"{st}0990")
        assert len(s) == 47, s
        return s


class Sim:
    def __init__(self, a, ser=None):
        self.nz = {}                                           # (head char, nz char) -> Nozzle
        sides = [s for s, c in ((0, "A"), (1, "B")) if c in a.sides.upper()]
        for head in a.du:
            for side in sides:
                self.nz[(ord('0') + head, nzcode(head, side))] = Nozzle(head, side, a.price)
        self.nak = a.nak
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
        self.ser = ser or serial.Serial(a.port, a.baud, bytesize=8, parity=serial.PARITY_NONE,
                                        stopbits=1, timeout=0.02)
        log(f"{a.port} {a.baud} 8N1, Yenen, nozzles {[n.name for n in self.by_label()]}, "
            f"RSP {a.price:.2f}{', every RSP write NAKed' if a.nak else ''}")
        for du in a.du:
            log(f"  dispenser {du:02d} = head {du}; on the gateway: du<k>Make \"yenen\", "
                f"du<k>Sfx \"{du:02d}\"; RSP writes to <base>/rsp/imei/<imei>{du:02d}/in")
        log("type 'help' for commands")

    def by_label(self):
        return sorted(self.nz.values(), key=lambda n: n.label)

    def maybe_sale(self):
        if self.next_sale and time.time() >= self.next_sale:
            self.next_sale = time.time() + self.sale_every
            idle = [n for n in self.nz.values() if not n.sale]
            if not idle:
                return
            n = random.choice(idle)
            litres_c = random.randint(200, SALE_MAX_C)
            secs = random.uniform(25, 60)             # outlasts one 10 s poll + retries
            n.start_sale(litres_c, secs)
            log(f"{n.name} sale started: {litres_c / 100:.2f} L, dispensing for {secs:.0f} s")

    def send(self, frame: bytes, what: str):
        if self.fault and len(frame) > 1 and random.random() < self.fault:
            frame = bytearray(frame)
            i = random.randrange(1, len(frame))
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
        if f[-2:] != cksum(f[:-2]):
            log(f"RX {hexs(f)}  checksum {hexs(f[-2:])}, should be {hexs(cksum(f[:-2]))} "
                "(XOR of STX..ETX, each nibble + 0x30) - silent, the cloud gets 404")
            return
        if len(f) < 10 or f[1:4] != b"080":
            log(f"RX {hexs(f)}  not a Yenen request (no '080' after STX) - silent")
            return
        n = self.nz.get((f[4], f[5]))
        if n is None:
            log(f"RX {hexs(f)}  head '{chr(f[4])}' nz '{chr(f[5])}' not simulated - silent")
            return
        cmd = f[6]

        if cmd == ord('A') and len(f) == 10:
            log(f"RX {hexs(f)}  {n.name} Command 2 (read)")
            p = n.payload()
            body = bytes([STX]) + p.encode() + bytes([ETX])
            st = "DISPENSING" if p[41:43] != "00" else "idle"
            self.send(body + cksum(body),
                      f"{n.name} {st}, RSP {n.price_c / 100:.2f}, "
                      f"total {n.tot_l / 100:.2f} L / Rs {n.tot_amt / 100:.2f}")
        elif cmd == ord('K') and len(f) == 14:
            log(f"RX {hexs(f)}  <- RSP WRITE to {n.name}")
            digits = f[7:11].decode(errors="replace")
            if not digits.isdigit():
                self.send(bytes([NAK]), f"{n.name} price '{digits}' is not 4 digits, NAK")
                return
            if self.nak:
                self.send(bytes([NAK]), f"{n.name} RSP write {digits} refused (--nak), NAK")
                return
            if n.dispensing():
                log(f"   !! {n.name} is DISPENSING: the cloud wrote mid-sale without reading "
                    "the status (Command 2) first")
            old = n.price_c
            n.price_c = int(digits)
            self.send(ACK_FRAME, f"{n.name} RSP write ACK")
            log(f"*** {n.name} RSP {old / 100:.2f} -> {n.price_c / 100:.2f}, served from the "
                "next Command 2 on ***")
        else:
            log(f"RX {hexs(f)}  {n.name} command '{chr(cmd)}' ({len(f)} bytes) is not "
                "Command 2 or K - silent, the cloud gets 404")

    def feed(self, chunk: bytes, now: float):
        """Bytes off the line -> whole frames -> handle()."""
        if chunk:
            self.buf += chunk
            self.last_rx = now
        while True:
            i = self.buf.find(STX)
            if i < 0:
                self.noise += len(self.buf)
                self.buf.clear()
                return
            self.noise += i
            del self.buf[:i]
            e = self.buf.find(ETX, 1)
            nxt = self.buf.find(STX, 1)
            if nxt >= 0 and (e < 0 or nxt < e):        # a new frame began before this one
                self.noise += nxt                      # ended: STX is never inside a request
                del self.buf[:nxt]
                continue
            if e < 0:
                if len(self.buf) > FRAME_MAX:          # no ETX in sight, resync
                    self.noise += 1
                    del self.buf[0]
                    continue
                break
            if len(self.buf) < e + 3:                  # ETX + 2 checksum bytes
                break
            frame = bytes(self.buf[:e + 3])
            del self.buf[:e + 3]
            self.handle(frame)
        # The line went quiet before the frame was complete. A pump resets its
        # receiver on the gap, so drop it rather than glue it to the next frame.
        if self.buf and now - self.last_rx > FRAME_GAP_S:
            log(f"RX {hexs(self.buf)}  cut short (no ETX + 2 checksum bytes) - ignored")
            self.buf.clear()

    def tick(self, now: float):
        self.maybe_sale()
        while not self.cmds.empty():
            self.command(self.cmds.get())
        if now - self.last_heard > QUIET_HINT_S:
            seen = f"; {self.noise} stray bytes arrived (A/B swapped? baud?)" if self.noise else ""
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
        for n in self.nz.values():
            if n.label == int(arg):
                return n
        raise ValueError(f"nozzle {arg} is not simulated "
                         f"(nozzles {[n.label for n in self.by_label()]})")

    @staticmethod
    def price_arg(arg: str) -> int:
        c = int(round(float(arg) * 100))
        if not 0 <= c <= PRICE_MAX_C:
            raise ValueError("the price must be 0.00 to 99.99")
        return c

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
                litres_c = (int(round(float(p[2]) * 100)) if len(p) > 2
                            else random.randint(200, SALE_MAX_C))
                if not 1 <= litres_c <= SALE_MAX_C:
                    raise ValueError("a sale is 0.01 to 9.99 L (the delivery field holds 9.99)")
                secs = float(p[3]) if len(p) > 3 else 40.0
                if secs <= 0:
                    raise ValueError("seconds must be above 0")
                n.start_sale(litres_c, secs)
                log(f"{n.name} sale started by hand: {litres_c / 100:.2f} L, "
                    f"dispensing for {secs:.0f} s")
            elif c == "price":
                n = self.nozzle(p[1])
                new = self.price_arg(p[2])
                log(f"{n.name} RSP {n.price_c / 100:.2f} -> {new / 100:.2f}, set at the pump")
                n.price_c = new
            elif c == "frame":
                n = self.nozzle(p[1])
                new = self.price_arg(p[2])
                k = request(n.head, n.side, "K", f"{new:04d}".encode())
                r = request(n.head, n.side, "A")
                log(f"{n.name} to {new / 100:.2f}, publish to <base>/rsp/imei/<imei>{n.head:02d}/in:")
                log(f"  write (Command K) : {hexs(k)}")
                log(f"  status (Command 2): {hexs(r)}   read this first, write only on status 00")
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
        log("nozzle            RSP      total L      total Rs  state")
        for n in self.by_label():
            n.update()
            log(f"  {n.name:<15} {n.price_c / 100:6.2f}  {n.tot_l / 100:11.2f}  "
                f"{n.tot_amt / 100:12.2f}  {'DISPENSING' if n.dispensing() else 'idle'}")

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
    p = argparse.ArgumentParser(description="Yenen dispenser slave simulator")
    p.add_argument("--port", "-p", help="serial port, e.g. COM5")
    p.add_argument("--baud", "-b", type=int, default=9600)
    p.add_argument("--du", type=int, nargs="+", default=[1, 2],
                   help="Yenen dispenser (head) numbers to simulate (1..5), default 1 2")
    p.add_argument("--sides", default="AB", help="sides that answer: AB, A or B (default AB)")
    p.add_argument("--price", type=float, default=67.72, help="unit price (RSP)")
    p.add_argument("--nak", action="store_true", help="refuse every RSP write with 0x15")
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
    if any(d < 1 or d > 5 for d in a.du):
        sys.exit("--du values must be 1..5 (Yenen protocol limit)")
    if not any(c in a.sides.upper() for c in "AB"):
        sys.exit("--sides must contain A, B or both")
    if not 0 <= round(a.price * 100) <= PRICE_MAX_C:
        sys.exit("--price must be 0.00 to 99.99 (4 digits)")
    if a.log:
        _log_file = open(a.log, "a", encoding="utf-8")
    try:
        Sim(a).run()
    finally:
        if _log_file:
            _log_file.close()


if __name__ == "__main__":
    main()
