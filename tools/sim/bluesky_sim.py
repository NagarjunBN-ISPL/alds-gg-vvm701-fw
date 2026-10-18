#!/usr/bin/env python3
"""
bluesky_sim.py - BlueSky dispenser slave for alds-gg-vvm701-fw bench tests.

Answers the gateway's BlueSky polls on a serial port (RS232 or RS485 adapter),
9600 8E1, byte-exact to what src/dispenser.c validates:

    request : F5 <addr> A2 <cmd> <chk>                 chk = addr ^ cmd ^ 0xD7
    reply   : F5 <addr> A<n> <data..> <cmd> <chk>      n = bytes after byte 2
                                                       chk = XOR of all prior bytes & 0x7F
    D5 status   : 1 byte   0x80 idle, 0x81 delivering
    C5 total    : 10 BCD   5 bytes litres x100, 5 bytes amount x100
    B6 RSP      : 2 BCD    price x100          (65.82 -> 65 82)
    E2 k-factor : 2 bytes  raw, echoed as set  (default 23 60)
    writes      : any frame longer than 5 bytes, built by the cloud and forwarded
                  unread by the gateway's rsp/kfactor tunnel (src/rsp_tunnel.c).
                  B2 sets RSP, the station-proven layout (2026-09-29):
                      664: F5 <addr> A4 <RR> <PP> B2 <chk>
                      886: F5 <addr> A5 00 <RR> <PP> B2 <chk>
                  E3 sets k-factor (GUESS, --kf-write-cmd). Every write is
                  answered like the station pump: F5 <addr> A3 59 <cmd> <chk>.

A BlueSky dispenser N owns nozzle addresses 2N-1 and 2N.

    python tools/sim/bluesky_sim.py --port COM13                    # DU1 (nz 1,2)
    python tools/sim/bluesky_sim.py --port COM13 --du 1 2           # nz 1..4
    python tools/sim/bluesky_sim.py --port COM13 --sale-every 90    # random sale, delivering 25-60 s
    python tools/sim/bluesky_sim.py --port COM13 --fault 5          # 5% corrupted replies
"""

import argparse
import random
import sys
import time
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial is missing: pip install pyserial")

PRE = 0xF5
CMD_STATUS, CMD_TOTAL, CMD_RSP, CMD_KFACT = 0xD5, 0xC5, 0xB6, 0xE2
CMD_RSP_WRITE = 0xB2
WRITE_ACK = b"\x59"          # the station pump's ack data byte: F5 nz A3 59 <cmd> chk


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


def hexs(b: bytes) -> str:
    return ",".join(f"{v:02x}" for v in b)


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

    def on_status(self) -> int:
        if self.sale_end:
            if time.time() < self.sale_end:
                return 0x81
            self.sale_end = 0.0                       # sale settles on this poll
            self.vol_c += self.sale_vol_c
            self.amt_c += self.sale_vol_c * self.price_c // 100
            log(f"nz{self.addr} sale done: +{self.sale_vol_c / 100:.2f} L "
                f"-> total {self.vol_c / 100:.2f} L / Rs {self.amt_c / 100:.2f}")
        return 0x80


def log(msg: str):
    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {msg}", flush=True)


class Sim:
    def __init__(self, a):
        self.nz = {}
        kf = bytes.fromhex(a.kfactor)
        for du in a.du:
            for addr in (2 * du - 1, 2 * du):
                self.nz[addr] = Nozzle(addr, a.price, kf)
        self.kf_write_cmd = int(a.kf_write_cmd, 16)
        self.fault = a.fault / 100.0
        self.sale_every = a.sale_every
        self.next_sale = time.time() + a.sale_every if a.sale_every else None
        self.delay = a.delay / 1000.0
        self.last_tx = b""                            # to drop our own echo (RS485 adapters)
        self.ser = serial.Serial(a.port, a.baud, bytesize=8, parity=serial.PARITY_EVEN,
                                 stopbits=1, timeout=0.02)
        log(f"{a.port} {a.baud} 8E1, nozzles {sorted(self.nz)}, "
            f"RSP {a.price:.2f}, kfact {kf.hex()}")

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
        addr, cmd = f[1], f[3]
        n = self.nz.get(addr)
        if n is None:
            log(f"RX {hexs(f)}  nz{addr} not simulated - silent")
            return
        if len(f) == 5 and f[4] != (addr ^ cmd ^ 0xD7):
            log(f"RX {hexs(f)}  bad request checksum - silent")
            return
        log(f"RX {hexs(f)}")

        if len(f) > 5:                                # write
            cmd = f[-2]                               # the command byte sits before chk
            data = f[3:-2]
            if cmd == CMD_RSP_WRITE and len(data) >= 2 and data[-2:].hex().isdigit():
                n.price_c = int(data[-2:].hex())      # RR PP, last two data bytes (664 and 886)
                note = f"RSP set to {n.price_c / 100:.2f}"
            elif cmd == self.kf_write_cmd and len(data) >= 2:
                n.kf = bytes(data[-2:])
                note = f"k-factor set to {n.kf.hex()}"
            else:
                note = f"write 0x{cmd:02x} data {hexs(data)} (ignored)"
            self.send(reply(addr, cmd, WRITE_ACK), f"nz{addr} ACK, {note}")
        elif cmd == CMD_STATUS:
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

    def run(self):
        buf = bytearray()
        try:
            while True:
                self.maybe_sale()
                buf += self.ser.read(64)
                while True:
                    i = buf.find(PRE)
                    if i < 0:
                        buf.clear()
                        break
                    del buf[:i]
                    if len(buf) < 3:
                        break
                    if (buf[2] & 0xF0) != 0xA0:
                        del buf[0]                    # not a header, resync
                        continue
                    ln = (buf[2] & 0x0F) + 3
                    if len(buf) < ln:
                        break
                    frame = bytes(buf[:ln])
                    del buf[:ln]
                    self.handle(frame)
        except KeyboardInterrupt:
            log("stopped")
        finally:
            self.ser.close()


def main():
    p = argparse.ArgumentParser(description="BlueSky dispenser slave simulator")
    p.add_argument("--port", "-p", help="serial port, e.g. COM13")
    p.add_argument("--baud", "-b", type=int, default=9600)
    p.add_argument("--du", type=int, nargs="+", default=[1],
                   help="BlueSky dispenser numbers to simulate (1..6), default 1")
    p.add_argument("--price", type=float, default=65.82, help="unit price (RSP)")
    p.add_argument("--kfactor", default="2360", help="k-factor raw bytes, hex (default 2360)")
    p.add_argument("--kf-write-cmd", default="E3",
                   help="opcode treated as a k-factor write (hex, default E3 - unconfirmed)")
    p.add_argument("--sale-every", type=float, default=0,
                   help="start a random sale every N seconds (0 = off)")
    p.add_argument("--fault", type=float, default=0, help="percent of replies with a bit flipped")
    p.add_argument("--delay", type=float, default=20, help="reply delay in ms (default 20)")
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
    Sim(a).run()


if __name__ == "__main__":
    main()
