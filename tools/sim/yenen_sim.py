#!/usr/bin/env python3
"""
yenen_sim.py - Yenen dispenser slave for alds-gg-vvm701-fw bench tests.

Answers the gateway's Yenen polls on a serial port (RS232 or RS485 adapter),
9600 8N1, byte-exact to what src/yenen.c validates:

    Command 2 (read)  : 02 '0' '8' '0' <head> <nz> 'A' 03 <ck_hi> <ck_lo>
    reply (51 bytes)  : 02 <47 ASCII chars> 03 <ck_hi> <ck_lo>
    Command K (RSP)   : 02 '0' '8' '0' <head> <nz> 'K' p1 p2 p3 p4 03 <ck_hi> <ck_lo>
    reply             : 06 30 31 (ACK)  or  15 (NAK, --nak)

    checksum : XOR of STX..ETX, each nibble + 0x30 (0x7C -> '7' '<')
    <head>   : '0' + head (1..5)
    <nz>     : side A '0'+(head-1), side B '5'+(head-1)

47-char reply payload (CLOUD_INTEGRATION_GUIDE.md appendix, tools/yenen_gen.py):
    "49" "0"<head><nz> "A" price4 preset_amt4 preset_vol4 deliv_l3 deliv_amt6
    total_l6 total_amt8 status2 "0990"          status "00" idle, "11" dispensing

Yenen dispenser N is head N (both sides).

    python tools/sim/yenen_sim.py --port COM5                    # head 1, sides A+B
    python tools/sim/yenen_sim.py --port COM5 --du 1 2           # heads 1 and 2
    python tools/sim/yenen_sim.py --port COM5 --sides A          # single-sided DU, B silent
    python tools/sim/yenen_sim.py --port COM5 --sale-every 90    # random sale, dispensing 25-60 s
    python tools/sim/yenen_sim.py --port COM5 --fault 5          # 5% corrupted replies
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

STX, ETX, ACK, NAK = 0x02, 0x03, 0x06, 0x15


def cksum(frame: bytes) -> bytes:
    x = 0
    for b in frame:
        x ^= b
    return bytes([((x >> 4) & 0x0F) + 0x30, (x & 0x0F) + 0x30])


def nzcode(head: int, side: int) -> int:
    return (ord('5') if side else ord('0')) + head - 1


def hexs(b: bytes) -> str:
    return ",".join(f"{v:02x}" for v in b)


def log(msg: str):
    print(f"[{datetime.now().strftime('%H:%M:%S.%f')[:-3]}] {msg}", flush=True)


class Nozzle:
    def __init__(self, head: int, side: int, price: float):
        self.head, self.side = head, side
        self.name = f"head{head}/{'B' if side else 'A'}"
        self.price_c = int(round(price * 100))                 # paise
        self.tot_l = 874_333 + (head * 2 + side) * 10_000      # litres x100, 6 digits
        self.tot_amt = self.tot_l * self.price_c // 100        # rupees x100, 8 digits
        self.del_l = 0                                         # last delivery litres x100
        self.del_amt = 0
        self.sale = None                                       # (start, end, litres x100)

    def start_sale(self, litres: float, seconds: float):
        now = time.time()
        self.sale = (now, now + seconds, int(litres * 100), self.tot_l, self.tot_amt)

    def update(self) -> str:
        """Advance any running sale; return the status field."""
        if not self.sale:
            return "00"
        t0, t1, lit, base_l, base_amt = self.sale
        frac = min(1.0, (time.time() - t0) / (t1 - t0))
        self.del_l = int(lit * frac)
        self.del_amt = self.del_l * self.price_c // 100
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
             f"{self.price_c % 10000:04d}" "0000" "0000"
             f"{self.del_l % 1000:03d}{self.del_amt % 1_000_000:06d}"
             f"{self.tot_l % 1_000_000:06d}{self.tot_amt % 100_000_000:08d}"
             f"{st}0990")
        assert len(s) == 47, s
        return s


class Sim:
    def __init__(self, a):
        self.nz = {}                                           # (head_char, nz_char) -> Nozzle
        sides = [s for s, c in ((0, "A"), (1, "B")) if c in a.sides.upper()]
        for head in a.du:
            for side in sides:
                self.nz[(ord('0') + head, nzcode(head, side))] = Nozzle(head, side, a.price)
        self.nak = a.nak
        self.fault = a.fault / 100.0
        self.sale_every = a.sale_every
        self.next_sale = time.time() + a.sale_every if a.sale_every else None
        self.delay = a.delay / 1000.0
        self.last_tx = b""                                     # to drop our own echo
        self.last_heard = time.time()
        self.ser = serial.Serial(a.port, a.baud, bytesize=8, parity=serial.PARITY_NONE,
                                 stopbits=1, timeout=0.02)
        log(f"{a.port} {a.baud} 8N1, nozzles {[n.name for n in self.nz.values()]}, "
            f"RSP {a.price:.2f}{', RSP writes NAKed' if a.nak else ''}")

    def maybe_sale(self):
        if self.next_sale and time.time() >= self.next_sale:
            self.next_sale = time.time() + self.sale_every
            idle = [n for n in self.nz.values() if not n.sale]
            if not idle:
                return
            n = random.choice(idle)
            litres = round(random.uniform(2, 9.9), 2)          # delivery field caps at 9.99 L
            secs = random.uniform(25, 60)
            n.start_sale(litres, secs)
            log(f"{n.name} sale started: {litres:.2f} L, dispensing for {secs:.0f} s")

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
        if f == self.last_tx:                                  # our own reply echoed back
            self.last_tx = b""
            return
        if f[-2:] != cksum(f[:-2]):
            log(f"RX {hexs(f)}  bad checksum - silent")
            return
        if len(f) < 10 or f[1:4] != b"080":
            log(f"RX {hexs(f)}  not a Yenen request - silent")
            return
        n = self.nz.get((f[4], f[5]))
        if n is None:
            log(f"RX {hexs(f)}  head '{chr(f[4])}' nz '{chr(f[5])}' not simulated - silent")
            return
        cmd = f[6]
        log(f"RX {hexs(f)}  {n.name} cmd '{chr(cmd)}'")

        if cmd == ord('A') and len(f) == 10:
            p = n.payload()
            body = bytes([STX]) + p.encode() + bytes([ETX])
            st = "DISPENSING" if p[41:43] != "00" else "idle"
            self.send(body + cksum(body),
                      f"{n.name} {st}, RSP {n.price_c / 100:.2f}, "
                      f"total {n.tot_l / 100:.2f} L / Rs {n.tot_amt / 100:.2f}")
        elif cmd == ord('K') and len(f) == 14:
            digits = f[7:11].decode(errors="replace")
            if self.nak or not digits.isdigit():
                self.send(bytes([NAK]), f"{n.name} RSP write {digits} NAK")
            else:
                n.price_c = int(digits)
                self.send(bytes([ACK, 0x30, 0x31]), f"{n.name} RSP set to {n.price_c / 100:.2f}, ACK")
        else:
            log(f"{n.name} unknown command '{chr(cmd)}' ({len(f)} bytes) - silent")

    def run(self):
        buf = bytearray()
        try:
            while True:
                self.maybe_sale()
                chunk = self.ser.read(64)
                if chunk:
                    log(f"raw {hexs(chunk)}")                  # every byte seen, framed or not
                    buf += chunk
                elif time.time() - self.last_heard > 15:
                    log("nothing received for 15 s - check port, wiring, converter power")
                    self.last_heard = time.time()
                if chunk:
                    self.last_heard = time.time()
                while True:
                    i = buf.find(STX)
                    if i < 0:
                        buf.clear()
                        break
                    del buf[:i]
                    e = buf.find(ETX, 1)
                    if e < 0:
                        if len(buf) > 64:                      # no ETX in sight, resync
                            del buf[0]
                            continue
                        break
                    if len(buf) < e + 3:
                        break
                    frame = bytes(buf[:e + 3])
                    del buf[:e + 3]
                    self.handle(frame)
        except KeyboardInterrupt:
            log("stopped")
        finally:
            self.ser.close()


def main():
    p = argparse.ArgumentParser(description="Yenen dispenser slave simulator")
    p.add_argument("--port", "-p", help="serial port, e.g. COM5")
    p.add_argument("--baud", "-b", type=int, default=9600)
    p.add_argument("--du", type=int, nargs="+", default=[1],
                   help="Yenen dispenser (head) numbers to simulate (1..5), default 1")
    p.add_argument("--sides", default="AB", help="sides that answer: AB, A or B (default AB)")
    p.add_argument("--price", type=float, default=67.72, help="unit price (RSP)")
    p.add_argument("--nak", action="store_true", help="reject every RSP write with 0x15")
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
    if any(d < 1 or d > 5 for d in a.du):
        sys.exit("--du values must be 1..5 (Yenen protocol limit)")
    Sim(a).run()


if __name__ == "__main__":
    main()
