#!/usr/bin/env python3
"""
du_sim.py - Universal Dispenser Bus Simulator for ALDS LPG Gateway (alds-gg-vvm701-fw).

Simulates all 6 Dispensers (DU1..DU6, 12 Nozzles) for BOTH BlueSky and Yenen.
Automatically detects incoming queries and responds with the exact protocol,
frame format, and UART parity (8E1 for BlueSky, 8N1 for Yenen).

NO protocol flags required. Simply run:
    python tools/sim/du_sim.py --port COM13
    python tools/sim/du_sim.py --port COM13 --simulate-sales
"""

import argparse
import sys
import time
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial is missing! Run: pip install pyserial")


# --- Protocol Constants -------------------------------------------------------
BS_PREAMBLE = 0xF5
BS_CMD_STATUS = 0xD5
BS_CMD_TOTAL  = 0xC5
BS_CMD_RSP    = 0xB6
BS_CMD_KFACT  = 0xE2

YN_STX = 0x02
YN_ETX = 0x03
YN_CMD_READ = 0x41      # 'A' (Command 2)
YN_CMD_SET_RSP = 0x4B   # 'K' (RSP Write)
YN_ACK = 0x06


def yenen_checksum(data: bytes) -> bytes:
    """XOR bytes STX..ETX inclusive, emit high nibble + 0x30, low nibble + 0x30."""
    x = 0
    for b in data:
        x ^= b
    hi = ((x >> 4) & 0x0F) + 0x30
    lo = (x & 0x0F) + 0x30
    return bytes([hi, lo])


class UniversalDispenserSimulator:
    def __init__(self, port: str, baud: int = 9600, simulate_sales: bool = False):
        self.port = port
        self.baud = baud
        self.simulate_sales = simulate_sales

        self.price = 67.72           # Default unit price in Rs.
        self.kfactor = 1.0000

        # Nozzles 1..12 (DU1..DU6 x 2 nozzles each)
        self.volumes = {}
        self.amounts = {}
        for nz in range(1, 13):
            self.volumes[nz] = 1000.0 * nz + 45.20
            self.amounts[nz] = round(self.volumes[nz] * self.price, 2)

        # Start in 8E1 because gateway starts polling DU1 (BlueSky) first
        self.current_parity = serial.PARITY_EVEN

        print("=" * 75)
        print("  ALDS LPG UNIVERSAL DISPENSER SIMULATOR (DU1..DU6 BlueSky & Yenen)")
        print(f"  Port: {self.port} | Baud: {self.baud} | Auto-Detect Parity (8E1 / 8N1)")
        print("=" * 75)

        self.ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=self.current_parity,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.04
        )
        print(f"[+] Serial port {port} opened in 8E1 (listening for BlueSky/Yenen)...\n")

    def log(self, tag: str, desc: str, raw: bytes = None):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        dump = f" [{ ' '.join(f'{b:02X}' for b in raw) }]" if raw else ""
        print(f"[{ts}] {tag:14} {desc}{dump}")

    def ensure_parity(self, target_parity):
        """Switch parity only when protocol changes, with settling guard."""
        if self.current_parity != target_parity:
            # Let TX FIFO finish completely before changing UART hardware registers
            time.sleep(0.015)
            self.ser.parity = target_parity
            self.current_parity = target_parity
            # Allow line voltage / transceiver to settle
            time.sleep(0.015)

    def advance_sales(self, nz: int):
        if self.simulate_sales and nz in self.volumes:
            self.volumes[nz] += 0.50
            self.amounts[nz] = round(self.volumes[nz] * self.price, 2)

    # --------------------------------------------------------------------------
    # BlueSky Handler (Addresses 1..12, Even Parity 8E1)
    # --------------------------------------------------------------------------
    def handle_bluesky(self, rx: bytes):
        if len(rx) < 4:
            return

        addr = rx[1]
        cmd = rx[3]
        nz = addr

        if nz not in self.volumes:
            self.volumes[nz] = 1000.0 * nz + 45.20
            self.amounts[nz] = round(self.volumes[nz] * self.price, 2)

        du_num = ((nz - 1) // 2) + 1
        side_label = 'A' if (nz % 2 != 0) else 'B'

        # Ensure port is in EVEN parity (8E1) and stays in EVEN parity
        self.ensure_parity(serial.PARITY_EVEN)

        if cmd == BS_CMD_STATUS:
            # Status frame (6 bytes): F5 <addr> A3 80 00 <chk>
            # 0x80 indicates nozzle is IDLE (ready for data)
            resp = bytes([BS_PREAMBLE, addr, 0xA3, 0x80, 0x00, 0x00])
            self.ser.write(resp)
            self.ser.flush()
            self.log("[BlueSky TX]", f"DU{du_num} NZ{nz} ({side_label}) STATUS -> IDLE (0x80)", resp)

        elif cmd == BS_CMD_TOTAL:
            self.advance_sales(nz)
            vol_int = int(self.volumes[nz] * 100)
            amt_int = int(self.amounts[nz] * 100)
            vol_str = f"{vol_int:012d}"
            amt_str = f"{amt_int:012d}"
            data_bytes = (
                bytes([int(vol_str[i:i+2], 16) for i in range(0, 12, 2)]) +
                bytes([int(amt_str[i:i+2], 16) for i in range(0, 12, 2)])
            )
            # Totalizer frame: 15 bytes total (F5 + addr + AC + 12 data + chk)
            resp = bytes([BS_PREAMBLE, addr, 0xAC]) + data_bytes + bytes([0x00])
            self.ser.write(resp)
            self.ser.flush()
            self.log("[BlueSky TX]", f"DU{du_num} NZ{nz} ({side_label}) TOTAL -> {self.volumes[nz]:.2f} L / Rs.{self.amounts[nz]:.2f}", resp)

        elif cmd == BS_CMD_RSP:
            price_int = int(self.price * 100)
            p_str = f"{price_int:06d}"
            data_bytes = bytes([int(p_str[i:i+2], 16) for i in range(0, 6, 2)]) + b"\x00\x00\x00"
            # RSP frame: 9 bytes total (F5 + addr + A6 + 6 data + chk)
            resp = bytes([BS_PREAMBLE, addr, 0xA6]) + data_bytes + bytes([0x00])
            self.ser.write(resp)
            self.ser.flush()
            self.log("[BlueSky TX]", f"DU{du_num} NZ{nz} ({side_label}) RSP -> Rs.{self.price:.2f}", resp)

        elif cmd == BS_CMD_KFACT:
            # K-Factor frame: 7 bytes total (F5 + addr + A4 + 4 data + chk)
            resp = bytes([BS_PREAMBLE, addr, 0xA4, 0x00, 0x01, 0x00, 0x00, 0x00])
            self.ser.write(resp)
            self.ser.flush()
            self.log("[BlueSky TX]", f"DU{du_num} NZ{nz} ({side_label}) K-FACTOR -> {self.kfactor}", resp)

        else:
            # Command ack
            resp = bytes([BS_PREAMBLE, addr, 0xA2, cmd, 0x00])
            self.ser.write(resp)
            self.ser.flush()
            self.log("[BlueSky TX]", f"DU{du_num} NZ{nz} CMD 0x{cmd:02X} ACK", resp)

    # --------------------------------------------------------------------------
    # Yenen Handler (Heads 1..6, No Parity 8N1)
    # --------------------------------------------------------------------------
    def handle_yenen(self, rx: bytes):
        if len(rx) < 8:
            return

        try:
            head = int(chr(rx[4]))
        except ValueError:
            head = 1
        nzcode = rx[5]
        cmd = rx[6]

        # Side calculation:
        # Head 1..5: Side A = '0'+(head-1), Side B = '5'+(head-1)
        # Head 6:    Side A = '5',          Side B = ':'
        if head == 6:
            side = 1 if nzcode == ord(':') else 0
        else:
            side = 1 if nzcode >= ord('5') else 0

        side_label = 'B' if side else 'A'
        nz = (head - 1) * 2 + side + 1

        if nz not in self.volumes:
            self.volumes[nz] = 1000.0 * nz + 45.20
            self.amounts[nz] = round(self.volumes[nz] * self.price, 2)

        # Switch to NONE parity (8N1) and stay in NONE parity
        self.ensure_parity(serial.PARITY_NONE)

        if cmd == YN_CMD_READ:  # Command 2 ('A')
            self.advance_sales(nz)
            vol_int = int(self.volumes[nz] * 100)
            amt_int = int(self.amounts[nz] * 100)
            price_int = int(self.price * 100)

            # ASCII data block (46 chars between STX and ETX)
            ascii_block = (
                f"490{head}{chr(nzcode)}A"
                f"{price_int:04d}"
                f"00000000"
                f"00000000"
                f"{vol_int:08d}"
                f"{amt_int:07d}"
                f"000990"
            ).encode("ascii")

            frame_body = bytes([YN_STX]) + ascii_block + bytes([YN_ETX])
            ck = yenen_checksum(frame_body)
            full_resp = frame_body + ck

            self.ser.write(full_resp)
            self.ser.flush()
            self.log("[Yenen TX]", f"DU{head} Head {head}/{side_label} (NZ{nz}) TOTAL -> {self.volumes[nz]:.2f} L / Rs.{self.amounts[nz]:.2f}", full_resp)

        elif cmd == YN_CMD_SET_RSP:  # Command 'K' (RSP Write)
            if len(rx) >= 12:
                try:
                    new_p = int(rx[7:11].decode("ascii")) / 100.0
                    self.price = new_p
                    self.log("[Yenen RSP]", f"Updated station unit price to Rs.{self.price:.2f}")
                except Exception:
                    pass
            self.ser.write(bytes([YN_ACK]))
            self.ser.flush()
            self.log("[Yenen TX]", f"DU{head} Head {head}/{side_label} RSP WRITE ACK (0x06)", bytes([YN_ACK]))

    # --------------------------------------------------------------------------
    # Main Event Loop
    # --------------------------------------------------------------------------
    def run(self):
        buf = bytearray()
        last_rx_time = time.time()

        try:
            while True:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    chunk = self.ser.read(waiting)
                    buf.extend(chunk)
                    last_rx_time = time.time()

                if len(buf) > 0 and (time.time() - last_rx_time > 0.025 or len(buf) >= 10):
                    rx = bytes(buf)
                    buf.clear()

                    # Find BlueSky frame: search for 0xF5 preamble
                    f5_idx = rx.find(bytes([BS_PREAMBLE]))
                    if f5_idx != -1 and len(rx) >= f5_idx + 4:
                        frame = rx[f5_idx:f5_idx + 5]
                        du_idx = ((frame[1] - 1) // 2) + 1
                        self.log("[BlueSky RX]", f"DU{du_idx} NZ{frame[1]} CMD 0x{frame[3]:02X}", frame)
                        self.handle_bluesky(frame)
                        continue

                    # Find Yenen frame: search for STX (0x02)
                    stx_idx = rx.find(bytes([YN_STX]))
                    if stx_idx != -1 and len(rx) >= stx_idx + 8:
                        frame = rx[stx_idx:]
                        cmd_c = chr(frame[6]) if len(frame) > 6 else '?'
                        head_c = chr(frame[4]) if len(frame) > 4 else '?'
                        self.log("[Yenen RX]", f"DU{head_c} Head {head_c} CMD '{cmd_c}'", frame[:10])
                        self.handle_yenen(frame)
                        continue

                time.sleep(0.005)

        except KeyboardInterrupt:
            print("\n[*] Stopping dispenser simulator.")
        finally:
            self.ser.close()


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports detected!")
    else:
        print("Available serial ports on this PC:")
        for p in ports:
            print(f"  {p.device:10} - {p.description}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="ALDS LPG Universal Dispenser Simulator (BlueSky & Yenen)")
    parser.add_argument("--port", "-p", help="Serial port (e.g. COM13 or /dev/ttyUSB0)")
    parser.add_argument("--baud", "-b", type=int, default=9600, help="Baud rate (default: 9600)")
    parser.add_argument("--simulate-sales", "-s", action="store_true", help="Auto-increment volume on each poll")
    parser.add_argument("--list", "-l", action="store_true", help="List serial ports and exit")

    args = parser.parse_args()

    if args.list:
        list_ports()
        sys.exit(0)

    if not args.port:
        print("Error: --port argument is required! Example: python tools/sim/du_sim.py --port COM13")
        list_ports()
        sys.exit(1)

    sim = UniversalDispenserSimulator(port=args.port, baud=args.baud, simulate_sales=args.simulate_sales)
    sim.run()
