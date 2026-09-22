#!/usr/bin/env python3
"""Generate Yenen Command-2 TX/RX test vectors with correct checksums."""

STX, ETX = 0x02, 0x03

def cksum(frame_bytes):
    x = 0
    for b in frame_bytes:
        x ^= b
    return bytes([((x >> 4) & 0x0F) + 0x30, (x & 0x0F) + 0x30])

def tx_frame(head, side):
    nz = (ord('5') if side else ord('0')) + (head - 1)
    body = bytes([STX, ord('0'), ord('8'), ord('0'), ord('0') + head, nz, ord('A'), ETX])
    return body + cksum(body)

def payload(head, side, price, p_amt, p_vol, d_lit, d_amt, t_lit, t_amt, status):
    nzc = chr((ord('5') if side else ord('0')) + (head - 1))
    s = ("49" + "0" + str(head) + nzc + "A"
         + f"{price:04d}" + f"{p_amt:04d}" + f"{p_vol:04d}"
         + f"{d_lit:03d}" + f"{d_amt:06d}"
         + f"{t_lit:06d}" + f"{t_amt:08d}"
         + f"{status:02d}" + "0990")
    assert len(s) == 47, f"payload is {len(s)} chars, must be 47"
    return s

def rx_frame(**kw):
    p = payload(**kw)
    body = bytes([STX]) + p.encode() + bytes([ETX])
    return body + cksum(body), p

def hx(b):
    return ",".join(f"{x:02x}" for x in b)

# ── self-test against the vendor-documented frames ───────────────────────────
DOC_TX = {
    (1, 0): "02 30 38 30 31 30 41 03 37 39",
    (1, 1): "02 30 38 30 31 35 41 03 37 3C",
    (2, 0): "02 30 38 30 32 31 41 03 37 3B",
    (2, 1): "02 30 38 30 32 36 41 03 37 3C",
    (3, 0): "02 30 38 30 33 32 41 03 37 39",
    (5, 1): "02 30 38 30 35 39 41 03 37 34",
}
print("=== SELF-TEST: TX vs vendor doc ===")
for (h, s), want in DOC_TX.items():
    got = " ".join(f"{b:02X}" for b in tx_frame(h, s))
    print(f"  head{h}/{'B' if s else 'A'}: {got}   {'OK' if got == want else 'MISMATCH want ' + want}")

DOC_PAYLOAD = "49010A67720260000035902431187433322182451110990"
mine = payload(1, 0, 6772, 260, 0, 359, 24311, 874333, 22182451, 11)
print(f"\n=== SELF-TEST: payload layout ===\n  {'OK' if mine == DOC_PAYLOAD else 'MISMATCH'}  {mine}")

# ── scenario ─────────────────────────────────────────────────────────────────
PRICE1 = 6772   # 67.72 /L  (petrol)  nozzle 1 = head1 sideA
PRICE2 = 8250   # 82.50 /L  (diesel)  nozzle 2 = head1 sideB

seq1 = [
    ("T0  idle baseline",        359, 24311, 874333, 22182451,  0),
    ("T1  DISPENSING",           250, 16930, 874583, 22199381, 11),
    ("T2  DISPENSING",           500, 33860, 874833, 22216311, 11),
    ("T3  idle  <- SALE 1 done", 500, 33860, 874833, 22216311,  0),
    ("T4  idle  (no activity)",  500, 33860, 874833, 22216311,  0),
    ("T5  DISPENSING",           800, 54176, 875633, 22270487, 11),
    ("T6  idle  <- SALE 2 done", 800, 54176, 875633, 22270487,  0),
]
seq2 = [
    ("T0  idle baseline",        400, 33000, 512900, 15003300,  0),
    ("T1  DISPENSING",           600, 49500, 513500, 15052800, 11),
    ("T2  idle  <- SALE done",   600, 49500, 513500, 15052800,  0),
]

for label, head, side, price, seq in (("NOZZLE 1  (DU1 NZ1, head1 sideA)", 1, 0, PRICE1, seq1),
                                      ("NOZZLE 2  (DU1 NZ2, head1 sideB)", 1, 1, PRICE2, seq2)):
    print(f"\n\n{'='*78}\n{label}   unit price {price/100:.2f}/L")
    print(f"TX (every poll, identical): {hx(tx_frame(head, side))}")
    print(f"{'='*78}")
    prev = None
    for name, dl, da, tl, ta, st in seq:
        fr, p = rx_frame(head=head, side=side, price=price, p_amt=0, p_vol=0,
                         d_lit=dl, d_amt=da, t_lit=tl, t_amt=ta, status=st)
        assert len(fr) == 51, len(fr)
        print(f"\n{name}   status={st:02d} {'(IGNORE)' if st else ''}")
        print(f"  RX  : {hx(fr)}")
        print(f"  ASCII: {p}")
        print(f"  tot : {tl/100:10.2f} L   {ta/100:12.2f}   last delivery {dl/100:.2f} L / {da/100:.2f}")
        if st == 0:
            if prev is not None:
                dL, dA = (tl - prev[0]) / 100, (ta - prev[1]) / 100
                if dL or dA:
                    print(f"  ==> SALE: {dL:.2f} L , {dA:.2f}   (check {dL:.2f} x {price/100:.2f} = {dL*price/100:.2f})")
                else:
                    print(f"  ==> no change -> NO transaction")
            prev = (tl, ta)

# ── MQTT messages exactly as the device publishes them ───────────────────────
IMEI = "868020030001234"
print(f"\n\n{'='*78}\nMQTT  topic: fuelsense/data     HW_ID = IMEI + sfxYn1(51)\n{'='*78}")
import datetime
t = datetime.datetime(2026, 8, 21, 9, 14, 0)
rows = []
for i, (name, dl, da, tl, ta, st) in enumerate(seq1):
    fr, _ = rx_frame(head=1, side=0, price=PRICE1, p_amt=0, p_vol=0,
                     d_lit=dl, d_amt=da, t_lit=tl, t_amt=ta, status=st)
    rows.append((t + datetime.timedelta(seconds=20 * i), "1", fr, name))
for i, (name, dl, da, tl, ta, st) in enumerate(seq2):
    fr, _ = rx_frame(head=1, side=1, price=PRICE2, p_amt=0, p_vol=0,
                     d_lit=dl, d_amt=da, t_lit=tl, t_amt=ta, status=st)
    rows.append((t + datetime.timedelta(seconds=20 * i + 2), "2", fr, name))
for ts, lbl, fr, name in sorted(rows, key=lambda r: r[0]):
    print(f'{{"HW_ID":"{IMEI}51","nozzles":{{"{lbl}":{{"data":"{hx(fr)}"}}}},"TS":"{ts:%Y-%m-%d %H:%M:%S}"}}')
