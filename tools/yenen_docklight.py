#!/usr/bin/env python3
"""Emit Yenen DU1 (nozzle 1 + nozzle 2) TX/RX frames in Docklight paste format.

Same checksum/layout code as yenen_gen.py — space-separated uppercase hex is
what Docklight's Send/Receive Sequence editor accepts in HEX mode.
"""

STX, ETX = 0x02, 0x03

def cksum(b):
    x = 0
    for c in b:
        x ^= c
    return bytes([((x >> 4) & 0x0F) + 0x30, (x & 0x0F) + 0x30])

def nzcode(head, side):
    return (ord('5') if side else ord('0')) + (head - 1)

def tx_frame(head, side):
    body = bytes([STX, ord('0'), ord('8'), ord('0'),
                  ord('0') + head, nzcode(head, side), ord('A'), ETX])
    return body + cksum(body)

def payload(head, side, price, p_amt, p_vol, d_lit, d_amt, t_lit, t_amt, status):
    s = ("49" + "0" + str(head) + chr(nzcode(head, side)) + "A"
         + f"{price:04d}" + f"{p_amt:04d}" + f"{p_vol:04d}"
         + f"{d_lit:03d}" + f"{d_amt:06d}"
         + f"{t_lit:06d}" + f"{t_amt:08d}"
         + f"{status:02d}" + "0990")
    assert len(s) == 47, len(s)
    return s

def rx_frame(**kw):
    p = payload(**kw)
    body = bytes([STX]) + p.encode() + bytes([ETX])
    return body + cksum(body), p

def dl(b):                       # Docklight HEX paste format
    return " ".join(f"{x:02X}" for x in b)

# ── self-test against the vendor doc ────────────────────────────────────────
assert dl(tx_frame(1, 0)) == "02 30 38 30 31 30 41 03 37 39"
assert dl(tx_frame(1, 1)) == "02 30 38 30 31 35 41 03 37 3C"
assert payload(1, 0, 6772, 260, 0, 359, 24311, 874333, 22182451, 11) == \
       "49010A67720260000035902431187433322182451110990"

PRICE1, PRICE2 = 6772, 8250      # 67.72 /L petrol, 82.50 /L diesel

# (label, preset_vol x100, delivery L x100, delivery amt x100,
#  totaliser L x100, totaliser amt x100, status)
#
# preset_vol is 0 everywhere except NZ1_S4. S4 is the "no activity since the
# last poll" case, so its totaliser MUST equal S3's - that is the whole point of
# the frame. Left alone the two frames come out byte-identical, so S4 carries a
# distinct preset volume instead: it is not part of the sale derivation (the
# cloud differences totalisers, Appendix B rule 2), so the frame stays a valid
# zero-delta test while remaining unique on the wire. Every RX frame below is
# asserted unique before printing.
SEQ1 = [
    ("NZ1_S0_IDLE_BASE",   0,  359, 24311, 874333, 22182451,  0),
    ("NZ1_S1_DISPENSING",  0,  250, 16930, 874583, 22199381, 11),
    ("NZ1_S2_DISPENSING",  0,  500, 33860, 874833, 22216311, 11),
    ("NZ1_S3_SALE1_DONE",  0,  500, 33860, 874833, 22216311,  0),
    ("NZ1_S4_IDLE_NOCHG", 1000, 500, 33860, 874833, 22216311,  0),
    ("NZ1_S5_DISPENSING",  0,  800, 54176, 875633, 22270487, 11),
    ("NZ1_S6_SALE2_DONE",  0,  800, 54176, 875633, 22270487,  0),
]
SEQ2 = [
    ("NZ2_S0_IDLE_BASE",   0,  400, 33000, 512900, 15003300,  0),
    ("NZ2_S1_DISPENSING",  0,  600, 49500, 513500, 15052800, 11),
    ("NZ2_S2_SALE_DONE",   0,  600, 49500, 513500, 15052800,  0),
]

ALL_RX = {}   # frame bytes -> name, to prove uniqueness

for title, head, side, price, seq in (
        ("NOZZLE 1 — DU1 NZ1, head 1 side A", 1, 0, PRICE1, SEQ1),
        ("NOZZLE 2 — DU1 NZ2, head 1 side B", 1, 1, PRICE2, SEQ2)):
    print(f"\n{'='*70}\n{title}   unit price {price/100:.2f}/L\n{'='*70}")
    print(f"RECEIVE SEQ  POLL_{'NZ1' if not side else 'NZ2'}   {dl(tx_frame(head, side))}   (10 B)")
    print("SEND SEQUENCES (51 B each):")
    prev = None
    for name, p_v, d_l, d_a, t_l, t_a, st in seq:
        fr, p = rx_frame(head=head, side=side, price=price, p_amt=0, p_vol=p_v,
                         d_lit=d_l, d_amt=d_a, t_lit=t_l, t_amt=t_a, status=st)
        assert len(fr) == 51
        assert fr not in ALL_RX, f"{name} is byte-identical to {ALL_RX.get(fr)}"
        ALL_RX[fr] = name
        note = "DISPENSING - cloud must ignore" if st else "idle"
        if st == 0 and prev is not None:
            dL, dA = (t_l - prev[0]) / 100, (t_a - prev[1]) / 100
            note = (f"idle -> SALE {dL:.2f} L / {dA:.2f}"
                    if (dL or dA) else "idle, zero delta -> NO transaction")
        if st == 0:
            prev = (t_l, t_a)
        print(f"\n  {name}   [{note}]")
        print(f"  {dl(fr)}")
        print(f"  ASCII {p}   tot {t_l/100:.2f} L / {t_a/100:.2f}")

print()
print(f"{'='*70}")
print(f"UNIQUENESS: {len(ALL_RX)} RX frames, all byte-unique.")
print(f"TX frames:  {dl(tx_frame(1,0))} / {dl(tx_frame(1,1))} - distinct.")
print(f"{'='*70}")
