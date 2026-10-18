# Genset register maps

Companion to `CLOUD_INTEGRATION_GUIDE.md` section 6.4, which defines the
`genset` messages themselves. This file only covers how to decode the bytes
inside them. You need it only on sites where `gensetType` is 1.

Each message is the complete Modbus reply as the controller sent it:
`frame[0]` slave (the genset suffix), `frame[1]` function code, `frame[2]` byte
count, then the register data, then the CRC (low byte, high byte). The gateway
has checked CRC, slave, function code and length before publishing.

`register[k] = frame[3+2k]<<8 | frame[3+2k+1]`, 0-based within the message. The
`registers` read starts at Modbus offset 1, so `register[k]` is protocol offset
`k+1` (in `Alarms` it is `k+57`, in `status` `k+67`). For signed fields,
subtract 65536 if the value is above 32767. For 32-bit fields,
`(register[lo]<<16) | register[hi]`, where the lower index is the high word.
The Ashok Leyland capture confirms that order: `0x0009, 0x6D3A` is 61778.6 kWh,
while the other order would be 183 million kWh.

Which map applies depends on `gensetModel`. The model-to-controller table, and
which models send `Alarms` and `status` at all, is in section 6.4 of the
integration guide; that table is the single source of truth for both.

Values marked as captured were checked on 23 September 2026 against the frames
in the DG integration spec: a Mahindra GC1211/GC1111 running unloaded, an Ashok
Leyland at 16.7 kW, and an SGC 120 running unloaded. Everything else is
transcribed from vendor documents and has not been seen on a live unit.

## Map 1: Mahindra GC1211/GC1111, Ashok Leyland GC1201/02/03, Sedemac GC902

36 registers, offsets 1-36:

```
0  Oil pressure (0.01 Bar)        9-11   Gen L1/L2/L3-N voltage (0.1 V)
1  Engine temp (1 C)              12-14  Gen L1-L2/L2-L3/L3-L1 voltage (0.1 V)
2  Fuel level (1 %)               15-17  Load L1/L2/L3 current (see below)
3  Charging alt. voltage (0.1 V)  18-21  Gen L1/L2/L3/total active power (0.1 kW)
4  Battery voltage (0.1 V)        22-25  Gen L1/L2/L3/total apparent power (0.1 kVA)
5  Number of starts               26-29  Gen L1/L2/L3/total reactive power (0.1 kVAr)
6  Engine run time (hours)        30-33  Gen L1/L2/L3/avg power factor (0.01)
7  Engine speed (RPM)             34-35  Cumulative energy, 32-bit (0.1 kWh)
8  Genset frequency (0.1 Hz)
```

Captured (Ashok Leyland): 4.80 Bar, 89 C, 40 %, 13.9 V battery, 5371 starts,
3712 h, 1501 RPM, 49.9 Hz, 240.8 V L1-N, 416.3 V L1-L2, PF 0.82/0.79/0.75/0.79
(each equals kW / kVA).

**Load current scale: use 0.1 A, not the documented 0.01 A.** At 0.01 A the
Ashok Leyland L1 current is 3.05 A, and 3.05 A x 240.8 V is 0.73 kVA, but
register 22 reads 7.3 kVA. At 0.1 A (30.5 A) every power register agrees
(V x I = 7.3 kVA, V x I x PF = 6.0 kW), and so does the energy counter:
61778.6 kWh over 3712 h is an average of 16.6 kW, against the 16.7 kW now.
Confirm on a Mahindra under load; its capture had no load.

`Alarms`, Mahindra only: 3 registers, offsets 57-59. A healthy, running GC1211
sent `0a,04,06,11,10,11,11,11,11,48,44` (data `11,10,11,11,11,11`). The bit layout is in the Sedemac protocol document,
not in this repo. Decode from that document rather than guessing from the
capture.

`status`: Mahindra sends 2 registers (`[0]` is the DG status word at offset 67,
`[1]` the command status at offset 68), while Ashok Leyland and Sedemac send
only `[0]`. DG status bits: 15 available/loadOnDG, 14 running, 13
stoppedNormally, 12 stoppedWithFault, 11 mode (1 = Auto). Command status,
Mahindra only: 1 Started, 2 Stopped, 3 AlreadyON, 4 AlreadyOFF, 5 Manual,
6 NotStarted, anything else Unknown. Captured: Mahindra `0a,04,04,c0,00,00,00,7d,44`
(0xC000, available and running) and Ashok Leyland `0a,04,02,40,00,2d,31` (0x4000,
running).

Worked example: `registers` of `0a,04,48,01,3f,00,3e,00,22,...` is slave 10, FC
04, 72 data bytes, then oil 0x013F = 319 = 3.19 Bar, temp 0x003E = 62 C, fuel
0x0022 = 34 %. A `status` of `0a,04,04,c0,00,00,00,7d,44` is 0xC000, bits 15 and
14 set, so available and running.

GC902 is not in the spec. It is read like the Ashok Leyland, but an older
driver gave it its own map, with the status word at offset 68. Treat any GC902
decode as unverified until a unit has been captured.

## Map 2: SGC 120 Mk II

78 registers, offsets 1-78, in one read. There is no separate `Alarms` or
`status` message: both blocks are inside `registers`. Anything unlisted is
reserved or zero.

**Corrected 23 September 2026:** from index 49 on, the previous version of this
table was one position early. The capture puts engine speed at 56 (1511 RPM at
50.4 Hz, which is 4-pole synchronous speed), and the DEIF-based decode that was
checked on a live unit agrees on oil, temperature, battery and speed.

```
k      field                                      captured
0-2    Gen L1/L2/L3-N voltage (0.1 V)             240.1 / 239.3 / 228.6 V
3-5    Gen L1-L2/L2-L3/L3-L1 voltage (0.1 V)      414.8 / 405.6 / 405.9 V
6      Generator frequency (0.1 Hz)               50.4 Hz (7 and 8 repeat it)
22-24  Gen L1/L2/L3 current (0.1 A)               0, no load: unverified
25-28  Gen L1/L2/L3/total power (0.1 kW, signed)  0, no load: unverified
50     Lube oil pressure (0.1 Bar)                4.9 Bar
51     Coolant temp (0.1 C, signed)               40.0 C
52     Fuel level (/10)                           28.0 %
54     Charging alt. voltage (0.1 V)              14.6 V
55     Battery voltage (0.1 V)                    14.5 V
56     Engine speed (RPM)                         1511
57     Number of starts                           45
64-75  Alarm registers (offsets 65-76)            layout in the DEIF document
77     Status word (offset 78)                    0x2310
```

The line-to-line voltages at 3-5 equal the phasor sums of 0-2 to within 0.4 V,
which confirms both groups. Status word: bit 8 is running (set in the capture,
engine at 1511 RPM); bits 11-13 are the mode, 5 = Auto (the capture reads 4,
not verified).

Run time is not settled. The old table put a 32-bit value at 58-59; shifted, it
is 59-60, which the capture reads as 1 h. That is only plausible because the
spec calls this a new controller, and index 58 (34) is unexplained. Check run
time against a unit with known hours before displaying it.
