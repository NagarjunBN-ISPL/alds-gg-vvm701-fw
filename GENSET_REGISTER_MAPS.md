# Genset register maps

Companion to `CLOUD_INTEGRATION_GUIDE_SR.md` section 5.5, which defines the
`genset` payload itself. This file only covers how to decode the register bytes
inside it. You need it only on sites where `gensetType` is 1.

`register[k] = data[2k]<<8 | data[2k+1]`, 0-based within the message. Every read
starts at Modbus offset 1, so `register[k]` is protocol offset `k+1`. For signed
fields, subtract 65536 if the value is above 32767. For 32-bit fields,
`(register[lo]<<16) | register[hi]`, where the lower index is the high word. That
last rule is not yet confirmed against live hardware.

Which map applies depends on `gensetModel`. The model-to-controller table, and
which models send a `status` message at all, is in section 5.5 of the integration
guide — that table is the single source of truth for both.

## Map 1: Mahindra GC1211/GC1111, Ashok Leyland GC1201/02/03, Sedemac GC902

36 registers, offsets 1-36:

```
0  Oil pressure (0.01 Bar)        9-11   Gen L1/L2/L3-N voltage (0.1 V)
1  Engine temp (1 C)              12-14  Gen L1-L2/L2-L3/L3-L1 voltage (0.1 V)
2  Fuel level (1 %)               15-17  Load L1/L2/L3 current (0.01 A)
3  Charging alt. voltage (0.1 V)  18-21  Gen L1/L2/L3/total active power (0.1 kW)
4  Battery voltage (0.1 V)        22-25  Gen L1/L2/L3/total apparent power (0.1 kVA)
5  Number of starts               26-29  Gen L1/L2/L3/total reactive power (0.1 kVAr)
6  Engine run time (hours)        30-33  Gen L1/L2/L3/avg power factor (0.01)
7  Engine speed (RPM)             34-35  Cumulative energy, 32-bit (0.1 kWh)
8  Genset frequency (0.1 Hz)
```

For `status`, Mahindra sends 2 registers (`[0]` is the DG status word at offset
67, `[1]` the command status at offset 68) while Ashok Leyland and Sedemac send
only `[0]`. DG status bits: 15 available/loadOnDG, 14 running, 13
stoppedNormally, 12 stoppedWithFault, 11 mode (1 = Auto). Command status, Mahindra
only: 1 Started, 2 Stopped, 3 AlreadyON, 4 AlreadyOFF, 5 Manual, 6 NotStarted,
anything else Unknown.

Worked example: `registers` of `01,3f,00,3e,00,22,...` gives oil 0x013F = 319 =
3.19 Bar, temp 0x003E = 62 C, fuel 0x0022 = 34 %. A `status` of `c0,00,00,00` is
0xC000, bits 15 and 14 set, so available and running.

## Map 2: SGC 120 Mk II

78 registers, offsets 1-78, sparse. Anything unlisted is reserved or zero, and
this controller sends no `status` message.

```
0-2    Gen L1/L2/L3-N voltage (0.1 V)             50  Coolant temp (0.1 C, signed)
6      Generator frequency (0.1 Hz)               51  Fuel level (/10, inferred)
22-24  Gen L1/L2/L3 current (0.1 A)               53  Charging alt. voltage (0.1 V)
25-28  Gen L1/L2/L3/total power (0.1 kW, signed)  54  Battery voltage (0.1 V)
49     Lube oil pressure (0.1 Bar, unverified)    55  Engine speed (RPM)
                                                  56  Number of starts
                                                  58-59  Engine run time, 32-bit (hours)
```

These positions come from DEIF documentation and are only partly confirmed, so
sanity-check the first real reading of each field.
