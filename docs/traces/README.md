# The captured wire: annotated I2C traces

`i2c-trace.vcd` is a real capture from Wokwi's logic analyzer (D0 = SDA,
D1 = SCL), taken by CI during the simulate job (run 34472027073, 10 Sept
2026) — the firmware whose serial output CI asserted and the wire traffic
here are the same execution. Open it in PulseView (add an I2C decoder,
SDA→D0, SCL→D1) or GTKWave, or read `i2c-trace-decoded.txt`, produced by
`decode_i2c.py` in this directory.

Notation below: `S` start, `Sr` repeated start, `P` stop; every byte is
followed by the ACK/NACK sampled on the 9th clock.

## 1. The address NACK — nothing lives at 0x77

```
t=0.524005s  S addr 0x77 W NACK P
```

The master sends START and clocks out `0xEE` (0x77<<1, write). On the 9th
clock it releases SDA and listens; with nobody there to pull the line low,
SDA stays high — that lone high bit is the entire NACK. In the VCD you can
see SDA released and never yanked down. The firmware provokes this on
purpose at boot (probing the SDO-high address) so every capture contains a
failure signature next to the healthy traffic. Compare it with the next
line — same clock, same edges, one bit different, completely different
meaning.

## 2. Probe, identity, reset

```
t=0.532670s  S addr 0x76 W ACK P                                # probe: address-only
t=0.532940s  S addr 0x76 W ACK | 0xD0 ACK                        # register pointer = ID
t=0.533205s  Sr addr 0x76 R ACK | 0x60 ACK | 0x00 NACK P         # chip id 0x60 = BME280
t=0.533717s  S addr 0x76 W ACK | 0xE0 ACK | 0xB6 ACK P           # soft reset
```

The `Sr` is a repeated start: write the register pointer, then flip to
read without releasing the bus, so no other master can sneak in between.
The `0x60` is the byte the whole error model pivots on — a stuck-high SDA
would deliver `0xFF` right here, with every ACK bit reading "fine".

## 3. Calibration, burst-read

```
t=0.534998s  S addr 0x76 W ACK | 0x88 ACK
t=0.535262s  Sr addr 0x76 R ACK | 0x70 ACK | 0x6B ACK | ... 26 bytes ... | 0x4B NACK P
t=0.537938s  S addr 0x76 W ACK | 0xE1 ACK
t=0.538203s  Sr addr 0x76 R ACK | 0x6A ACK | 0x01 ACK | 0x00 ACK | 0x13 ACK | 0x2B ACK | 0x03 ACK | 0x1E NACK P
```

First bytes `0x70 0x6B` little-endian = 27504 = dig_T1, and the last byte
of the first bank is `0x4B` = 75 = dig_H1 — the worked-example image the
host tests pin against, now visible on the wire. The second burst carries
the interleaved dig_H4/H5: `0x13 0x2B 0x03` decodes to H4=315, H5=50. The
final byte of each read is NACKed by the *master* — that's how a
controller says "I'm done", and it's why a NACK is not inherently an
error; position decides.

## 4. Forced conversion: trigger, stretch of polls, data

```
t=0.549436s  S addr 0x76 W ACK | 0xF4 ACK | 0x25 ACK P           # mode=forced
t=0.549923s  S addr 0x76 W ACK | 0xF3 ACK
t=0.550188s  Sr addr 0x76 R ACK | 0x08 ACK | 0x25 NACK P         # measuring=1
             ... nine more identical status polls over ~8 ms ...
t=0.558126s  Sr addr 0x76 R ACK | 0x00 ACK | 0x24 NACK P         # measuring=0, back in sleep
t≈0.559s     S addr 0x76 W ACK | 0xF7 ACK
             Sr addr 0x76 R ACK | 8 data bytes, last NACKed | P
```

The driver's bounded poll loop, on the wire: ten status reads while the
conversion runs, then the 8-byte data burst (`0x7E 0x78 0x00` in the
middle = adc_T 0x7E780 = 518016, which compensates to the 24.50 °C CI
asserted). Note `0xF4` reading back `0x24` after completion: forced mode
self-clears to sleep, exactly as section 3.3.3 says.

## 5. A quirk only the wire shows

Every 1-byte read in the trace clocks *two* bytes — the requested one,
ACKed, then one more, NACKed and discarded (see the `0x60 ACK | 0x00
NACK` id read: the 0x00 is register 0xD1, the id's auto-increment
neighbour). The ESP32's I2C controller does this on short reads; the
driver never sees it, the sensor doesn't mind it, and no amount of
firmware debugging would reveal it. That asymmetry — burst reads NACK
exactly on the last requested byte, single reads over-fetch — is the kind
of detail a wire capture exists to catch.

## What this capture cannot say

Digital edges only: protocol order and ACK bits, not rise times against
bus capacitance, not pull-up strength, not noise margins. See "Honest
limits" in the main README.
