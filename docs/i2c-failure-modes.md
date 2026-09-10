# The I2C failure gallery: what each fault looks like, on the wire and in code

Every failure mode below is (a) modeled by the mock bus, (b) walked along
every transaction the driver makes by the stage-3 fault sweep, and (c)
mapped to a distinct `bme280_err_t` — because "the read failed" tells a
field technician nothing, while "address NACK" says *check the wiring and
the address strap* and "timeout" says *something is holding a line*.

Captured traces from Wokwi's logic analyzer (D0 = SDA, D1 = SCL, VCD
format, open in PulseView with the I2C decoder) live in `docs/traces/` and
are annotated in the README.

## Address NACK — `BME280_ERR_NACK`

The master drives the start condition and clocks out the 7-bit address +
R/W bit. On the 9th clock it releases SDA and listens: an ACK is the
*slave* pulling SDA low. Nothing there = nobody pulls = SDA stays high
through the 9th clock. That high bit is the entire signature.

Causes: wrong address (the BME280 straps to 0x76 or 0x77 via SDO — boards
differ), sensor unpowered, SDA/SCL swapped, wrong bus. The firmware
deliberately probes 0x77 at boot so the healthy capture contains one of
these to look at.

## Data NACK (mid-transfer) — `BME280_ERR_NACK`

Same signature, later in the frame: the address byte ACKed, then some
subsequent byte didn't. The device dropped off mid-conversation —
brown-out during the transfer, hot-unplug, or (on writes) a register the
device refuses. Distinct from address NACK in *when* it happens, which is
why the mock injects them separately and the driver's every read and write
call site is swept.

## Clock stretching → timeout — `BME280_ERR_TIMEOUT`

A slave that needs time may hold SCL low after the ACK; the master must
wait until the line rises. Legitimate stretching is invisible in firmware
(the controller rides it out). The failure mode is a stretch that outlives
the master's patience — or an SCL/SDA stuck low for electrical reasons,
which is indistinguishable from an infinite stretch. On the wire: a clock
line that goes low and stays there while the master's timeout counts down.

Two distinct budgets exist in this driver and they should not be confused:
the *transport* timeout (the ESP-IDF binding gives each transaction
100 ms) and the *conversion* poll budget (`bme280_measure_forced()` reads
`status.measuring` at most N times). The suite exercises both: a
conversion busy for 2 polls must succeed, busy past the budget must return
`BME280_ERR_NOT_READY`, and a transport timeout at any transaction must
surface as `BME280_ERR_TIMEOUT`.

## Bus busy / arbitration lost — `BME280_ERR_BUSY`

A multi-master bus, or a previous transaction that wedged (a device left
mid-read holds SDA — the classic fix is clocking out 9 dummy SCL cycles).
The driver reports it and does nothing else: recovery pulses are bus-owner
policy, and the driver doesn't own the bus.

## The stuck bus that "works" — `BME280_ERR_BAD_ID` / `BME280_ERR_BAD_CALIB`

The dangerous one, and the reason chip-id verification is not a
formality. A bus that reads SDA stuck high yields 0xFF for every byte —
and depending on the controller and HAL, the transfer can complete with a
*clean status*. 0xFF is a perfectly plausible-looking register value. A
naive driver (stage 0 of the test suite implements one) reads the id
register, ignores the value, reports the sensor present, and then ships
temperature readings compensated from 0xFF calibration garbage.

The defence is semantic, not transport-level: the id must *equal* 0x60
(0xFF fails; so does 0x58 — that's a BMP280, same bus behaviour, no
humidity), and a calibration bank that is all one value is rejected as
`BAD_CALIB` even though every transfer "succeeded". The stage-0 control
proves the naive driver falls for it before the real driver is allowed to
demonstrate that it doesn't.

## Corrupt calibration → divide-by-zero — `BME280_ERR_BAD_CALIB`

The section 8.2 pressure arithmetic divides by a term proportional to
`dig_P1`. The datasheet's own reference inserts `if (var1 == 0) return 0`
— quietly yielding 0 Pa. This driver keeps the guard but refuses to
present the result as a measurement: a zero there means the calibration
image is corrupt, and the caller hears that, not "the pressure is zero
Pascals today".
