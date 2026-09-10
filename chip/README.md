# The simulated sensor: a BME280 custom chip, from the same datasheet

Wokwi has no built-in BME280 part (the nearest is the BMP180, a different
chip), so this repo ships its own: `bme280.chip.c`, a Wokwi custom chip
implementing the BME280's I2C register file from BST-BME280-DS002.

What it implements:

- The real I2C access protocol: writes as (register, value) pairs, reads
  auto-incrementing from the register pointer (section 6.2). The chip ACKs
  only 0x76, so the firmware's deliberate probe of 0x77 gets a genuine
  address NACK on the wire.
- Chip id 0x60 at 0xD0; soft reset via 0xB6 at 0xE0, restoring the
  power-on register image.
- Forced mode with a realistic ~8 ms conversion during which
  `status.measuring` reads 1 — so the driver's poll loop is exercised for
  real — and normal mode free-running at the config register's standby
  period (table 27).
- The calibration banks hold the datasheet worked-example image: the same
  values `test/mock_bus.c` uses and the host suite pins against.

The diagram attrs (`temperature` in °C, `pressure` in Pa, `humidity` in
%RH) are physical units. At each conversion the chip *inverts* the section
8.2 fixed-point compensation numerically — a binary search over the ADC
space, valid because each output is monotonic in its raw input — and
serves the raw values that compensate back to the attrs.

That makes the simulation a two-sided test: the chip and the driver each
implement section 8.2 independently, and the reading the firmware prints
is only right if both readings of the datasheet agree.
`chip/test_chip.c` asserts exactly that on the host (CI runs it on every
commit): the driver's I2C traffic replayed against the chip's callbacks,
five operating points, round-tripped within fixed-point quantisation.

## Building

```bash
# wasm for the simulator (what CI does):
docker run --rm -v "$PWD":/src -w /src wokwi/builder-clang-wasm make -C chip

# host-side chip-vs-driver agreement test (plain cc):
make -C chip host-test
```
