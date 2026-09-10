# Why the driver takes an injected bus, and what that buys

`bme280.c` never calls `i2c_master_transmit()`. It is handed this at init:

```c
typedef struct {
    void *ctx;
    int (*read_regs)(void *ctx, uint8_t reg, uint8_t *buf, size_t len);
    int (*write_regs)(void *ctx, uint8_t reg, const uint8_t *buf, size_t len);
    int (*probe)(void *ctx);
} bme280_bus_t;
```

Three operations, one context pointer, and a status vocabulary
(`bme280_bus_status_t`) that names physical failures — address NACK, data
NACK, timeout, busy — rather than whatever the transport's error type is.
That's the entire contract between the driver and the world.

## What this costs

One indirect call per transaction (~nanoseconds against a 100 kHz bus where
a single register read takes ~200 µs), and one small struct per device.
That is the whole bill.

## What this buys

**Testability that is otherwise impossible.** The mock bus in
`test/mock_bus.c` implements the same three ops over a 256-byte array, with
one scripted fault that can be aimed at any single transaction. That is how
CI executes *every* error branch in `bme280.c` on every commit — the
coverage gate refuses anything under 100% branch. A driver that calls
`i2c_master_*` directly can only be tested where an ESP32 and a sensor
exist, which in practice means its error paths are tested by field
failures.

**Ports that touch one file.** The ESP-IDF binding in `main/main.c` is
~50 lines: three functions over `i2c_master_transmit_receive` /
`i2c_master_transmit` / `i2c_master_probe` and an `esp_err_t` translation.
Nothing in `bme280.c` changed to run on target, and nothing changes to run
it over an ESP-IDF SPI binding either — the BME280 speaks SPI too, and the
ops signature doesn't care.

**The Zephyr module, already paid for.** Project 09 of this portfolio
ports this driver as a proper out-of-tree Zephyr module — devicetree
binding, `DT_INST` macros, sensor API glue. The port is a third
implementation of the same three ops over `i2c_write_read_dt()`, plus
Zephyr packaging. The compensation arithmetic, the calibration parsing,
and the error model — the parts that take datasheet-reading time and are
easy to get subtly wrong — move without edits. That is what "portable
driver" means when it's a property of the code rather than a hope.

## Where the line sits

Policy stays above the interface, mechanism below. The driver never
retries, never sleeps, never logs: `bme280_measure_forced()` takes a poll
*budget* (a count of status reads), not a timeout in milliseconds, because
the driver owns no clock. The ESP-IDF binding decides what 100 ms of
patience means; the mock decides that time doesn't exist. Both are right.

The same discipline is why there is no `bme280_init_i2c(sda, scl)`
convenience wrapper: the moment the driver knows what a pin is, it stops
compiling on the host, and the fault-injection suite — the actual product
of this repo — dies with it.
