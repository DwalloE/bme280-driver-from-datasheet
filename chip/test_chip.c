/*
 * test_chip.c - host-side check of the custom chip's logic before it ever
 * runs in the simulator: stub implementations of the Wokwi APIs, then the
 * driver's actual I2C traffic replayed against the chip's callbacks.
 *
 * The strongest assertion available: read the chip the way the driver
 * does, run the raw values through the REAL driver's compensation
 * (main/bme280.c, included via its public API against a shim bus that
 * fronts this chip), and require the result to land back on the attr
 * values the chip was configured with. Driver and chip each implement
 * section 8.2 independently; this is where they have to meet.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- stub Wokwi API ----------------------------------------------------- */
#include "wokwi-api.h"

static float g_attrs[8];
static int g_attr_count;
static struct { void *user_data; void (*cb)(void *); bool armed; } g_timers[8];
static int g_timer_count;
static i2c_config_t g_i2c;

pin_t pin_init(const char *name, uint32_t mode)
{
    (void)name; (void)mode; return 1;
}
uint32_t attr_init_float(const char *name, float def)
{
    (void)name;
    g_attrs[g_attr_count] = def;
    return (uint32_t)g_attr_count++;
}
float attr_read_float(uint32_t id) { return g_attrs[id]; }
timer_t timer_init(const timer_config_t *cfg)
{
    g_timers[g_timer_count].user_data = cfg->user_data;
    g_timers[g_timer_count].cb = cfg->callback;
    return (timer_t)g_timer_count++;
}
void timer_start(timer_t t, uint32_t us, bool repeat)
{
    (void)us; (void)repeat; g_timers[t].armed = true;
}
void timer_stop(timer_t t) { g_timers[t].armed = false; }
i2c_dev_t i2c_init(const i2c_config_t *cfg) { g_i2c = *cfg; return 0; }

/* fire any armed timer, as the sim scheduler would */
static void run_timers(void)
{
    for (int i = 0; i < g_timer_count; i++) {
        if (g_timers[i].armed) {
            g_timers[i].armed = false;
            g_timers[i].cb(g_timers[i].user_data);
        }
    }
}

#include "bme280.chip.c"

/* ---- a bme280_bus_t that talks to the chip's callbacks ------------------ */
#include "../main/bme280.h"

static int chipbus_read(void *ctx, uint8_t reg, uint8_t *buf, size_t len)
{
    (void)ctx;
    g_i2c.connect(g_i2c.user_data, ADDR, false);
    g_i2c.write(g_i2c.user_data, reg);
    g_i2c.connect(g_i2c.user_data, ADDR, true);
    for (size_t i = 0; i < len; i++)
        buf[i] = g_i2c.read(g_i2c.user_data);
    return BME280_BUS_OK;
}

static int chipbus_write(void *ctx, uint8_t reg, const uint8_t *buf,
                         size_t len)
{
    (void)ctx;
    g_i2c.connect(g_i2c.user_data, ADDR, false);
    for (size_t i = 0; i < len; i++) {
        g_i2c.write(g_i2c.user_data, (uint8_t)(reg + i));
        g_i2c.write(g_i2c.user_data, buf[i]);
    }
    /* a write can complete a conversion trigger */
    run_timers();
    return BME280_BUS_OK;
}

static int chipbus_probe(void *ctx) { (void)ctx; return BME280_BUS_OK; }

static int failures = 0;
#define CHECK(cond) do {                                                   \
        if (!(cond)) {                                                     \
            failures++;                                                    \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
        }                                                                  \
    } while (0)

int main(void)
{
    chip_init();

    const bme280_bus_t bus = {
        .ctx = NULL,
        .read_regs = chipbus_read,
        .write_regs = chipbus_write,
        .probe = chipbus_probe,
    };

    bme280_t dev;
    CHECK(bme280_init(&dev, &bus) == BME280_OK);
    CHECK(dev.calib.dig_T1 == DIG_T1);
    CHECK(dev.calib.dig_H4 == DIG_H4);
    CHECK(dev.calib.dig_H5 == DIG_H5);

    bme280_config_t cfg = {
        .oversample_temp = BME280_OVERSAMPLE_X1,
        .oversample_press = BME280_OVERSAMPLE_X1,
        .oversample_hum = BME280_OVERSAMPLE_X1,
        .filter = BME280_FILTER_OFF,
        .standby = BME280_STANDBY_500_MS,
    };
    CHECK(bme280_configure(&dev, &cfg, BME280_MODE_SLEEP) == BME280_OK);

    /* Sweep operating points: for each attr triple, a forced measurement
     * through the real driver must land back on the attrs within the
     * fixed-point quantisation (0.01 degC, ~0.004 Pa, ~0.001 %RH - allow
     * generous slack for the inversion's rounding). */
    static const float points[][3] = {
        { 24.5f, 101325.0f, 42.0f },   /* the diagram defaults */
        { 5.5f, 95000.0f, 80.0f },
        { 44.5f, 105000.0f, 10.0f },
        { 0.0f, 87000.0f, 55.0f },
        { -10.0f, 101325.0f, 30.0f },
    };
    for (size_t i = 0; i < sizeof points / sizeof points[0]; i++) {
        g_attrs[0] = points[i][0];
        g_attrs[1] = points[i][1];
        g_attrs[2] = points[i][2];
        bme280_sample_t s;
        bme280_err_t err = bme280_measure_forced(&dev, 8, &s);
        CHECK(err == BME280_OK);
        float t = s.temperature_c100 / 100.0f;
        float p = s.pressure_q24_8_pa / 256.0f;
        float h = s.humidity_q22_10_rh / 1024.0f;
        printf("attr (%.2f C, %.0f Pa, %.0f %%) -> driver "
               "(%.2f C, %.1f Pa, %.2f %%)\n",
               points[i][0], points[i][1], points[i][2], t, p, h);
        CHECK(fabsf(t - points[i][0]) <= 0.02f);
        CHECK(fabsf(p - points[i][1]) <= 1.0f);
        CHECK(fabsf(h - points[i][2]) <= 0.05f);
    }

    /* Normal mode: configure it, fire the periodic timer, read. */
    g_attrs[0] = 24.5f; g_attrs[1] = 101325.0f; g_attrs[2] = 42.0f;
    CHECK(bme280_configure(&dev, &cfg, BME280_MODE_NORMAL) == BME280_OK);
    run_timers();
    bme280_sample_t s;
    CHECK(bme280_read(&dev, &s) == BME280_OK);
    CHECK(abs(s.temperature_c100 - 2450) <= 2);

    /* Reset drops it back to the skipped-data pattern. */
    uint8_t b6 = 0xB6, d0 = 0;
    chipbus_write(NULL, REG_RESET, &b6, 1);
    chipbus_read(NULL, REG_DATA + 3, &d0, 1);
    CHECK(d0 == 0x80);

    if (failures) {
        printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    printf("CHIP HOST TEST PASS: driver and chip agree on section 8.2\n");
    return 0;
}

/* the real driver, compiled into this test */
#include "../main/bme280.c"
