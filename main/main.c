/*
 * main.c - target-side binding for the BME280 driver.
 *
 * Everything sensor-shaped lives in bme280.c, which does not know this
 * file exists. What lives HERE is exactly what a port owns: an
 * implementation of the three bus ops over ESP-IDF v5.3's i2c_master
 * driver, the error-code translation, and policy (retry, pacing,
 * logging) that the driver deliberately refuses to have.
 *
 * Boot sequence doubles as the wire-trace generator for the README: it
 * first probes 0x77 - where nothing lives in the simulation - so the
 * logic analyzer capture (wokwi.toml: i2c-trace.vcd) contains a real
 * NACK next to the healthy transactions that follow at 0x76.
 */
#include <inttypes.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#include "bme280.h"

static const char *TAG = "bme280-demo";

#define PIN_SDA        21
#define PIN_SCL        22
#define I2C_TIMEOUT_MS 100

/* ------------------------------------------------- the bus ops binding -- */

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
} esp_i2c_ctx_t;

/* v5.3 i2c_master reports a NACK as ESP_ERR_INVALID_STATE and a
 * stretched-or-stuck bus as ESP_ERR_TIMEOUT; anything else gets the
 * pessimistic mapping. The driver never sees an esp_err_t. */
static int map_esp_err(esp_err_t err, int nack_kind)
{
    if (err == ESP_OK)
        return BME280_BUS_OK;
    if (err == ESP_ERR_TIMEOUT)
        return BME280_BUS_ERR_TIMEOUT;
    if (err == ESP_ERR_INVALID_STATE)
        return nack_kind;
    return BME280_BUS_ERR_BUSY;
}

static int esp_read_regs(void *ctx, uint8_t reg, uint8_t *buf, size_t len)
{
    esp_i2c_ctx_t *c = ctx;
    esp_err_t err = i2c_master_transmit_receive(c->dev, &reg, 1, buf, len,
                                                I2C_TIMEOUT_MS);
    return map_esp_err(err, BME280_BUS_ERR_NACK_DATA);
}

static int esp_write_regs(void *ctx, uint8_t reg, const uint8_t *buf,
                          size_t len)
{
    esp_i2c_ctx_t *c = ctx;
    uint8_t frame[1 + 4];
    if (len > sizeof frame - 1)
        return BME280_BUS_ERR_NACK_DATA; /* driver never writes this much */
    frame[0] = reg;
    for (size_t i = 0; i < len; i++)
        frame[1 + i] = buf[i];
    esp_err_t err = i2c_master_transmit(c->dev, frame, 1 + len,
                                        I2C_TIMEOUT_MS);
    return map_esp_err(err, BME280_BUS_ERR_NACK_DATA);
}

static int esp_probe(void *ctx)
{
    esp_i2c_ctx_t *c = ctx;
    esp_err_t err = i2c_master_probe(c->bus, BME280_I2C_ADDR_SDO_LOW,
                                     I2C_TIMEOUT_MS);
    return map_esp_err(err, BME280_BUS_ERR_NACK_ADDR);
}

/* ------------------------------------------------------------- the app -- */

static void report(const bme280_sample_t *s)
{
    /* Fixed-point formats from the driver: 0.01 degC, Q24.8 Pa, Q22.10
     * %RH. The plausibility verdict is printed here and asserted by the
     * Wokwi CI scenario: 5 < T < 45 degC in the simulated room. */
    int32_t t_c100 = s->temperature_c100;
    uint32_t p_pa_x10 = (s->pressure_q24_8_pa * 10ull) >> 8;
    uint32_t h_rh_x10 = (s->humidity_q22_10_rh * 10ull) >> 10;
    bool plausible = t_c100 > 500 && t_c100 < 4500;
    printf("bme280: T=%" PRId32 ".%02d C  P=%" PRIu32 ".%" PRIu32 " Pa  "
           "H=%" PRIu32 ".%" PRIu32 " %%RH  -> %s\n",
           t_c100 / 100, (int)(t_c100 % 100),
           p_pa_x10 / 10, p_pa_x10 % 10,
           h_rh_x10 / 10, h_rh_x10 % 10,
           plausible ? "plausible" : "IMPLAUSIBLE");
}

void app_main(void)
{
    ESP_LOGI(TAG, "up: BME280 driver from the datasheet, injected bus, "
                  "no vendor code");

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = -1,
        .sda_io_num = PIN_SDA,
        .scl_io_num = PIN_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_i2c_ctx_t ctx;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &ctx.bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_I2C_ADDR_SDO_LOW,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(ctx.bus, &dev_cfg, &ctx.dev));

    /* Deliberate failure first: probe 0x77, where nothing answers, so
     * the captured VCD contains an address NACK to annotate. */
    esp_err_t nack = i2c_master_probe(ctx.bus, BME280_I2C_ADDR_SDO_HIGH,
                                      I2C_TIMEOUT_MS);
    ESP_LOGI(TAG, "probe 0x77 (SDO high): %s - that NACK is in the trace",
             nack == ESP_OK ? "unexpectedly ACKed?!" : "no ACK, as expected");

    const bme280_bus_t bus = {
        .ctx = &ctx,
        .read_regs = esp_read_regs,
        .write_regs = esp_write_regs,
        .probe = esp_probe,
    };

    bme280_t dev;
    bme280_err_t err = bme280_init(&dev, &bus);
    if (err != BME280_OK) {
        ESP_LOGE(TAG, "init failed: %s", bme280_err_str(err));
        return; /* app_main returning parks the main task cleanly */
    }
    ESP_LOGI(TAG, "chip id 0x60 verified, calibration loaded "
                  "(dig_T1=%u dig_H4=%d)", dev.calib.dig_T1, dev.calib.dig_H4);

    bme280_config_t cfg = {
        .oversample_temp = BME280_OVERSAMPLE_X1,
        .oversample_press = BME280_OVERSAMPLE_X1,
        .oversample_hum = BME280_OVERSAMPLE_X1,
        .filter = BME280_FILTER_OFF,
        .standby = BME280_STANDBY_500_MS,
    };
    err = bme280_configure(&dev, &cfg, BME280_MODE_SLEEP);
    if (err != BME280_OK) {
        ESP_LOGE(TAG, "configure: %s", bme280_err_str(err));
        return;
    }

    /* Three forced conversions - trigger, poll, read - then switch to
     * normal mode and let the sensor free-run at 500 ms standby. Both
     * modes on the wire, both in the trace. */
    ESP_LOGI(TAG, "forced mode: trigger, poll status.measuring, read");
    for (int i = 0; i < 3; i++) {
        bme280_sample_t s;
        err = bme280_measure_forced(&dev, 200, &s);
        if (err != BME280_OK) {
            ESP_LOGE(TAG, "forced measure: %s", bme280_err_str(err));
        } else {
            report(&s);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "normal mode: free-running, standby 500 ms, filter off");
    err = bme280_configure(&dev, &cfg, BME280_MODE_NORMAL);
    if (err != BME280_OK)
        ESP_LOGE(TAG, "configure normal: %s", bme280_err_str(err));

    int64_t t0 = xTaskGetTickCount();
    int last_heartbeat_s = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        bme280_sample_t s;
        err = bme280_read(&dev, &s);
        if (err != BME280_OK) {
            ESP_LOGE(TAG, "read: %s", bme280_err_str(err));
            continue;
        }
        report(&s);
        int up_s = (int)((xTaskGetTickCount() - t0) / configTICK_RATE_HZ);
        if (up_s / 5 > last_heartbeat_s / 5) {
            /* The CI scenario waits for t=10s: outliving the 5 s task
             * watchdog is part of the pass condition (01's lesson). */
            ESP_LOGI(TAG, "alive t=%ds", (up_s / 5) * 5);
            last_heartbeat_s = up_s;
        }
    }
}
