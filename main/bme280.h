/*
 * bme280.h - Bosch BME280 environmental sensor driver, written from the
 * datasheet (BST-BME280-DS002) register map. No vendor code.
 *
 * The driver never touches a bus. It is handed a bme280_bus_t - three
 * function pointers and a context - and everything it knows about I2C
 * arrives through that interface. The same bme280.c therefore compiles
 * unchanged against:
 *
 *   - the fault-injection mock bus in test/ (host, CI),
 *   - the ESP-IDF i2c_master implementation in main/main.c (target),
 *   - and, in project 09 of this portfolio, a Zephyr i2c_dt_spec shim,
 *     where this file becomes the core of an out-of-tree Zephyr module.
 *
 * That layering is the point: a driver that calls i2c_master_transmit()
 * directly can only ever be tested where an ESP32 exists. This one's error
 * model is proven on the host, one injected fault at a time.
 */
#ifndef BME280_H
#define BME280_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ bus --
 * Status codes a bus implementation reports up. Each names a distinct
 * physical failure, because "the read failed" is not an error model:
 *
 *   NACK_ADDR  nobody ACKed the address byte - wrong address, sensor
 *              absent or unpowered, or SDA/SCL swapped.
 *   NACK_DATA  the address ACKed but a later byte didn't - the device
 *              dropped off mid-transfer (brown-out, hot-unplug) or the
 *              register write was rejected.
 *   TIMEOUT    the transfer never completed - a slave stretching SCL
 *              past the master's limit, or a line stuck low.
 *   BUSY       the bus was owned by someone else (multi-master
 *              arbitration lost, or a previous transaction wedged).
 */
typedef enum {
    BME280_BUS_OK             =  0,
    BME280_BUS_ERR_NACK_ADDR  = -1,
    BME280_BUS_ERR_NACK_DATA  = -2,
    BME280_BUS_ERR_TIMEOUT    = -3,
    BME280_BUS_ERR_BUSY       = -4,
} bme280_bus_status_t;

/* The injected bus interface. `read_regs` and `write_regs` address the
 * sensor's register file (write the register pointer, then transfer);
 * `probe` is a bare address-ACK check with no data phase. All return a
 * bme280_bus_status_t. */
typedef struct {
    void *ctx;
    int (*read_regs)(void *ctx, uint8_t reg, uint8_t *buf, size_t len);
    int (*write_regs)(void *ctx, uint8_t reg, const uint8_t *buf, size_t len);
    int (*probe)(void *ctx);
} bme280_bus_t;

/* --------------------------------------------------------------- errors --
 * What the driver reports up. Bus failures keep their identity (a timeout
 * is not a NACK); the rest name the ways a bus that "works" can still be
 * lying to you.
 */
typedef enum {
    BME280_OK             = 0,
    BME280_ERR_BAD_ARG,    /* NULL device/bus/output, or a config value
                              outside its field's range */
    BME280_ERR_NACK,       /* address or data NACK, from the bus */
    BME280_ERR_TIMEOUT,    /* clock stretched past the master's patience */
    BME280_ERR_BUSY,       /* bus owned by someone else */
    BME280_ERR_BAD_ID,     /* chip id readable but wrong. Includes the trap
                              this driver exists to catch: a stuck bus reads
                              as 0xFF with a clean bus status, which a naive
                              driver accepts as a chip id. 0xFF != 0x60. */
    BME280_ERR_BAD_CALIB,  /* calibration block all-0x00 or all-0xFF: the
                              read "worked" but returned no device */
    BME280_ERR_NOT_READY,  /* forced conversion still running after the
                              bounded poll budget */
    BME280_ERR_NOT_INIT,   /* API called before a successful bme280_init */
} bme280_err_t;

const char *bme280_err_str(bme280_err_t err);

/* ------------------------------------------------------- register map --
 * Datasheet section 5.3 (memory map). Calibration lives in two banks:
 * 0x88..0xA1 (temperature + pressure + dig_H1) and 0xE1..0xE7 (the rest
 * of humidity, including the split dig_H4/dig_H5 nibbles).
 */
#define BME280_REG_CALIB00     0x88  /* 26-byte burst: dig_T*, dig_P*, dig_H1 */
#define BME280_REG_ID          0xD0
#define BME280_REG_RESET       0xE0
#define BME280_REG_CALIB26     0xE1  /* 7-byte burst: dig_H2..dig_H6 */
#define BME280_REG_CTRL_HUM    0xF2
#define BME280_REG_STATUS      0xF3
#define BME280_REG_CTRL_MEAS   0xF4
#define BME280_REG_CONFIG      0xF5
#define BME280_REG_DATA        0xF7  /* 8-byte burst: press, temp, hum */

#define BME280_CHIP_ID         0x60  /* section 5.4.1; BMP280 would be 0x58 */
#define BME280_RESET_WORD      0xB6  /* section 5.4.2 */
#define BME280_STATUS_MEASURING 0x08 /* bit 3: conversion running */

#define BME280_I2C_ADDR_SDO_LOW  0x76
#define BME280_I2C_ADDR_SDO_HIGH 0x77

/* --------------------------------------------------------------- config --
 * Field encodings from sections 5.4.3-5.4.6. The enum values ARE the
 * register field values.
 */
typedef enum {           /* osrs_t / osrs_p / osrs_h */
    BME280_OVERSAMPLE_SKIP = 0,  /* output forced to 0x80000 / 0x8000 */
    BME280_OVERSAMPLE_X1   = 1,
    BME280_OVERSAMPLE_X2   = 2,
    BME280_OVERSAMPLE_X4   = 3,
    BME280_OVERSAMPLE_X8   = 4,
    BME280_OVERSAMPLE_X16  = 5,
} bme280_oversample_t;

typedef enum {           /* mode[1:0] in ctrl_meas */
    BME280_MODE_SLEEP  = 0,
    BME280_MODE_FORCED = 1,      /* one conversion, then back to sleep */
    BME280_MODE_NORMAL = 3,      /* free-running: convert, standby, repeat */
} bme280_mode_t;

typedef enum {           /* t_sb[2:0] in config, section 3.3.4 table 27 */
    BME280_STANDBY_0_5_MS  = 0,
    BME280_STANDBY_62_5_MS = 1,
    BME280_STANDBY_125_MS  = 2,
    BME280_STANDBY_250_MS  = 3,
    BME280_STANDBY_500_MS  = 4,
    BME280_STANDBY_1000_MS = 5,
    BME280_STANDBY_10_MS   = 6,
    BME280_STANDBY_20_MS   = 7,
} bme280_standby_t;

typedef enum {           /* filter[2:0] in config, section 3.4.4 */
    BME280_FILTER_OFF = 0,
    BME280_FILTER_2   = 1,
    BME280_FILTER_4   = 2,
    BME280_FILTER_8   = 3,
    BME280_FILTER_16  = 4,
} bme280_filter_t;

typedef struct {
    bme280_oversample_t oversample_temp;
    bme280_oversample_t oversample_press;
    bme280_oversample_t oversample_hum;
    bme280_filter_t     filter;
    bme280_standby_t    standby;   /* normal mode only */
} bme280_config_t;

/* ---------------------------------------------------------- calibration --
 * Section 4.2.2, table 16. The types are the trap: dig_T1 and dig_P1 are
 * UNSIGNED, their neighbours signed, all little-endian in the register
 * file - and dig_H4/dig_H5 are 12-bit values sharing the byte at 0xE5.
 */
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4, dig_H5;     /* 12-bit signed, split across 0xE4..0xE6 */
    int8_t   dig_H6;
} bme280_calib_t;

/* --------------------------------------------------------------- device -- */
typedef struct {
    bme280_bus_t   bus;
    bme280_calib_t calib;
    int32_t        t_fine;       /* carries temperature into P and H comp */
    bool           initialised;
} bme280_t;

/* Fixed-point outputs, formats straight from the section 8.2 reference:
 *   temperature_c100   0.01 degC  (2508 = 25.08 degC)
 *   pressure_q24_8_pa  Q24.8 Pa   (24674867 = 96386.2 Pa)
 *   humidity_q22_10_rh Q22.10 %RH (47445 = 46.333 %RH)
 */
typedef struct {
    int32_t  temperature_c100;
    uint32_t pressure_q24_8_pa;
    uint32_t humidity_q22_10_rh;
} bme280_sample_t;

/* ------------------------------------------------------------------ api --
 * Every call returns bme280_err_t and reports failure honestly; none of
 * them retry, log, or sleep - policy belongs to the caller.
 */

/* Probe the address, verify the chip id (rejecting the stuck-bus 0xFF),
 * soft-reset, read and sanity-check both calibration banks. */
bme280_err_t bme280_init(bme280_t *dev, const bme280_bus_t *bus);

/* Write ctrl_hum + config + ctrl_meas. Mode SLEEP or NORMAL here; use
 * bme280_measure_forced() for forced conversions. Per section 5.4.3,
 * ctrl_hum only takes effect after a ctrl_meas write, so order matters
 * and this function owns it. */
bme280_err_t bme280_configure(bme280_t *dev, const bme280_config_t *cfg,
                              bme280_mode_t mode);

/* Trigger one forced conversion and poll status.measuring until it
 * clears, at most max_polls status reads (the caller decides how long a
 * poll budget is; wall-clock delay between polls is the caller's too -
 * the driver does not sleep). Then read and compensate. */
bme280_err_t bme280_measure_forced(bme280_t *dev, int max_polls,
                                   bme280_sample_t *out);

/* Read and compensate the current data registers (normal mode, or after
 * a forced conversion is known complete). Burst-reads all 8 data bytes
 * in one transfer, as section 4 requires for consistency. */
bme280_err_t bme280_read(bme280_t *dev, bme280_sample_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BME280_H */
