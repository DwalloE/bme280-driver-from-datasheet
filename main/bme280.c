/*
 * bme280.c - BME280 driver core. Compiles unchanged for host (mock bus)
 * and target (ESP-IDF i2c_master); see bme280.h for the layering note.
 *
 * The compensation arithmetic is the datasheet's own fixed-point
 * reference (BST-BME280-DS002 section 8.2), transcribed - not adapted -
 * because every shift in it encodes a scaling decision Bosch made
 * against the cell characteristics. int32 for temperature and humidity,
 * int64 for pressure.
 *
 * CI gates this file at 100% branch coverage: the error model is the
 * product, and an untaken error branch is exactly the kind of code that
 * works until the day the bus doesn't.
 */
#include "bme280.h"

/* ------------------------------------------------------------- plumbing -- */

/* Bus failures keep their identity on the way up. */
static bme280_err_t map_bus_err(int rc)
{
    switch (rc) {
    case BME280_BUS_ERR_NACK_ADDR: return BME280_ERR_NACK;
    case BME280_BUS_ERR_NACK_DATA: return BME280_ERR_NACK;
    case BME280_BUS_ERR_TIMEOUT:   return BME280_ERR_TIMEOUT;
    case BME280_BUS_ERR_BUSY:      return BME280_ERR_BUSY;
    default:                       return BME280_ERR_NACK;
    }
}

const char *bme280_err_str(bme280_err_t err)
{
    switch (err) {
    case BME280_OK:             return "ok";
    case BME280_ERR_BAD_ARG:    return "bad argument";
    case BME280_ERR_NACK:       return "bus NACK";
    case BME280_ERR_TIMEOUT:    return "bus timeout (clock stretch?)";
    case BME280_ERR_BUSY:       return "bus busy";
    case BME280_ERR_BAD_ID:     return "wrong chip id (stuck bus reads 0xFF)";
    case BME280_ERR_BAD_CALIB:  return "calibration block implausible";
    case BME280_ERR_NOT_READY:  return "conversion did not finish";
    case BME280_ERR_NOT_INIT:   return "driver not initialised";
    default:                    return "unknown error";
    }
}

static bme280_err_t read_regs(bme280_t *dev, uint8_t reg, uint8_t *buf,
                              size_t len)
{
    int rc = dev->bus.read_regs(dev->bus.ctx, reg, buf, len);
    if (rc != BME280_BUS_OK)
        return map_bus_err(rc);
    return BME280_OK;
}

static bme280_err_t write_reg(bme280_t *dev, uint8_t reg, uint8_t val)
{
    int rc = dev->bus.write_regs(dev->bus.ctx, reg, &val, 1);
    if (rc != BME280_BUS_OK)
        return map_bus_err(rc);
    return BME280_OK;
}

/* -------------------------------------------------- calibration parsing --
 * Section 4.2.2 table 16. All 16-bit words are little-endian in the
 * register file. The classic datasheet-reading failures live here:
 *
 *   - dig_T1 and dig_P1 are UNSIGNED; T2/T3 and P2..P9 are SIGNED. Parse
 *     dig_T1 as int16_t and every temperature above ~32 degC of the
 *     word's midpoint corrupts t_fine - and t_fine feeds P and H too.
 *   - dig_H4/dig_H5 are 12-bit signed values interleaved across three
 *     bytes: 0xE4 is H4[11:4], 0xE5's low nibble H4[3:0], 0xE5's high
 *     nibble H5[3:0], 0xE6 is H5[11:4]. Sign lives in the byte that
 *     holds bits [11:4], so that byte must be widened as int8_t BEFORE
 *     the shift.
 */
static uint16_t le_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[1] << 8 | p[0]);
}

static int16_t le_s16(const uint8_t *p)
{
    return (int16_t)le_u16(p);
}

static void parse_calib(bme280_calib_t *c, const uint8_t *bank0 /* 26B @0x88 */,
                        const uint8_t *bank1 /* 7B @0xE1 */)
{
    c->dig_T1 = le_u16(bank0 + 0);
    c->dig_T2 = le_s16(bank0 + 2);
    c->dig_T3 = le_s16(bank0 + 4);
    c->dig_P1 = le_u16(bank0 + 6);
    c->dig_P2 = le_s16(bank0 + 8);
    c->dig_P3 = le_s16(bank0 + 10);
    c->dig_P4 = le_s16(bank0 + 12);
    c->dig_P5 = le_s16(bank0 + 14);
    c->dig_P6 = le_s16(bank0 + 16);
    c->dig_P7 = le_s16(bank0 + 18);
    c->dig_P8 = le_s16(bank0 + 20);
    c->dig_P9 = le_s16(bank0 + 22);
    /* bank0[24] (0xA0) is a hole in the map; dig_H1 sits at 0xA1. */
    c->dig_H1 = bank0[25];

    c->dig_H2 = le_s16(bank1 + 0);                     /* 0xE1/0xE2 */
    c->dig_H3 = bank1[2];                              /* 0xE3      */
    c->dig_H4 = (int16_t)(((int16_t)(int8_t)bank1[3] << 4)   /* 0xE4 */
                          | (bank1[4] & 0x0F));               /* 0xE5[3:0] */
    c->dig_H5 = (int16_t)(((int16_t)(int8_t)bank1[5] << 4)   /* 0xE6 */
                          | (bank1[4] >> 4));                 /* 0xE5[7:4] */
    c->dig_H6 = (int8_t)bank1[6];                      /* 0xE7      */
}

/* A bus that ACKs but returns a constant is indistinguishable from a
 * device at the transport level - but no real BME280 ships a calibration
 * bank that is all one value. */
static bool calib_block_plausible(const uint8_t *bank0)
{
    for (size_t i = 1; i < 26; i++) {
        if (bank0[i] != bank0[0])
            return true;
    }
    return false;
}

/* ------------------------------------------------- compensation, s 8.2 --
 * Variable names (var1, var2, v_x1_u32r) and every constant are the
 * datasheet's, so the code diffs cleanly against the reference.
 */

/* Section 8.2: "Returns temperature in DegC, resolution is 0.01 DegC.
 * Output value of '5123' equals 51.23 DegC." Sets dev->t_fine. */
static int32_t compensate_temperature(bme280_t *dev, int32_t adc_T)
{
    const bme280_calib_t *c = &dev->calib;
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)c->dig_T1 << 1))) *
            ((int32_t)c->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)c->dig_T1)) *
              ((adc_T >> 4) - ((int32_t)c->dig_T1))) >> 12) *
            ((int32_t)c->dig_T3)) >> 14;
    dev->t_fine = var1 + var2;
    T = (dev->t_fine * 5 + 128) >> 8;
    return T;
}

/* Section 8.2: "Returns pressure in Pa as unsigned 32 bit integer in
 * Q24.8 format (24 integer bits and 8 fractional bits). Output value of
 * '24674867' represents 24674867/256 = 96386.2 Pa." 64-bit variant. */
static uint32_t compensate_pressure(const bme280_t *dev, int32_t adc_P)
{
    const bme280_calib_t *c = &dev->calib;
    int64_t var1, var2, p;

    var1 = ((int64_t)dev->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)c->dig_P6;
    var2 = var2 + ((var1 * (int64_t)c->dig_P5) << 17);
    var2 = var2 + (((int64_t)c->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)c->dig_P3) >> 8) +
           ((var1 * (int64_t)c->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)c->dig_P1) >> 33;
    if (var1 == 0) {
        return 0; /* datasheet's own guard: avoid division by zero */
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)c->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)c->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)c->dig_P7) << 4);
    return (uint32_t)p;
}

/* Section 8.2: "Returns humidity in %RH as unsigned 32 bit integer in
 * Q22.10 format (22 integer and 10 fractional bits). Output value of
 * '47445' represents 47445/1024 = 46.333 %RH." */
static uint32_t compensate_humidity(const bme280_t *dev, int32_t adc_H)
{
    const bme280_calib_t *c = &dev->calib;
    int32_t v_x1_u32r;

    v_x1_u32r = (dev->t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)c->dig_H4) << 20) -
                    (((int32_t)c->dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)c->dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)c->dig_H3)) >> 11) +
                       ((int32_t)32768))) >> 10) +
                   ((int32_t)2097152)) * ((int32_t)c->dig_H2) +
                  8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)c->dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12);
}

/* ------------------------------------------------------------------ api -- */

bme280_err_t bme280_init(bme280_t *dev, const bme280_bus_t *bus)
{
    if (dev == NULL || bus == NULL || bus->read_regs == NULL ||
        bus->write_regs == NULL || bus->probe == NULL) {
        return BME280_ERR_BAD_ARG;
    }
    dev->bus = *bus;
    dev->initialised = false;
    dev->t_fine = 0;

    /* Address ACK first: separates "nothing there" from every later
     * failure, which all mean "something answered and then went wrong". */
    int rc = dev->bus.probe(dev->bus.ctx);
    if (rc != BME280_BUS_OK)
        return map_bus_err(rc);

    /* Chip id. The equality check is the whole defence against a stuck
     * bus: SDA held high through a read yields 0xFF with a clean status,
     * and a driver that only checks the transfer result initialises
     * happily against a sensor that is not there. 0x60 or nothing. */
    uint8_t id = 0;
    bme280_err_t err = read_regs(dev, BME280_REG_ID, &id, 1);
    if (err != BME280_OK)
        return err;
    if (id != BME280_CHIP_ID)
        return BME280_ERR_BAD_ID;

    /* Soft reset to a known state, then wait for the NVM copy to finish:
     * status bit 0 (im_update) stays set while calibration data streams
     * from NVM into the register file (section 5.4.4). Reading the
     * calibration bank before it clears reads a half-copied image. The
     * driver never sleeps, so this is a bounded poll; at I2C speeds each
     * status read is ~0.1 ms and the copy takes ~2 ms. */
    err = write_reg(dev, BME280_REG_RESET, BME280_RESET_WORD);
    if (err != BME280_OK)
        return err;
    int polls = 64;
    for (;;) {
        uint8_t status = 0;
        err = read_regs(dev, BME280_REG_STATUS, &status, 1);
        if (err != BME280_OK)
            return err;
        if ((status & 0x01) == 0)
            break;
        if (--polls == 0)
            return BME280_ERR_NOT_READY;
    }

    /* Both calibration banks, each in one burst. */
    uint8_t bank0[26];
    uint8_t bank1[7];
    err = read_regs(dev, BME280_REG_CALIB00, bank0, sizeof bank0);
    if (err != BME280_OK)
        return err;
    err = read_regs(dev, BME280_REG_CALIB26, bank1, sizeof bank1);
    if (err != BME280_OK)
        return err;
    if (!calib_block_plausible(bank0))
        return BME280_ERR_BAD_CALIB;

    parse_calib(&dev->calib, bank0, bank1);
    dev->initialised = true;
    return BME280_OK;
}

bme280_err_t bme280_configure(bme280_t *dev, const bme280_config_t *cfg,
                              bme280_mode_t mode)
{
    if (dev == NULL || cfg == NULL)
        return BME280_ERR_BAD_ARG;
    if (!dev->initialised)
        return BME280_ERR_NOT_INIT;
    if (cfg->oversample_temp > BME280_OVERSAMPLE_X16 ||
        cfg->oversample_press > BME280_OVERSAMPLE_X16 ||
        cfg->oversample_hum > BME280_OVERSAMPLE_X16 ||
        cfg->filter > BME280_FILTER_16 ||
        cfg->standby > BME280_STANDBY_20_MS) {
        return BME280_ERR_BAD_ARG;
    }
    if (mode != BME280_MODE_SLEEP && mode != BME280_MODE_NORMAL)
        return BME280_ERR_BAD_ARG; /* forced is bme280_measure_forced()'s */

    /* Order is load-bearing: section 5.4.3 - "Changes to [ctrl_hum] only
     * become effective after a write operation to ctrl_meas." So
     * ctrl_hum first, config next (writable in sleep mode, which is
     * where reset left us), ctrl_meas last to latch all of it. */
    bme280_err_t err = write_reg(dev, BME280_REG_CTRL_HUM,
                                 (uint8_t)cfg->oversample_hum);
    if (err != BME280_OK)
        return err;
    err = write_reg(dev, BME280_REG_CONFIG,
                    (uint8_t)((uint8_t)cfg->standby << 5 |
                              (uint8_t)cfg->filter << 2));
    if (err != BME280_OK)
        return err;
    return write_reg(dev, BME280_REG_CTRL_MEAS,
                     (uint8_t)((uint8_t)cfg->oversample_temp << 5 |
                               (uint8_t)cfg->oversample_press << 2 |
                               (uint8_t)mode));
}

bme280_err_t bme280_measure_forced(bme280_t *dev, int max_polls,
                                   bme280_sample_t *out)
{
    if (dev == NULL || out == NULL || max_polls < 1)
        return BME280_ERR_BAD_ARG;
    if (!dev->initialised)
        return BME280_ERR_NOT_INIT;

    /* Re-arm the mode bits without disturbing the oversampling fields
     * bme280_configure() set: read-modify-write ctrl_meas. */
    uint8_t ctrl_meas = 0;
    bme280_err_t err = read_regs(dev, BME280_REG_CTRL_MEAS, &ctrl_meas, 1);
    if (err != BME280_OK)
        return err;
    ctrl_meas = (uint8_t)((ctrl_meas & ~0x03u) | BME280_MODE_FORCED);
    err = write_reg(dev, BME280_REG_CTRL_MEAS, ctrl_meas);
    if (err != BME280_OK)
        return err;

    /* Poll status.measuring for completion, at most max_polls reads. The
     * driver owns no clock, so the poll budget is the caller's timeout
     * policy expressed in bus transactions. */
    for (int i = 0; i < max_polls; i++) {
        uint8_t status = 0;
        err = read_regs(dev, BME280_REG_STATUS, &status, 1);
        if (err != BME280_OK)
            return err;
        if ((status & BME280_STATUS_MEASURING) == 0)
            return bme280_read(dev, out);
    }
    return BME280_ERR_NOT_READY;
}

bme280_err_t bme280_read(bme280_t *dev, bme280_sample_t *out)
{
    if (dev == NULL || out == NULL)
        return BME280_ERR_BAD_ARG;
    if (!dev->initialised)
        return BME280_ERR_NOT_INIT;

    /* One 8-byte burst, 0xF7..0xFE. Section 4: "it is strongly
     * recommended to use a burst read", because the sensor shadows the
     * data registers only for the duration of one read - separate reads
     * can mix two measurements. */
    uint8_t d[8];
    bme280_err_t err = read_regs(dev, BME280_REG_DATA, d, sizeof d);
    if (err != BME280_OK)
        return err;

    int32_t adc_P = (int32_t)((uint32_t)d[0] << 12 | (uint32_t)d[1] << 4 |
                              (uint32_t)d[2] >> 4);
    int32_t adc_T = (int32_t)((uint32_t)d[3] << 12 | (uint32_t)d[4] << 4 |
                              (uint32_t)d[5] >> 4);
    int32_t adc_H = (int32_t)((uint32_t)d[6] << 8 | (uint32_t)d[7]);

    /* Temperature first: it produces t_fine, which P and H consume. */
    out->temperature_c100 = compensate_temperature(dev, adc_T);
    out->pressure_q24_8_pa = compensate_pressure(dev, adc_P);
    if (out->pressure_q24_8_pa == 0) {
        /* var1 == 0 in the pressure arithmetic - only reachable with a
         * corrupt dig_P1, since real calibrations keep var1 far from
         * zero. The reading is not a measurement; say so. */
        return BME280_ERR_BAD_CALIB;
    }
    out->humidity_q22_10_rh = compensate_humidity(dev, adc_H);
    return BME280_OK;
}
