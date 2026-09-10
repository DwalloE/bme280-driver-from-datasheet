/*
 * mock_bus.c - see mock_bus.h. The mock is deliberately dumb: a register
 * file, an auto-incrementing pointer like the real part's, and one
 * scripted fault. All the intelligence stays in the tests.
 */
#include <string.h>
#include "mock_bus.h"

/* Should this operation fail? Counts every bus operation, so a test can
 * walk the fault along the driver's whole transaction sequence. */
static int fire_fault(mock_bus_t *m)
{
    int this_op = m->op_count++;
    if (m->fault == MOCK_FAULT_NONE || m->fault == MOCK_FAULT_STUCK_HIGH)
        return 0;
    if (m->fault_on_op != -1 && m->fault_on_op != this_op)
        return 0;
    switch (m->fault) {
    case MOCK_FAULT_NACK_ADDR: return BME280_BUS_ERR_NACK_ADDR;
    case MOCK_FAULT_NACK_DATA: return BME280_BUS_ERR_NACK_DATA;
    case MOCK_FAULT_TIMEOUT:   return BME280_BUS_ERR_TIMEOUT;
    case MOCK_FAULT_BUSY:      return BME280_BUS_ERR_BUSY;
    default:                   return -99; /* MOCK_FAULT_UNKNOWN */
    }
}

static int mock_read_regs(void *ctx, uint8_t reg, uint8_t *buf, size_t len)
{
    mock_bus_t *m = ctx;
    int rc = fire_fault(m);
    if (rc != 0)
        return rc;
    if (m->fault == MOCK_FAULT_STUCK_HIGH) {
        /* The trap: SDA reads high forever, the transport layer reports
         * success, and every byte is 0xFF. Looks exactly like data. */
        memset(buf, 0xFF, len);
        return BME280_BUS_OK;
    }
    for (size_t i = 0; i < len; i++) {
        uint8_t r = (uint8_t)(reg + i);
        if (r == BME280_REG_STATUS) {
            buf[i] = m->status_busy_reads > 0 ? 0x09 /* measuring|im_update */
                                              : 0x00;
            if (m->status_busy_reads > 0)
                m->status_busy_reads--;
        } else {
            buf[i] = m->regs[r];
        }
    }
    return BME280_BUS_OK;
}

static int mock_write_regs(void *ctx, uint8_t reg, const uint8_t *buf,
                           size_t len)
{
    mock_bus_t *m = ctx;
    int rc = fire_fault(m);
    if (rc != 0)
        return rc;
    if (m->fault == MOCK_FAULT_STUCK_HIGH)
        return BME280_BUS_OK; /* "succeeds", changes nothing */
    for (size_t i = 0; i < len; i++) {
        uint8_t r = (uint8_t)(reg + i);
        m->regs[r] = buf[i];
        if (r == BME280_REG_CTRL_MEAS && (buf[i] & 0x03) == BME280_MODE_FORCED)
            m->forced_triggers++;
    }
    return BME280_BUS_OK;
}

static int mock_probe(void *ctx)
{
    mock_bus_t *m = ctx;
    int rc = fire_fault(m);
    if (rc != 0)
        return rc;
    return BME280_BUS_OK;
}

/* Exposed as a template: tests copy it and point .ctx at their mock. */
const bme280_bus_t mock_bus_ops = {
    .ctx = NULL,
    .read_regs = mock_read_regs,
    .write_regs = mock_write_regs,
    .probe = mock_probe,
};

void mock_set_calib_bank0(mock_bus_t *m, const uint8_t bank0[26])
{
    memcpy(&m->regs[BME280_REG_CALIB00], bank0, 26);
}

void mock_set_calib_bank1(mock_bus_t *m, const uint8_t bank1[7])
{
    memcpy(&m->regs[BME280_REG_CALIB26], bank1, 7);
}

void mock_set_raw(mock_bus_t *m, int32_t adc_T, int32_t adc_P, int32_t adc_H)
{
    uint8_t *d = &m->regs[BME280_REG_DATA];
    d[0] = (uint8_t)(adc_P >> 12);
    d[1] = (uint8_t)(adc_P >> 4);
    d[2] = (uint8_t)(adc_P << 4);
    d[3] = (uint8_t)(adc_T >> 12);
    d[4] = (uint8_t)(adc_T >> 4);
    d[5] = (uint8_t)(adc_T << 4);
    d[6] = (uint8_t)(adc_H >> 8);
    d[7] = (uint8_t)(adc_H);
}

/* Little-endian encoders for building calibration images from dig values,
 * so tests state calibration in datasheet units, not raw bytes. */
static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

void mock_bus_init(mock_bus_t *m)
{
    memset(m, 0, sizeof *m);
    m->fault_on_op = -1;

    m->regs[BME280_REG_ID] = BME280_CHIP_ID;

    /* Healthy defaults: the BMP280 datasheet's worked-example T and P
     * calibration (section 3.12 there; T/P formulas are identical to the
     * BME280's section 8.2) plus a plausible humidity set. */
    uint8_t b0[26];
    put_u16(b0 + 0, 27504);            /* dig_T1, unsigned */
    put_u16(b0 + 2, (uint16_t)26435);  /* dig_T2 */
    put_u16(b0 + 4, (uint16_t)(int16_t)-1000);   /* dig_T3 */
    put_u16(b0 + 6, 36477);            /* dig_P1, unsigned */
    put_u16(b0 + 8, (uint16_t)(int16_t)-10685);  /* dig_P2 */
    put_u16(b0 + 10, (uint16_t)3024);  /* dig_P3 */
    put_u16(b0 + 12, (uint16_t)2855);  /* dig_P4 */
    put_u16(b0 + 14, (uint16_t)140);   /* dig_P5 */
    put_u16(b0 + 16, (uint16_t)(int16_t)-7);     /* dig_P6 */
    put_u16(b0 + 18, (uint16_t)15500); /* dig_P7 */
    put_u16(b0 + 20, (uint16_t)(int16_t)-14600); /* dig_P8 */
    put_u16(b0 + 22, (uint16_t)6000);  /* dig_P9 */
    b0[24] = 0;                        /* 0xA0: hole in the map */
    b0[25] = 75;                       /* dig_H1 */
    mock_set_calib_bank0(m, b0);

    /* H2=362 H3=0 H4=315 H5=50 H6=30, laid out per table 16:
     * 0xE4 = H4[11:4], 0xE5 = H5[3:0]<<4 | H4[3:0], 0xE6 = H5[11:4]. */
    uint8_t b1[7];
    put_u16(b1 + 0, (uint16_t)362);    /* dig_H2 */
    b1[2] = 0;                         /* dig_H3 */
    b1[3] = (uint8_t)(315 >> 4);       /* 0xE4 = dig_H4[11:4] */
    b1[4] = (uint8_t)(((50 & 0x0F) << 4) | (315 & 0x0F)); /* 0xE5, shared */
    b1[5] = (uint8_t)(50 >> 4);        /* 0xE6 = dig_H5[11:4] */
    b1[6] = 30;                        /* 0xE7 = dig_H6 */
    mock_set_calib_bank1(m, b1);

    /* Worked-example raw readings: ~25 degC, ~100 kPa, mid-range RH. */
    mock_set_raw(m, 519888, 415148, 32768);
}
