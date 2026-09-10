/*
 * test_bme280.c - host suite for main/bme280.c, in evidence order.
 *
 * Stage 0 proves the harness can catch a driver that ignores errors: a
 * deliberately naive init (the tutorial version - fire the reads, check
 * nothing) is run against a stuck bus and an absent device, and the
 * control PASSES only if the naive driver wrongly reports success. Only
 * after the trap is demonstrated real does the real driver get to claim
 * it steps around it.
 *
 * Stage 1: calibration parsing - endianness, the signed/unsigned traps,
 *          the split dig_H4/dig_H5 nibbles.
 * Stage 2: compensation against the datasheet's worked example, checked
 *          two ways (pinned integers + the double-precision reference).
 * Stage 3: the fault sweep - every bus fault walked along every
 *          transaction the driver makes, plus the poll-budget paths.
 * Stage 4: argument checking, mode/config encoding, humidity clamps,
 *          the corrupt-dig_P1 divide guard.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../main/bme280.h"
#include "mock_bus.h"

static int failures = 0;

#define CHECK(cond) do {                                                   \
        if (!(cond)) {                                                     \
            failures++;                                                    \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
        }                                                                  \
    } while (0)

#define CHECK_EQ(a, b) do {                                                \
        long long _a = (long long)(a), _b = (long long)(b);                \
        if (_a != _b) {                                                    \
            failures++;                                                    \
            printf("FAIL %s:%d: %s == %s (%lld != %lld)\n",                \
                   __FILE__, __LINE__, #a, #b, _a, _b);                    \
        }                                                                  \
    } while (0)

static bme280_bus_t ops_for(mock_bus_t *m)
{
    bme280_bus_t b = mock_bus_ops;
    b.ctx = m;
    return b;
}

/* ------------------------------------------------------------- stage 0 --
 * The naive driver. This is what most integrations actually do: run the
 * transfers, never look at the status, never check the id value. It must
 * "succeed" against both faults, or the harness has no trap to prove the
 * real driver escapes.
 */
static int naive_init(const bme280_bus_t *bus, uint8_t *id_out)
{
    (void)bus->probe(bus->ctx);                            /* ignored */
    (void)bus->read_regs(bus->ctx, BME280_REG_ID, id_out, 1); /* ignored */
    return 0;                                              /* "fine!" */
}

static void stage0_controls(void)
{
    printf("--- stage 0: controls - the naive driver must fall into "
           "the traps\n");

    /* Trap A: stuck bus. Every read is 0xFF with a clean status; 0xFF is
     * a perfectly plausible-looking chip id if you never compare it. */
    mock_bus_t m;
    mock_bus_init(&m);
    m.fault = MOCK_FAULT_STUCK_HIGH;
    bme280_bus_t bus = ops_for(&m);

    uint8_t id = 0;
    CHECK_EQ(naive_init(&bus, &id), 0);   /* naive reports success... */
    CHECK_EQ(id, 0xFF);                   /* ...for a sensor that isn't there */
    printf("    control A holds: naive init accepted chip id 0xFF from a "
           "stuck bus\n");

    bme280_t dev;
    CHECK_EQ(bme280_init(&dev, &bus), BME280_ERR_BAD_ID);
    printf("    the real driver rejects it: %s\n",
           bme280_err_str(BME280_ERR_BAD_ID));

    /* Trap B: nothing on the bus at all - every transfer NACKs the
     * address. The naive driver still says yes. */
    mock_bus_init(&m);
    m.fault = MOCK_FAULT_NACK_ADDR;
    id = 0x42;
    CHECK_EQ(naive_init(&bus, &id), 0);   /* success again... */
    CHECK_EQ(id, 0x42);                   /* ...having read nothing at all */
    printf("    control B holds: naive init reported success against an "
           "empty bus\n");

    CHECK_EQ(bme280_init(&dev, &bus), BME280_ERR_NACK);
    printf("    the real driver reports it: %s\n",
           bme280_err_str(BME280_ERR_NACK));
}

/* ------------------------------------------------------------- stage 1 -- */
static void stage1_calibration_parsing(void)
{
    printf("--- stage 1: calibration parsing - endianness, sign, the "
           "dig_H4/H5 split\n");

    mock_bus_t m;
    mock_bus_init(&m);
    bme280_bus_t bus = ops_for(&m);
    bme280_t dev;
    CHECK_EQ(bme280_init(&dev, &bus), BME280_OK);

    /* The worked-example image, parsed back out. */
    CHECK_EQ(dev.calib.dig_T1, 27504);
    CHECK_EQ(dev.calib.dig_T2, 26435);
    CHECK_EQ(dev.calib.dig_T3, -1000);
    CHECK_EQ(dev.calib.dig_P1, 36477);
    CHECK_EQ(dev.calib.dig_P2, -10685);
    CHECK_EQ(dev.calib.dig_P9, 6000);
    CHECK_EQ(dev.calib.dig_H1, 75);
    CHECK_EQ(dev.calib.dig_H2, 362);
    CHECK_EQ(dev.calib.dig_H4, 315);
    CHECK_EQ(dev.calib.dig_H5, 50);
    CHECK_EQ(dev.calib.dig_H6, 30);

    /* The unsigned trap: dig_T1 = 65000 must stay 65000. Parse it as
     * int16_t (the classic mistake) and it becomes -536, t_fine follows,
     * and every output of the sensor is wrong by whole degrees. */
    uint8_t bank0[26];
    memcpy(bank0, &m.regs[BME280_REG_CALIB00], 26);
    bank0[0] = (uint8_t)(65000 & 0xFF);
    bank0[1] = (uint8_t)(65000 >> 8);
    bank0[6] = (uint8_t)(60000 & 0xFF);   /* dig_P1 = 60000, same trap */
    bank0[7] = (uint8_t)(60000 >> 8);
    mock_set_calib_bank0(&m, bank0);
    CHECK_EQ(bme280_init(&dev, &bus), BME280_OK);
    CHECK_EQ(dev.calib.dig_T1, 65000);
    CHECK_EQ(dev.calib.dig_P1, 60000);

    /* The signed trap, other direction: dig_T2 = -32000 must stay
     * negative, not wrap to 33536. */
    bank0[2] = (uint8_t)((-32000) & 0xFF);
    bank0[3] = (uint8_t)(((-32000) >> 8) & 0xFF);
    mock_set_calib_bank0(&m, bank0);
    CHECK_EQ(bme280_init(&dev, &bus), BME280_OK);
    CHECK_EQ(dev.calib.dig_T2, -32000);

    /* The dig_H4/dig_H5 split with negative values. Both are 12-bit
     * two's complement interleaved across 0xE4..0xE6; the sign bit lives
     * in the byte holding bits [11:4], so H4 = -100 (0xF9C) encodes as
     * 0xE4=0xF9 + low nibble 0xC, and H5 = -1 (0xFFF) as 0xE6=0xFF +
     * high nibble 0xF. Get the nibble order backwards (or sign-extend
     * after the OR instead of before the shift) and humidity is garbage
     * only at some temperatures - the worst kind of wrong. */
    uint8_t bank1[7];
    memcpy(bank1, &m.regs[BME280_REG_CALIB26], 7);
    bank1[3] = 0xF9;                       /* H4[11:4] */
    bank1[4] = (uint8_t)((0xF << 4) | 0xC);/* H5[3:0] | H4[3:0] */
    bank1[5] = 0xFF;                       /* H5[11:4] */
    mock_set_calib_bank1(&m, bank1);
    CHECK_EQ(bme280_init(&dev, &bus), BME280_OK);
    CHECK_EQ(dev.calib.dig_H4, -100);
    CHECK_EQ(dev.calib.dig_H5, -1);
}

/* ------------------------------------------------------------- stage 2 --
 * The double-precision reference from datasheet section 8.1 - used here
 * as an independent implementation to check the fixed-point path
 * against, exactly as the datasheet intends.
 */
static double ref_temperature(const bme280_calib_t *c, int32_t adc_T,
                              double *t_fine_out)
{
    double var1 = (((double)adc_T) / 16384.0 - ((double)c->dig_T1) / 1024.0) *
                  ((double)c->dig_T2);
    double var2 = ((((double)adc_T) / 131072.0 - ((double)c->dig_T1) / 8192.0) *
                   (((double)adc_T) / 131072.0 - ((double)c->dig_T1) / 8192.0)) *
                  ((double)c->dig_T3);
    *t_fine_out = var1 + var2;
    return (var1 + var2) / 5120.0;
}

static double ref_pressure(const bme280_calib_t *c, int32_t adc_P,
                           double t_fine)
{
    double var1 = t_fine / 2.0 - 64000.0;
    double var2 = var1 * var1 * ((double)c->dig_P6) / 32768.0;
    var2 = var2 + var1 * ((double)c->dig_P5) * 2.0;
    var2 = (var2 / 4.0) + (((double)c->dig_P4) * 65536.0);
    var1 = (((double)c->dig_P3) * var1 * var1 / 524288.0 +
            ((double)c->dig_P2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)c->dig_P1);
    if (var1 == 0.0)
        return 0.0;
    double p = 1048576.0 - (double)adc_P;
    p = (p - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)c->dig_P9) * p * p / 2147483648.0;
    var2 = p * ((double)c->dig_P8) / 32768.0;
    return p + (var1 + var2 + ((double)c->dig_P7)) / 16.0;
}

static double ref_humidity(const bme280_calib_t *c, int32_t adc_H,
                           double t_fine)
{
    double var_H = t_fine - 76800.0;
    var_H = ((double)adc_H - (((double)c->dig_H4) * 64.0 +
                              ((double)c->dig_H5) / 16384.0 * var_H)) *
            (((double)c->dig_H2) / 65536.0 *
             (1.0 + ((double)c->dig_H6) / 67108864.0 * var_H *
                    (1.0 + ((double)c->dig_H3) / 67108864.0 * var_H)));
    var_H = var_H * (1.0 - ((double)c->dig_H1) * var_H / 524288.0);
    if (var_H > 100.0)
        var_H = 100.0;
    else if (var_H < 0.0)
        var_H = 0.0;
    return var_H;
}

static void stage2_compensation(void)
{
    printf("--- stage 2: compensation vs the datasheet worked example\n");

    mock_bus_t m;
    mock_bus_init(&m);   /* worked-example calibration + raw values */
    bme280_bus_t bus = ops_for(&m);
    bme280_t dev;
    CHECK_EQ(bme280_init(&dev, &bus), BME280_OK);

    bme280_sample_t s;
    CHECK_EQ(bme280_read(&dev, &s), BME280_OK);

    /* The BMP280 datasheet works this exact calibration + adc_T=519888,
     * adc_P=415148 through the double-precision code and states
     * 25.08 degC / 100653.27 Pa (its section 3.12; the BME280's T and P
     * formulae in section 8 are the same arithmetic). Pin the fixed-point
     * results as integers so a regression cannot hide inside a
     * tolerance. */
    CHECK_EQ(s.temperature_c100, 2508);              /* 25.08 degC */
    CHECK_EQ(s.pressure_q24_8_pa, 25767233);         /* 100653.25 Pa */

    /* And cross-check all three against the independent double
     * reference, with the quantisation bounds the datasheet implies:
     * 0.01 degC, 1 Pa, 0.1 %RH. */
    double t_fine;
    double t_ref = ref_temperature(&dev.calib, 519888, &t_fine);
    double p_ref = ref_pressure(&dev.calib, 415148, t_fine);
    double h_ref = ref_humidity(&dev.calib, 32768, t_fine);
    CHECK(fabs(s.temperature_c100 / 100.0 - t_ref) < 0.01);
    CHECK(fabs(s.pressure_q24_8_pa / 256.0 - p_ref) < 1.0);
    CHECK(fabs(s.humidity_q22_10_rh / 1024.0 - h_ref) < 0.1);
    printf("    T=%.2f degC  P=%.2f Pa  H=%.3f %%RH (fixed point, refs "
           "%.2f / %.2f / %.3f)\n",
           s.temperature_c100 / 100.0, s.pressure_q24_8_pa / 256.0,
           s.humidity_q22_10_rh / 1024.0, t_ref, p_ref, h_ref);

    /* A second operating point - cold and dry - so the check isn't
     * pinned to one corner of the curve: adc values for roughly -8 degC
     * at altitude. Also drives t_fine negative territory through the
     * humidity path. */
    mock_set_raw(&m, 400000, 300000, 20000);
    CHECK_EQ(bme280_read(&dev, &s), BME280_OK);
    t_ref = ref_temperature(&dev.calib, 400000, &t_fine);
    p_ref = ref_pressure(&dev.calib, 300000, t_fine);
    h_ref = ref_humidity(&dev.calib, 20000, t_fine);
    CHECK(fabs(s.temperature_c100 / 100.0 - t_ref) < 0.01);
    CHECK(fabs(s.pressure_q24_8_pa / 256.0 - p_ref) < 1.0);
    CHECK(fabs(s.humidity_q22_10_rh / 1024.0 - h_ref) < 0.1);
}

/* ------------------------------------------------------------- stage 3 --
 * Walk every fault along every bus operation of both hot paths. A
 * healthy init is 6 operations (probe, id, reset, status, calib x2); a
 * healthy forced measurement is 4 (ctrl_meas read, ctrl_meas write,
 * status, data). Each op index x each fault must produce the mapped
 * error - this is what "every I2C failure path handled" means, and the
 * coverage gate below confirms no call site's branch was skipped.
 */
static const struct { mock_fault_t fault; bme280_err_t want; } FAULTS[] = {
    { MOCK_FAULT_NACK_ADDR, BME280_ERR_NACK },
    { MOCK_FAULT_NACK_DATA, BME280_ERR_NACK },
    { MOCK_FAULT_TIMEOUT,   BME280_ERR_TIMEOUT },
    { MOCK_FAULT_BUSY,      BME280_ERR_BUSY },
    { MOCK_FAULT_UNKNOWN,   BME280_ERR_NACK },  /* unmapped codes degrade
                                                   to NACK, never to OK */
};

static void stage3_fault_sweep(void)
{
    printf("--- stage 3: fault sweep - every fault at every bus "
           "operation\n");

    enum { INIT_OPS = 6, MEASURE_OPS = 4 };
    int cases = 0;

    for (size_t f = 0; f < sizeof FAULTS / sizeof FAULTS[0]; f++) {
        for (int op = 0; op < INIT_OPS; op++) {
            mock_bus_t m;
            mock_bus_init(&m);
            m.fault = FAULTS[f].fault;
            m.fault_on_op = op;
            bme280_bus_t bus = ops_for(&m);
            bme280_t dev;
            bme280_err_t err = bme280_init(&dev, &bus);
            if (err != FAULTS[f].want) {
                failures++;
                printf("FAIL init: fault %d at op %d -> %s, wanted %s\n",
                       (int)FAULTS[f].fault, op, bme280_err_str(err),
                       bme280_err_str(FAULTS[f].want));
            }
            cases++;
        }
        for (int op = 0; op < MEASURE_OPS; op++) {
            mock_bus_t m;
            mock_bus_init(&m);
            bme280_bus_t bus = ops_for(&m);
            bme280_t dev;
            CHECK_EQ(bme280_init(&dev, &bus), BME280_OK);
            m.fault = FAULTS[f].fault;      /* armed only after init */
            m.fault_on_op = m.op_count + op;
            bme280_sample_t s;
            bme280_err_t err = bme280_measure_forced(&dev, 4, &s);
            if (err != FAULTS[f].want) {
                failures++;
                printf("FAIL measure: fault %d at op +%d -> %s, wanted %s\n",
                       (int)FAULTS[f].fault, op, bme280_err_str(err),
                       bme280_err_str(FAULTS[f].want));
            }
            cases++;
        }
    }
    /* configure() is three writes; walk a fault along those too. */
    for (size_t f = 0; f < sizeof FAULTS / sizeof FAULTS[0]; f++) {
        for (int op = 0; op < 3; op++) {
            mock_bus_t m2;
            mock_bus_init(&m2);
            bme280_bus_t bus2 = ops_for(&m2);
            bme280_t dev2;
            CHECK_EQ(bme280_init(&dev2, &bus2), BME280_OK);
            m2.fault = FAULTS[f].fault;
            m2.fault_on_op = m2.op_count + op;
            bme280_config_t cfg = {
                .oversample_temp = BME280_OVERSAMPLE_X1,
                .oversample_press = BME280_OVERSAMPLE_X1,
                .oversample_hum = BME280_OVERSAMPLE_X1,
                .filter = BME280_FILTER_OFF,
                .standby = BME280_STANDBY_125_MS,
            };
            bme280_err_t err = bme280_configure(&dev2, &cfg,
                                                BME280_MODE_NORMAL);
            if (err != FAULTS[f].want) {
                failures++;
                printf("FAIL configure: fault %d at op +%d -> %s, wanted %s\n",
                       (int)FAULTS[f].fault, op, bme280_err_str(err),
                       bme280_err_str(FAULTS[f].want));
            }
            cases++;
        }
    }
    printf("    %d fault-injection cases, every one surfaced as the "
           "mapped error\n", cases);

    /* Clock stretching that stays within budget: the conversion is busy
     * for two status polls, then completes - the driver must ride it
     * out, not error. */
    mock_bus_t m;
    mock_bus_init(&m);
    bme280_bus_t bus = ops_for(&m);
    bme280_t dev;
    CHECK_EQ(bme280_init(&dev, &bus), BME280_OK);
    m.status_busy_reads = 2;
    bme280_sample_t s;
    CHECK_EQ(bme280_measure_forced(&dev, 5, &s), BME280_OK);
    CHECK_EQ(m.forced_triggers, 1);

    /* ...and a conversion that outlives the poll budget must say so. */
    mock_bus_init(&m);
    CHECK_EQ(bme280_init(&dev, &bus), BME280_OK);
    m.status_busy_reads = 1000;
    CHECK_EQ(bme280_measure_forced(&dev, 3, &s), BME280_ERR_NOT_READY);

    /* The same two paths through init's post-reset im_update poll. */
    mock_bus_init(&m);
    m.status_busy_reads = 3;              /* NVM copy takes three polls */
    CHECK_EQ(bme280_init(&dev, &bus), BME280_OK);
    mock_bus_init(&m);
    m.status_busy_reads = 1000;           /* NVM copy never finishes */
    CHECK_EQ(bme280_init(&dev, &bus), BME280_ERR_NOT_READY);
}

/* ------------------------------------------------------------- stage 4 -- */
static void stage4_error_model_details(void)
{
    printf("--- stage 4: identity, argument and range checking\n");

    mock_bus_t m;
    mock_bus_init(&m);
    bme280_bus_t bus = ops_for(&m);
    bme280_t dev;

    /* A BMP280 answers at the same addresses with id 0x58 - close enough
     * to fool anyone matching "did I get an id", wrong enough to have no
     * humidity registers. Identity means equality. */
    m.regs[BME280_REG_ID] = 0x58;
    CHECK_EQ(bme280_init(&dev, &bus), BME280_ERR_BAD_ID);

    /* Erased-NVM device: transfers ACK, calibration reads all zeros. */
    mock_bus_init(&m);
    uint8_t zeros[26] = { 0 };
    mock_set_calib_bank0(&m, zeros);
    CHECK_EQ(bme280_init(&dev, &bus), BME280_ERR_BAD_CALIB);

    /* Corrupt dig_P1 = 0: the datasheet's own division guard fires and
     * the driver refuses to present 0 Pa as weather. */
    mock_bus_init(&m);
    m.regs[BME280_REG_CALIB00 + 6] = 0;
    m.regs[BME280_REG_CALIB00 + 7] = 0;
    CHECK_EQ(bme280_init(&dev, &bus), BME280_OK);
    bme280_sample_t s;
    CHECK_EQ(bme280_read(&dev, &s), BME280_ERR_BAD_CALIB);

    /* Argument checking, every arm. */
    mock_bus_init(&m);
    CHECK_EQ(bme280_init(NULL, &bus), BME280_ERR_BAD_ARG);
    CHECK_EQ(bme280_init(&dev, NULL), BME280_ERR_BAD_ARG);
    bme280_bus_t broken = bus;
    broken.read_regs = NULL;
    CHECK_EQ(bme280_init(&dev, &broken), BME280_ERR_BAD_ARG);
    broken = bus;
    broken.write_regs = NULL;
    CHECK_EQ(bme280_init(&dev, &broken), BME280_ERR_BAD_ARG);
    broken = bus;
    broken.probe = NULL;
    CHECK_EQ(bme280_init(&dev, &broken), BME280_ERR_BAD_ARG);

    bme280_config_t cfg = {
        .oversample_temp = BME280_OVERSAMPLE_X2,
        .oversample_press = BME280_OVERSAMPLE_X16,
        .oversample_hum = BME280_OVERSAMPLE_X1,
        .filter = BME280_FILTER_16,
        .standby = BME280_STANDBY_500_MS,
    };

    /* Calls before init must refuse. */
    bme280_t cold;
    memset(&cold, 0, sizeof cold);
    cold.bus = bus;
    CHECK_EQ(bme280_configure(&cold, &cfg, BME280_MODE_NORMAL),
             BME280_ERR_NOT_INIT);
    CHECK_EQ(bme280_read(&cold, &s), BME280_ERR_NOT_INIT);
    CHECK_EQ(bme280_measure_forced(&cold, 4, &s), BME280_ERR_NOT_INIT);

    CHECK_EQ(bme280_init(&dev, &bus), BME280_OK);
    CHECK_EQ(bme280_configure(NULL, &cfg, BME280_MODE_NORMAL),
             BME280_ERR_BAD_ARG);
    CHECK_EQ(bme280_configure(&dev, NULL, BME280_MODE_NORMAL),
             BME280_ERR_BAD_ARG);
    CHECK_EQ(bme280_configure(&dev, &cfg, BME280_MODE_FORCED),
             BME280_ERR_BAD_ARG);   /* forced goes via measure_forced() */

    bme280_config_t bad = cfg;
    bad.oversample_temp = (bme280_oversample_t)6;
    CHECK_EQ(bme280_configure(&dev, &bad, BME280_MODE_NORMAL),
             BME280_ERR_BAD_ARG);
    bad = cfg;
    bad.oversample_press = (bme280_oversample_t)7;
    CHECK_EQ(bme280_configure(&dev, &bad, BME280_MODE_NORMAL),
             BME280_ERR_BAD_ARG);
    bad = cfg;
    bad.oversample_hum = (bme280_oversample_t)6;
    CHECK_EQ(bme280_configure(&dev, &bad, BME280_MODE_NORMAL),
             BME280_ERR_BAD_ARG);
    bad = cfg;
    bad.filter = (bme280_filter_t)5;
    CHECK_EQ(bme280_configure(&dev, &bad, BME280_MODE_NORMAL),
             BME280_ERR_BAD_ARG);
    bad = cfg;
    bad.standby = (bme280_standby_t)8;
    CHECK_EQ(bme280_configure(&dev, &bad, BME280_MODE_NORMAL),
             BME280_ERR_BAD_ARG);

    CHECK_EQ(bme280_read(&dev, NULL), BME280_ERR_BAD_ARG);
    CHECK_EQ(bme280_read(NULL, &s), BME280_ERR_BAD_ARG);
    CHECK_EQ(bme280_measure_forced(&dev, 0, &s), BME280_ERR_BAD_ARG);
    CHECK_EQ(bme280_measure_forced(&dev, 4, NULL), BME280_ERR_BAD_ARG);
    CHECK_EQ(bme280_measure_forced(NULL, 4, &s), BME280_ERR_BAD_ARG);

    /* Register encoding: what configure() wrote must match section 5.4's
     * field layout bit for bit. */
    CHECK_EQ(bme280_configure(&dev, &cfg, BME280_MODE_NORMAL), BME280_OK);
    CHECK_EQ(m.regs[BME280_REG_CTRL_HUM], 0x01);   /* osrs_h = x1  */
    CHECK_EQ(m.regs[BME280_REG_CONFIG], 0x90);     /* 500ms | f16  */
    CHECK_EQ(m.regs[BME280_REG_CTRL_MEAS], 0x57);  /* x2 | x16 | normal */

    /* Forced trigger must preserve the oversampling fields. */
    CHECK_EQ(bme280_configure(&dev, &cfg, BME280_MODE_SLEEP), BME280_OK);
    CHECK_EQ(bme280_measure_forced(&dev, 4, &s), BME280_OK);
    CHECK_EQ(m.regs[BME280_REG_CTRL_MEAS] & 0xFC, 0x54);
    CHECK_EQ(m.regs[BME280_REG_CTRL_MEAS] & 0x03, BME280_MODE_FORCED);

    /* Humidity clamp arms: the section 8.2 code clamps its accumulator
     * to [0, 419430400] (0..100 %RH in Q22.10 << 12). Drive both. */
    mock_bus_init(&m);
    CHECK_EQ(bme280_init(&dev, &bus), BME280_OK);
    mock_set_raw(&m, 519888, 415148, 0);           /* bone dry: clamps low */
    CHECK_EQ(bme280_read(&dev, &s), BME280_OK);
    CHECK_EQ(s.humidity_q22_10_rh, 0);

    /* Oversaturated: a pegged ADC pushes the accumulator past the
     * 100 %RH ceiling and the upper clamp must hold it there. */
    mock_set_raw(&m, 519888, 415148, 65535);
    CHECK_EQ(bme280_read(&dev, &s), BME280_OK);
    CHECK_EQ(s.humidity_q22_10_rh, 419430400 >> 12);  /* pegged at 100% */

    /* Every error string resolves, including one that doesn't exist. */
    for (int e = 0; e <= (int)BME280_ERR_NOT_INIT; e++)
        CHECK(bme280_err_str((bme280_err_t)e) != NULL);
    CHECK(strcmp(bme280_err_str((bme280_err_t)999), "unknown error") == 0);
}

int main(void)
{
    stage0_controls();
    stage1_calibration_parsing();
    stage2_compensation();
    stage3_fault_sweep();
    stage4_error_model_details();

    if (failures) {
        printf("\n%d FAILURE(S)\n", failures);
        return 1;
    }
    printf("\nALL BME280 HOST TESTS PASS\n");
    return 0;
}
