/*
 * bme280.chip.c - a Wokwi custom chip simulating the Bosch BME280 on I2C,
 * written from the same datasheet (BST-BME280-DS002) as the driver it
 * exists to exercise. Wokwi has no built-in BME280 part (the closest is
 * the BMP180), so this repo ships its own sensor.
 *
 * Behaviour implemented from the datasheet, not from the driver - the
 * point is that two independent readings of the document meet on the bus:
 *
 *   - I2C register file with the real access protocol: writes are
 *     (register, value) pairs (section 6.2), reads auto-increment.
 *   - chip id 0x60 at 0xD0; soft reset via 0xB6 at 0xE0 (section 5.4).
 *   - forced mode: a ctrl_meas write with mode=01 raises status.measuring
 *     for a realistic ~8 ms conversion, then latches fresh data and drops
 *     back to sleep. Normal mode free-runs at the config register's
 *     standby period (table 27).
 *   - the calibration banks hold the datasheet worked-example image (the
 *     same one the host tests pin against), so a reading from this chip
 *     is comparable end to end.
 *
 * The diagram attrs are physical units (temperature degC, pressure Pa,
 * humidity %RH). At each conversion the chip inverts the section 8.2
 * compensation numerically - binary search over the 20-bit ADC space,
 * legal because each output is monotonic in its raw input - so whatever
 * the firmware computes from the raw registers lands back on the attr
 * values. If driver and chip disagree about section 8.2, the readings
 * are visibly wrong; agreement is the test.
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "wokwi-api.h"

#define ADDR 0x76

/* Registers */
#define REG_CALIB00 0x88
#define REG_ID      0xD0
#define REG_RESET   0xE0
#define REG_CALIB26 0xE1
#define REG_CTRLH   0xF2
#define REG_STATUS  0xF3
#define REG_CTRLM   0xF4
#define REG_CONFIG  0xF5
#define REG_DATA    0xF7

/* The worked-example calibration (same image as test/mock_bus.c). */
#define DIG_T1 27504
#define DIG_T2 26435
#define DIG_T3 (-1000)
#define DIG_P1 36477
#define DIG_P2 (-10685)
#define DIG_P3 3024
#define DIG_P4 2855
#define DIG_P5 140
#define DIG_P6 (-7)
#define DIG_P7 15500
#define DIG_P8 (-14600)
#define DIG_P9 6000
#define DIG_H1 75
#define DIG_H2 362
#define DIG_H3 0
#define DIG_H4 315
#define DIG_H5 50
#define DIG_H6 30

typedef struct {
  uint8_t regs[256];
  uint8_t reg_ptr;
  bool expect_reg;      /* next written byte is a register address */
  bool measuring;
  timer_t conv_timer;   /* forced-mode conversion delay */
  timer_t normal_timer; /* normal-mode sampling period */
  uint32_t attr_t, attr_p, attr_h;
} chip_t;

/* ---- section 8.2 fixed point (the ground truth this chip inverts) ------- */
static int32_t comp_t(int32_t adc_T, int32_t *t_fine)
{
  int32_t var1 = ((((adc_T >> 3) - ((int32_t)DIG_T1 << 1))) *
                  ((int32_t)DIG_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)DIG_T1)) *
                    ((adc_T >> 4) - ((int32_t)DIG_T1))) >> 12) *
                  ((int32_t)DIG_T3)) >> 14;
  *t_fine = var1 + var2;
  return (*t_fine * 5 + 128) >> 8;
}

static uint32_t comp_p(int32_t adc_P, int32_t t_fine)
{
  int64_t var1 = ((int64_t)t_fine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)DIG_P6;
  var2 = var2 + ((var1 * (int64_t)DIG_P5) << 17);
  var2 = var2 + (((int64_t)DIG_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)DIG_P3) >> 8) +
         ((var1 * (int64_t)DIG_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)DIG_P1) >> 33;
  if (var1 == 0) return 0;
  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)DIG_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)DIG_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)DIG_P7) << 4);
  return (uint32_t)p;
}

static uint32_t comp_h(int32_t adc_H, int32_t t_fine)
{
  int32_t v = (t_fine - ((int32_t)76800));
  v = (((((adc_H << 14) - (((int32_t)DIG_H4) << 20) -
          (((int32_t)DIG_H5) * v)) + ((int32_t)16384)) >> 15) *
       (((((((v * ((int32_t)DIG_H6)) >> 10) *
            (((v * ((int32_t)DIG_H3)) >> 11) + ((int32_t)32768))) >> 10) +
         ((int32_t)2097152)) * ((int32_t)DIG_H2) + 8192) >> 14));
  v = (v - (((((v >> 15) * (v >> 15)) >> 7) * ((int32_t)DIG_H1)) >> 4));
  v = (v < 0 ? 0 : v);
  v = (v > 419430400 ? 419430400 : v);
  return (uint32_t)(v >> 12);
}

/* ---- numeric inversion: find the raw value that compensates to target --- */
static int32_t invert_t(int32_t target_c100, int32_t *t_fine)
{
  int32_t lo = 0, hi = 0xFFFFF; /* increasing in adc_T (DIG_T2 > 0) */
  while (lo < hi) {
    int32_t mid = lo + (hi - lo) / 2;
    int32_t tf;
    if (comp_t(mid, &tf) < target_c100) lo = mid + 1; else hi = mid;
  }
  comp_t(lo, t_fine);
  return lo;
}

static int32_t invert_p(uint32_t target_q24_8, int32_t t_fine)
{
  int32_t lo = 0, hi = 0xFFFFF; /* DECREASING in adc_P */
  while (lo < hi) {
    int32_t mid = lo + (hi - lo) / 2;
    if (comp_p(mid, t_fine) > target_q24_8) lo = mid + 1; else hi = mid;
  }
  return lo;
}

static int32_t invert_h(uint32_t target_q22_10, int32_t t_fine)
{
  int32_t lo = 0, hi = 0xFFFF; /* increasing in adc_H (DIG_H2 > 0) */
  while (lo < hi) {
    int32_t mid = lo + (hi - lo) / 2;
    if (comp_h(mid, t_fine) < target_q22_10) lo = mid + 1; else hi = mid;
  }
  return lo;
}

/* ---- register file --------------------------------------------------- */
static void put_le16(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }

static void load_defaults(chip_t *chip)
{
  memset(chip->regs, 0, sizeof chip->regs);
  chip->regs[REG_ID] = 0x60;

  uint8_t *b0 = &chip->regs[REG_CALIB00];
  put_le16(b0 + 0, DIG_T1);
  put_le16(b0 + 2, (uint16_t)(int16_t)DIG_T2);
  put_le16(b0 + 4, (uint16_t)(int16_t)DIG_T3);
  put_le16(b0 + 6, DIG_P1);
  put_le16(b0 + 8, (uint16_t)(int16_t)DIG_P2);
  put_le16(b0 + 10, (uint16_t)(int16_t)DIG_P3);
  put_le16(b0 + 12, (uint16_t)(int16_t)DIG_P4);
  put_le16(b0 + 14, (uint16_t)(int16_t)DIG_P5);
  put_le16(b0 + 16, (uint16_t)(int16_t)DIG_P6);
  put_le16(b0 + 18, (uint16_t)(int16_t)DIG_P7);
  put_le16(b0 + 20, (uint16_t)(int16_t)DIG_P8);
  put_le16(b0 + 22, (uint16_t)(int16_t)DIG_P9);
  b0[25] = DIG_H1;                              /* 0xA1 */

  uint8_t *b1 = &chip->regs[REG_CALIB26];
  put_le16(b1 + 0, (uint16_t)(int16_t)DIG_H2);
  b1[2] = DIG_H3;
  b1[3] = (uint8_t)(DIG_H4 >> 4);               /* 0xE4 = H4[11:4] */
  b1[4] = (uint8_t)(((DIG_H5 & 0x0F) << 4) | (DIG_H4 & 0x0F)); /* 0xE5 */
  b1[5] = (uint8_t)(DIG_H5 >> 4);               /* 0xE6 = H5[11:4] */
  b1[6] = (uint8_t)DIG_H6;                      /* 0xE7 */

  /* Data registers before any conversion: the "skipped" pattern. */
  chip->regs[REG_DATA + 0] = 0x80; /* press msb */
  chip->regs[REG_DATA + 3] = 0x80; /* temp msb  */
  chip->regs[REG_DATA + 6] = 0x80; /* hum msb   */
  chip->measuring = false;
}

static void latch_measurement(chip_t *chip)
{
  float t_c = attr_read_float(chip->attr_t);
  float p_pa = attr_read_float(chip->attr_p);
  float h_rh = attr_read_float(chip->attr_h);

  int32_t t_fine;
  int32_t adc_T = invert_t((int32_t)(t_c * 100.0f + (t_c >= 0 ? 0.5f : -0.5f)),
                           &t_fine);
  int32_t adc_P = invert_p((uint32_t)(p_pa * 256.0f + 0.5f), t_fine);
  int32_t adc_H = invert_h((uint32_t)(h_rh * 1024.0f + 0.5f), t_fine);

  uint8_t *d = &chip->regs[REG_DATA];
  d[0] = (uint8_t)(adc_P >> 12); d[1] = (uint8_t)(adc_P >> 4);
  d[2] = (uint8_t)(adc_P << 4);
  d[3] = (uint8_t)(adc_T >> 12); d[4] = (uint8_t)(adc_T >> 4);
  d[5] = (uint8_t)(adc_T << 4);
  d[6] = (uint8_t)(adc_H >> 8);  d[7] = (uint8_t)(adc_H);
}

/* Table 27: t_sb field -> standby period in microseconds. */
static uint32_t standby_us(uint8_t config_reg)
{
  static const uint32_t us[8] = { 500, 62500, 125000, 250000, 500000,
                                  1000000, 10000, 20000 };
  return us[(config_reg >> 5) & 0x07];
}

/* ---- timers ----------------------------------------------------------- */
static void on_conversion_done(void *user_data)
{
  chip_t *chip = user_data;
  latch_measurement(chip);
  chip->measuring = false;
  chip->regs[REG_CTRLM] &= (uint8_t)~0x03; /* forced returns to sleep */
}

static void on_normal_tick(void *user_data)
{
  chip_t *chip = user_data;
  latch_measurement(chip);
}

static void handle_ctrl_meas(chip_t *chip, uint8_t val)
{
  uint8_t mode = val & 0x03;
  timer_stop(chip->normal_timer);
  if (mode == 0x01 || mode == 0x02) {          /* forced (01 or 10) */
    chip->measuring = true;
    timer_start(chip->conv_timer, 8000, false); /* ~8 ms, table 13-ish */
  } else if (mode == 0x03) {                    /* normal */
    latch_measurement(chip);
    timer_start(chip->normal_timer,
                standby_us(chip->regs[REG_CONFIG]) + 8000, true);
  }
}

/* ---- I2C protocol ------------------------------------------------------ */
static bool on_connect(void *user_data, uint32_t address, bool read)
{
  chip_t *chip = user_data;
  (void)address; (void)read;
  chip->expect_reg = !read; /* a write transaction starts with a register */
  return true;              /* ACK; 0x77 never reaches us, so it NACKs */
}

static uint8_t on_read(void *user_data)
{
  chip_t *chip = user_data;
  uint8_t reg = chip->reg_ptr;
  uint8_t val;
  if (reg == REG_STATUS)
    val = chip->measuring ? 0x08 : 0x00;
  else
    val = chip->regs[reg];
  chip->reg_ptr = (uint8_t)(reg + 1); /* reads auto-increment (s 6.2) */
  return val;
}

static bool on_write(void *user_data, uint8_t data)
{
  chip_t *chip = user_data;
  if (chip->expect_reg) {
    chip->reg_ptr = data;
    chip->expect_reg = false;
    return true;
  }
  /* Value byte for reg_ptr; writes are (register, value) pairs, so the
   * next byte is a register address again (section 6.2). */
  uint8_t reg = chip->reg_ptr;
  chip->expect_reg = true;
  switch (reg) {
  case REG_RESET:
    if (data == 0xB6) {
      timer_stop(chip->conv_timer);
      timer_stop(chip->normal_timer);
      load_defaults(chip);
    }
    return true;
  case REG_CTRLH:
  case REG_CONFIG:
    chip->regs[reg] = data;
    return true;
  case REG_CTRLM:
    chip->regs[reg] = data;
    handle_ctrl_meas(chip, data);
    return true;
  default:
    return true; /* read-only or reserved: ACK, ignore - as silicon does */
  }
}

void chip_init(void)
{
  static chip_t chip;
  load_defaults(&chip);

  chip.attr_t = attr_init_float("temperature", 24.5f);
  chip.attr_p = attr_init_float("pressure", 101325.0f);
  chip.attr_h = attr_init_float("humidity", 42.0f);

  const timer_config_t conv_cfg = {
    .user_data = &chip, .callback = on_conversion_done,
  };
  chip.conv_timer = timer_init(&conv_cfg);
  const timer_config_t norm_cfg = {
    .user_data = &chip, .callback = on_normal_tick,
  };
  chip.normal_timer = timer_init(&norm_cfg);

  const i2c_config_t i2c = {
    .user_data = &chip,
    .address = ADDR,
    .scl = pin_init("SCL", INPUT),
    .sda = pin_init("SDA", INPUT),
    .connect = on_connect,
    .read = on_read,
    .write = on_write,
  };
  i2c_init(&i2c);

  printf("bme280 chip: id 0x60 at 0x%02X, worked-example calibration\n",
         ADDR);
}
