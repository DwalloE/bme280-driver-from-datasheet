/*
 * browser-demo/sketch.ino - Arduino-core port of bme280-driver-from-
 * datasheet, for the shareable Wokwi project. A DEMO, not the driver.
 *
 * Wokwi's browser editor compiles Arduino, not ESP-IDF, so this port
 * exists to give the README a click-to-run link. Two honest differences
 * from the canonical code in main/:
 *
 *  1. The transport here is Wire.h - the Arduino core's I2C driver -
 *     because the point of the browser demo is the sensor and the shell
 *     responding, not the bus layer. The canonical driver takes an
 *     injected ops struct and its every failure path is fault-injected
 *     and coverage-gated on the host; none of that machinery is here.
 *  2. The register map, calibration parsing (including the split
 *     dig_H4/dig_H5) and section 8.2 fixed-point compensation are the
 *     same arithmetic, pasted inline (a sketch is one file). If the two
 *     ever disagree, main/bme280.c is right.
 *
 * Type into the serial monitor:  r = read   i = chip id
 *                                c = calibration dump   n = NACK demo
 */
#include <Arduino.h>
#include <Wire.h>

#define BME_ADDR   0x76
#define REG_ID     0xD0
#define REG_RESET  0xE0
#define REG_CTRLH  0xF2
#define REG_STATUS 0xF3
#define REG_CTRLM  0xF4
#define REG_DATA   0xF7

/* ---- calibration, parsed exactly as main/bme280.c does ------------------ */
static uint16_t dig_T1;
static int16_t dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
static uint8_t dig_H1, dig_H3;
static int16_t dig_H2, dig_H4, dig_H5;
static int8_t dig_H6;
static int32_t t_fine;

static bool readRegs(uint8_t reg, uint8_t *buf, size_t len)
{
  Wire.beginTransmission(BME_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)BME_ADDR, (int)len) != (int)len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

static bool writeReg(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(BME_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static uint16_t leU16(const uint8_t *p) { return (uint16_t)(p[1] << 8 | p[0]); }
static int16_t leS16(const uint8_t *p) { return (int16_t)leU16(p); }

static bool readCalibration()
{
  uint8_t b0[26], b1[7];
  if (!readRegs(0x88, b0, 26) || !readRegs(0xE1, b1, 7)) return false;
  dig_T1 = leU16(b0 + 0);   /* UNSIGNED - the classic trap */
  dig_T2 = leS16(b0 + 2);
  dig_T3 = leS16(b0 + 4);
  dig_P1 = leU16(b0 + 6);   /* also unsigned */
  dig_P2 = leS16(b0 + 8);  dig_P3 = leS16(b0 + 10); dig_P4 = leS16(b0 + 12);
  dig_P5 = leS16(b0 + 14); dig_P6 = leS16(b0 + 16); dig_P7 = leS16(b0 + 18);
  dig_P8 = leS16(b0 + 20); dig_P9 = leS16(b0 + 22);
  dig_H1 = b0[25];
  dig_H2 = leS16(b1 + 0);
  dig_H3 = b1[2];
  /* the split: 0xE4=H4[11:4], 0xE5 low=H4[3:0], 0xE5 high=H5[3:0],
     0xE6=H5[11:4]; the sign bit lives in the [11:4] byte */
  dig_H4 = (int16_t)(((int16_t)(int8_t)b1[3] << 4) | (b1[4] & 0x0F));
  dig_H5 = (int16_t)(((int16_t)(int8_t)b1[5] << 4) | (b1[4] >> 4));
  dig_H6 = (int8_t)b1[6];
  return true;
}

/* ---- section 8.2 fixed point, same arithmetic as main/bme280.c ---------- */
static int32_t compT(int32_t adc_T)
{
  int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) *
                  ((int32_t)dig_T2)) >> 11;
  int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) *
                    ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
                  ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  return (t_fine * 5 + 128) >> 8;
}

static uint32_t compP(int32_t adc_P)
{
  int64_t var1 = ((int64_t)t_fine) - 128000;
  int64_t var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) +
         ((var1 * (int64_t)dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;
  if (var1 == 0) return 0;
  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4);
  return (uint32_t)p;
}

static uint32_t compH(int32_t adc_H)
{
  int32_t v = (t_fine - ((int32_t)76800));
  v = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) -
          (((int32_t)dig_H5) * v)) + ((int32_t)16384)) >> 15) *
       (((((((v * ((int32_t)dig_H6)) >> 10) *
            (((v * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
         ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
  v = (v - (((((v >> 15) * (v >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
  v = (v < 0 ? 0 : v);
  v = (v > 419430400 ? 419430400 : v);
  return (uint32_t)(v >> 12);
}

/* ---- shell --------------------------------------------------------------- */
static void doRead()
{
  /* forced mode: one conversion, then read all 8 data bytes in a burst */
  writeReg(REG_CTRLH, 0x01);          /* osrs_h x1 */
  writeReg(REG_CTRLM, 0x25);          /* osrs_t x1, osrs_p x1, forced */
  uint8_t status = 0x08;
  for (int i = 0; i < 50 && (status & 0x08); i++) {
    readRegs(REG_STATUS, &status, 1);
    delay(2);
  }
  uint8_t d[8];
  if (!readRegs(REG_DATA, d, 8)) { Serial.println("read failed (bus)"); return; }
  int32_t adc_P = (int32_t)((uint32_t)d[0] << 12 | (uint32_t)d[1] << 4 | d[2] >> 4);
  int32_t adc_T = (int32_t)((uint32_t)d[3] << 12 | (uint32_t)d[4] << 4 | d[5] >> 4);
  int32_t adc_H = (int32_t)((uint32_t)d[6] << 8 | d[7]);
  int32_t t = compT(adc_T);
  uint32_t p = compP(adc_P);
  uint32_t h = compH(adc_H);
  Serial.printf("T=%ld.%02d C  P=%.1f Pa  H=%.1f %%RH\n",
                (long)(t / 100), (int)abs((int)(t % 100)),
                p / 256.0, h / 1024.0);
}

static void doId()
{
  uint8_t id = 0;
  if (!readRegs(REG_ID, &id, 1)) { Serial.println("id read failed (bus)"); return; }
  Serial.printf("chip id 0x%02X - %s\n", id,
                id == 0x60 ? "BME280, verified" :
                id == 0x58 ? "that's a BMP280 (no humidity)!" :
                id == 0xFF ? "0xFF = stuck/empty bus, NOT a sensor" :
                "unknown part");
}

static void doCal()
{
  Serial.printf("dig_T1=%u (unsigned!) dig_T2=%d dig_T3=%d\n",
                dig_T1, dig_T2, dig_T3);
  Serial.printf("dig_P1=%u (unsigned!) dig_P2=%d ... dig_P9=%d\n",
                dig_P1, dig_P2, dig_P9);
  Serial.printf("dig_H1=%u dig_H2=%d dig_H3=%u dig_H4=%d dig_H5=%d dig_H6=%d\n",
                dig_H1, dig_H2, dig_H3, dig_H4, dig_H5, dig_H6);
  Serial.println("H4/H5 are 12-bit values interleaved across 0xE4..0xE6");
}

static void doNack()
{
  Wire.beginTransmission(0x77);       /* SDO-high address: nothing there */
  uint8_t rc = Wire.endTransmission();
  Serial.printf("probe 0x77 -> endTransmission()=%u: %s\n", rc,
                rc == 0 ? "ACK?!" : "address NACK - nothing lives there");
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(21, 22);
  Serial.println("\nBME280 from the datasheet - browser demo (Arduino port)");
  uint8_t id = 0;
  if (!readRegs(REG_ID, &id, 1) || id != 0x60) {
    Serial.printf("init failed: chip id 0x%02X (want 0x60)\n", id);
    return;
  }
  writeReg(REG_RESET, 0xB6);
  delay(5);
  if (!readCalibration()) { Serial.println("calibration read failed"); return; }
  Serial.println("chip id 0x60 verified, calibration loaded");
  Serial.println("commands: r=read  i=chip id  c=calibration  n=NACK demo");
  doRead();
}

void loop()
{
  if (!Serial.available()) return;
  char cmd = Serial.read();
  switch (cmd) {
    case 'r': doRead(); break;
    case 'i': doId(); break;
    case 'c': doCal(); break;
    case 'n': doNack(); break;
    case '\n': case '\r': break;
    default: Serial.println("commands: r i c n"); break;
  }
}
