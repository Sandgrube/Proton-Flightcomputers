// BMP388 minimal read over I2C (addr 0x76)
// - checks CHIP_ID (0x00) -> expected 0x50 for BMP388
// - reads raw temp + pressure and prints them (raw + compensated)
// Needs only Wire.h, no external libs.

#include <Arduino.h>
#include <Wire.h>

static const int I2C_SDA = 8;   // set to your board wiring
static const int I2C_SCL = 9;

static const uint8_t BMP_ADDR = 0x76;

// Registers (BMP388)
static const uint8_t REG_CHIP_ID   = 0x00; // should be 0x50
static const uint8_t REG_ERR       = 0x02;
static const uint8_t REG_STATUS    = 0x03;
static const uint8_t REG_PWR_CTRL  = 0x1B;
static const uint8_t REG_OSR       = 0x1C;
static const uint8_t REG_ODR       = 0x1D;
static const uint8_t REG_CONFIG    = 0x1F;
static const uint8_t REG_PRESS_XLSB= 0x04; // 0x04..0x06 (24-bit)
static const uint8_t REG_TEMP_XLSB = 0x07; // 0x07..0x09 (24-bit)
static const uint8_t REG_CALIB0    = 0x31; // 0x31..0x45

// I2C helpers
bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool i2cReadN(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)n) != (int)n) return false;
  for (size_t i=0;i<n;i++) buf[i] = Wire.read();
  return true;
}
bool i2cRead8(uint8_t addr, uint8_t reg, uint8_t &v) {
  return i2cReadN(addr, reg, &v, 1);
}

// Calibration (BMP388)
struct Calib {
  float T1, T2, T3;
  float P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;
} cal;

static inline uint16_t u16(const uint8_t *b){ return (uint16_t)b[0] | ((uint16_t)b[1] << 8); }
static inline int16_t  s16(const uint8_t *b){ return (int16_t)u16(b); }
static inline uint32_t u24(const uint8_t *b){ return (uint32_t)b[0] | ((uint32_t)b[1]<<8) | ((uint32_t)b[2]<<16); }

bool readCalib() {
  uint8_t b[21];
  if (!i2cReadN(BMP_ADDR, REG_CALIB0, b, sizeof(b))) return false;

  // Raw calib words as per BMP388 datasheet, then scaled.
  // Scaling factors from BMP388 datasheet compensation formula.
  uint16_t nT1 = u16(&b[0]);
  uint16_t nT2 = u16(&b[2]);
  int8_t   nT3 = (int8_t)b[4];

  int16_t  nP1 = s16(&b[5]);
  int16_t  nP2 = s16(&b[7]);
  int8_t   nP3 = (int8_t)b[9];
  int8_t   nP4 = (int8_t)b[10];
  uint16_t nP5 = u16(&b[11]);
  uint16_t nP6 = u16(&b[13]);
  int8_t   nP7 = (int8_t)b[15];
  int8_t   nP8 = (int8_t)b[16];
  int16_t  nP9 = s16(&b[17]);
  int8_t   nP10= (int8_t)b[19];
  int8_t   nP11= (int8_t)b[20];

  cal.T1  = (float)nT1 / 0.00390625f;           // 2^-8
  cal.T2  = (float)nT2 / 1073741824.0f;         // 2^30
  cal.T3  = (float)nT3 / 281474976710656.0f;    // 2^48

  cal.P1  = ((float)nP1 - 16384.0f) / 1048576.0f;    // 2^20
  cal.P2  = ((float)nP2 - 16384.0f) / 536870912.0f;  // 2^29
  cal.P3  = (float)nP3 / 4294967296.0f;              // 2^32
  cal.P4  = (float)nP4 / 137438953472.0f;            // 2^37
  cal.P5  = (float)nP5 / 0.125f;                     // 2^-3
  cal.P6  = (float)nP6 / 64.0f;                      // 2^6
  cal.P7  = (float)nP7 / 256.0f;                     // 2^8
  cal.P8  = (float)nP8 / 32768.0f;                   // 2^15
  cal.P9  = (float)nP9 / 281474976710656.0f;         // 2^48
  cal.P10 = (float)nP10 / 281474976710656.0f;        // 2^48
  cal.P11 = (float)nP11 / 36893488147419103232.0f;   // 2^65

  return true;
}

// Returns temperature in degC and pressure in Pa
bool readTempPress(float &tempC, float &pressPa) {
  uint8_t b[6];
  if (!i2cReadN(BMP_ADDR, REG_PRESS_XLSB, b, sizeof(b))) return false;

  uint32_t adcP = u24(&b[0]);
  uint32_t adcT = u24(&b[3]);

  // Compensation per BMP388 datasheet (floating-point form)
  float partial_data1 = (float)adcT - cal.T1;
  float partial_data2 = partial_data1 * cal.T2;
  float t_lin = partial_data2 + (partial_data1 * partial_data1) * cal.T3;
  tempC = t_lin;

  float pd1 = cal.P6 * t_lin;
  float pd2 = cal.P7 * t_lin * t_lin;
  float pd3 = cal.P8 * t_lin * t_lin * t_lin;
  float out1 = cal.P5 + pd1 + pd2 + pd3;

  float pd4 = cal.P2 * t_lin;
  float pd5 = cal.P3 * t_lin * t_lin;
  float pd6 = cal.P4 * t_lin * t_lin * t_lin;
  float out2 = (float)adcP * (cal.P1 + pd4 + pd5 + pd6);

  float pd7 = (float)adcP * (float)adcP;
  float pd8 = cal.P9 * pd7;
  float pd9 = cal.P10 * pd7 * t_lin;
  float pd10= cal.P11 * pd7 * (float)adcP;
  float out3 = pd8 + pd9 + pd10;

  pressPa = out1 + out2 + out3;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  Serial.println("\nBMP388 I2C test @0x76");

  uint8_t id=0;
  if (!i2cRead8(BMP_ADDR, REG_CHIP_ID, id)) {
    Serial.println("No response at 0x76.");
    return;
  }
  Serial.print("CHIP_ID = 0x"); if (id < 16) Serial.print("0"); Serial.println(id, HEX);

  uint8_t err=0, st=0;
  i2cRead8(BMP_ADDR, REG_ERR, err);
  i2cRead8(BMP_ADDR, REG_STATUS, st);
  Serial.print("ERR=0x"); if (err < 16) Serial.print("0"); Serial.println(err, HEX);
  Serial.print("STATUS=0x"); if (st < 16) Serial.print("0"); Serial.println(st, HEX);

  // Configure: normal mode, enable temp+pressure
  // PWR_CTRL: bit0=press_en, bit1=temp_en, bits5..4=mode (00 sleep, 01 forced, 11 normal)
  // We'll do: press+temp enable, normal mode -> 0b00110011 = 0x33
  i2cWrite8(BMP_ADDR, REG_PWR_CTRL, 0x33);

  // Oversampling: x4 temp, x8 press (reasonable)
  // OSR: temp_osr[5:3], press_osr[2:0]
  i2cWrite8(BMP_ADDR, REG_OSR, (2 << 3) | 3); // temp x4, press x8

  // ODR: 50 Hz approx (0x03 = 50Hz in BMP388)
  i2cWrite8(BMP_ADDR, REG_ODR, 0x03);

  // IIR filter config: enable, coeff 3 (moderate)
  // CONFIG: iir_filter[3:1]
  i2cWrite8(BMP_ADDR, REG_CONFIG, (3 << 1));

  if (!readCalib()) {
    Serial.println("Failed to read calibration.");
    return;
  }

  Serial.println("t_C p_Pa p_hPa");
}

void loop() {
  float tC=0, pPa=0;
  if (readTempPress(tC, pPa)) {
    Serial.print(tC, 2); Serial.print(' ');
    Serial.print(pPa, 2); Serial.print(' ');
    Serial.println(pPa / 100.0f, 2);
  } else {
    Serial.println("read fail");
  }
  delay(100);
}
