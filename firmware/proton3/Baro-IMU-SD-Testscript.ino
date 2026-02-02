#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

// ================== PINS ==================
static const int I2C_SDA = 8;
static const int I2C_SCL = 9;

static const int SPI_SCK  = 12;
static const int SPI_MOSI = 11;
static const int SPI_MISO = 13;

static const int SD_CS = 14;
static const int SD_CD = 21;
static const bool CD_ACTIVE_LOW = true;

// BMI088 CS
static const int CS_ACC  = 47;
static const int CS_GYRO = 38;

// ================== SD ==================
File logFile;
static const char* LOG_PATH = "/bmp388_log.csv";

bool cardPresent() {
  pinMode(SD_CD, INPUT_PULLUP);
  return CD_ACTIVE_LOW ? (digitalRead(SD_CD) == LOW) : (digitalRead(SD_CD) == HIGH);
}

bool initSD() {
  // alle CS HIGH gegen Buskampf
  pinMode(SD_CS, OUTPUT);   digitalWrite(SD_CS, HIGH);
  pinMode(CS_ACC, OUTPUT);  digitalWrite(CS_ACC, HIGH);
  pinMode(CS_GYRO, OUTPUT); digitalWrite(CS_GYRO, HIGH);

  SPI.end();
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);
  delay(5);

  // SD SPI wake: CS HIGH + >=74 clocks
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  digitalWrite(SD_CS, HIGH);
  for (int i=0;i<10;i++) SPI.transfer(0xFF);
  SPI.endTransaction();
  delay(2);

  if (!SD.begin(SD_CS, SPI, 400000)) {
    if (!SD.begin(SD_CS, SPI, 100000)) return false;
  }
  return true;
}

// ================== BMP388 (I2C) ==================
static const uint8_t BMP_ADDR = 0x76;

static const uint8_t REG_CHIP_ID   = 0x00; // BMP388: 0x50
static const uint8_t REG_ERR       = 0x02;
static const uint8_t REG_STATUS    = 0x03;
static const uint8_t REG_PRESS_XLSB= 0x04; // 0x04..0x06
static const uint8_t REG_TEMP_XLSB = 0x07; // 0x07..0x09
static const uint8_t REG_PWR_CTRL  = 0x1B;
static const uint8_t REG_OSR       = 0x1C;
static const uint8_t REG_ODR       = 0x1D;
static const uint8_t REG_CONFIG    = 0x1F;
static const uint8_t REG_CALIB0    = 0x31; // 0x31..0x45

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

struct Calib {
  float T1, T2, T3;
  float P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;
} cal;

static inline uint16_t u16(const uint8_t *b){ return (uint16_t)b[0] | ((uint16_t)b[1] << 8); }
static inline int16_t  s16(const uint8_t *b){ return (int16_t)u16(b); }
static inline uint32_t u24(const uint8_t *b){ return (uint32_t)b[0] | ((uint32_t)b[1]<<8) | ((uint32_t)b[2]<<16); }

bool bmpReadCalib() {
  uint8_t b[21];
  if (!i2cReadN(BMP_ADDR, REG_CALIB0, b, sizeof(b))) return false;

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

  cal.P1  = ((float)nP1 - 16384.0f) / 1048576.0f;
  cal.P2  = ((float)nP2 - 16384.0f) / 536870912.0f;
  cal.P3  = (float)nP3 / 4294967296.0f;
  cal.P4  = (float)nP4 / 137438953472.0f;
  cal.P5  = (float)nP5 / 0.125f;
  cal.P6  = (float)nP6 / 64.0f;
  cal.P7  = (float)nP7 / 256.0f;
  cal.P8  = (float)nP8 / 32768.0f;
  cal.P9  = (float)nP9 / 281474976710656.0f;
  cal.P10 = (float)nP10 / 281474976710656.0f;
  cal.P11 = (float)nP11 / 36893488147419103232.0f;

  return true;
}

bool bmpReadTempPress(float &tempC, float &pressPa) {
  uint8_t b[6];
  if (!i2cReadN(BMP_ADDR, REG_PRESS_XLSB, b, sizeof(b))) return false;
  uint32_t adcP = u24(&b[0]);
  uint32_t adcT = u24(&b[3]);

  float pd1 = (float)adcT - cal.T1;
  float pd2 = pd1 * cal.T2;
  float t_lin = pd2 + (pd1 * pd1) * cal.T3;
  tempC = t_lin;

  float po1 = cal.P5 + cal.P6 * t_lin + cal.P7 * t_lin * t_lin + cal.P8 * t_lin * t_lin * t_lin;
  float po2 = (float)adcP * (cal.P1 + cal.P2 * t_lin + cal.P3 * t_lin * t_lin + cal.P4 * t_lin * t_lin * t_lin);
  float pd7  = (float)adcP * (float)adcP;
  float po3  = cal.P9 * pd7 + cal.P10 * pd7 * t_lin + cal.P11 * pd7 * (float)adcP;

  pressPa = po1 + po2 + po3;
  return true;
}

bool initBMP388() {
  uint8_t id=0;
  if (!i2cRead8(BMP_ADDR, REG_CHIP_ID, id)) return false;
  if (id != 0x50) {
    Serial.print("BMP CHIP_ID unexpected: 0x"); Serial.println(id, HEX);
    // trotzdem weiter versuchen
  }

  // normal mode, temp+press enable
  i2cWrite8(BMP_ADDR, REG_PWR_CTRL, 0x33);
  // OSR: temp x4, press x8
  i2cWrite8(BMP_ADDR, REG_OSR, (2 << 3) | 3);
  // ODR ~50Hz
  i2cWrite8(BMP_ADDR, REG_ODR, 0x03);
  // IIR coeff 3
  i2cWrite8(BMP_ADDR, REG_CONFIG, (3 << 1));

  return bmpReadCalib();
}

// ================== BMI088 (SPI) ==================
static const uint32_t BMI_SPI_HZ = 1000000;

uint8_t bmiRead8(int cs, uint8_t reg) {
  SPI.beginTransaction(SPISettings(BMI_SPI_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(cs, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(0x00);               // dummy
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  return v;
}

void bmiWrite8(int cs, uint8_t reg, uint8_t val) {
  SPI.beginTransaction(SPISettings(BMI_SPI_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(cs, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

void bmiReadBurst(int cs, uint8_t startReg, uint8_t *buf, size_t n) {
  SPI.beginTransaction(SPISettings(BMI_SPI_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(cs, LOW);
  SPI.transfer(startReg | 0x80);
  SPI.transfer(0x00);               // dummy
  for (size_t i=0;i<n;i++) buf[i] = SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

static int16_t le16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

void initBMI088() {
  pinMode(CS_ACC, OUTPUT);  digitalWrite(CS_ACC, HIGH);
  pinMode(CS_GYRO, OUTPUT); digitalWrite(CS_GYRO, HIGH);

  // Accel enable: PWR_CTRL (0x7D) = 0x04
  bmiWrite8(CS_ACC, 0x7D, 0x04);
  delayMicroseconds(500);

  // Gyro normal (LPM1 0x11 = 0x00), Range/BW default ok für Anzeige
  bmiWrite8(CS_GYRO, 0x11, 0x00);
  delay(10);

  uint8_t acc_id  = bmiRead8(CS_ACC,  0x00);
  uint8_t gyro_id = bmiRead8(CS_GYRO, 0x00);

  Serial.print("BMI ACC_ID=0x");  Serial.println(acc_id, HEX);
  Serial.print("BMI GYRO_ID=0x"); Serial.println(gyro_id, HEX);
}

// ================== MAIN ==================
static uint32_t lastBmpMs = 0;
static uint32_t lastBmiMs = 0;

static const uint32_t BMP_PERIOD_MS = 100; // 10 Hz (anzeigen + loggen)
static const uint32_t BMI_PERIOD_MS = 20;  // 50 Hz (nur anzeigen)

static uint32_t logLineCount = 0;
static const uint32_t FLUSH_EVERY = 20;    // alle 20 Zeilen flushen (robust)

void setup() {
  Serial.begin(115200);
  delay(1200);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // SPI
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

  // BMP init
  if (!initBMP388()) {
    Serial.println("BMP init failed (0x76).");
    return;
  }
  uint8_t err=0, st=0;
  i2cRead8(BMP_ADDR, REG_ERR, err);
  i2cRead8(BMP_ADDR, REG_STATUS, st);
  Serial.print("BMP ERR=0x"); Serial.println(err, HEX);
  Serial.print("BMP STATUS=0x"); Serial.println(st, HEX);

  // BMI init (nur Anzeige)
  initBMI088();

  // SD init
  if (!cardPresent()) {
    Serial.println("No SD (CD).");
    return;
  }
  if (!initSD()) {
    Serial.println("SD init failed.");
    return;
  }
  Serial.println("SD init OK");

  logFile = SD.open(LOG_PATH, FILE_APPEND);
  if (!logFile) {
    Serial.println("open log failed");
    return;
  }
  if (logFile.size() == 0) {
    logFile.println("t_ms,temp_C,press_Pa,press_hPa");
    logFile.flush();
  }

  Serial.println("RUN");
}

void loop() {
  uint32_t now = millis();

  // ---------- BMI Anzeige ----------
  if (now - lastBmiMs >= BMI_PERIOD_MS) {
    lastBmiMs = now;

    uint8_t ab[6], gb[6];
    // Accel raw: 0x12..0x17
    bmiReadBurst(CS_ACC, 0x12, ab, 6);
    int16_t ax = le16(&ab[0]);
    int16_t ay = le16(&ab[2]);
    int16_t az = le16(&ab[4]);

    // Gyro raw: 0x02..0x07
    bmiReadBurst(CS_GYRO, 0x02, gb, 6);
    int16_t gx = le16(&gb[0]);
    int16_t gy = le16(&gb[2]);
    int16_t gz = le16(&gb[4]);

    Serial.print("BMI ");
    Serial.print(ax); Serial.print(' ');
    Serial.print(ay); Serial.print(' ');
    Serial.print(az); Serial.print(" | ");
    Serial.print(gx); Serial.print(' ');
    Serial.print(gy); Serial.print(' ');
    Serial.println(gz);
  }

  // ---------- BMP Anzeige + SD Log ----------
  if (now - lastBmpMs >= BMP_PERIOD_MS) {
    lastBmpMs = now;

    float tC=0, pPa=0;
    if (!bmpReadTempPress(tC, pPa)) {
      Serial.println("BMP read fail");
      return;
    }
    float phPa = pPa / 100.0f;

    Serial.print("BMP ");
    Serial.print(now);
    Serial.print(" T=");
    Serial.print(tC, 2);
    Serial.print("C P=");
    Serial.print(phPa, 2);
    Serial.println("hPa");

    if (logFile) {
      logFile.print(now);
      logFile.print(',');
      logFile.print(tC, 2);
      logFile.print(',');
      logFile.print(pPa, 2);
      logFile.print(',');
      logFile.println(phPa, 2);

      logLineCount++;
      if (logLineCount % FLUSH_EVERY == 0) logFile.flush();
    }
  }
}
