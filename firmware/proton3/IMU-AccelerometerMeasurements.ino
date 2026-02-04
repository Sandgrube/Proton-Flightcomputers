#include <Arduino.h>
#include <SPI.h>

static const int PIN_SCK  = 12;
static const int PIN_MOSI = 11;
static const int PIN_MISO = 13;

static const int CS_ACC   = 47;
static const int CS_GYRO  = 38;

static const uint32_t SPI_HZ = 1000000;

uint8_t r8(int cs, uint8_t reg){
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(cs, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(0x00);
  uint8_t v = SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
  return v;
}

void w8(int cs, uint8_t reg, uint8_t val){
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(cs, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

void rb(int cs, uint8_t startReg, uint8_t *buf, size_t n){
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE3));
  digitalWrite(cs, LOW);
  SPI.transfer(startReg | 0x80);
  SPI.transfer(0x00);
  for (size_t i=0;i<n;i++) buf[i]=SPI.transfer(0x00);
  digitalWrite(cs, HIGH);
  SPI.endTransaction();
}

static int16_t le16(const uint8_t *p){ return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1]<<8)); }

void setup(){
  Serial.begin(115200);
  delay(1200);

  pinMode(CS_ACC, OUTPUT);
  pinMode(CS_GYRO, OUTPUT);
  digitalWrite(CS_ACC, HIGH);
  digitalWrite(CS_GYRO, HIGH);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  // Accel einschalten (korrekt)
  w8(CS_ACC, 0x7D, 0x04); // ACC_PWR_CTRL enable
  delayMicroseconds(500);

  Serial.print("ACC_ID=0x"); Serial.println(r8(CS_ACC, 0x00), HEX);
  Serial.print("ACC_RANGE=0x"); Serial.println(r8(CS_ACC, 0x41), HEX);
  Serial.print("ACC_CONF=0x"); Serial.println(r8(CS_ACC, 0x40), HEX);
}

void loop(){
  uint8_t st = r8(CS_ACC, 0x03); // ACC_STATUS
  uint8_t ab[6];
  rb(CS_ACC, 0x12, ab, 6);
  int16_t ax = le16(&ab[0]);
  int16_t ay = le16(&ab[2]);
  int16_t az = le16(&ab[4]);

  // Betrag grob (ohne float)
  int32_t mag2 = (int32_t)ax*ax + (int32_t)ay*ay + (int32_t)az*az;

  Serial.print("st=0x"); if(st<16) Serial.print("0"); Serial.print(st, HEX);
  Serial.print(" ax="); Serial.print(ax);
  Serial.print(" ay="); Serial.print(ay);
  Serial.print(" az="); Serial.print(az);
  Serial.print(" | mag2="); Serial.println(mag2);

  delay(100);
}
