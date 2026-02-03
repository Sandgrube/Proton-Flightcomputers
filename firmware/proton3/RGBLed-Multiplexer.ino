#include <Wire.h>

static constexpr int SDA_PIN = 8;
static constexpr int SCL_PIN = 9;
static constexpr uint32_t I2C_FREQ = 100000;

static constexpr uint8_t TCA_ADDR = 0x70;
static constexpr uint8_t PCA_ADDR = 0x40;

static constexpr uint8_t MUX_CH = 4;

// PCA OE# hängt bei dir an GPIO15 (aktiv LOW!)
static constexpr int OE_PIN = 15;

static constexpr uint8_t CH_R = 4;
static constexpr uint8_t CH_G = 5;
static constexpr uint8_t CH_B = 6;

static constexpr bool COMMON_ANODE = false;

// PCA9685 regs
static constexpr uint8_t MODE1     = 0x00;
static constexpr uint8_t MODE2     = 0x01;
static constexpr uint8_t LED0_ON_L = 0x06;
static constexpr uint8_t PRESCALE  = 0xFE;

static void printHex2(uint8_t v){ if(v<16) Serial.print('0'); Serial.print(v,HEX); }

bool tcaSelect(uint8_t ch){
  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << ch);
  return Wire.endTransmission() == 0;
}

bool write8(uint8_t addr, uint8_t reg, uint8_t val){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

uint8_t read8(uint8_t addr, uint8_t reg){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  if (Wire.requestFrom((int)addr, 1) != 1) return 0xFF;
  return Wire.read();
}

bool setPWM(uint8_t ch, uint16_t on, uint16_t off){
  uint8_t base = LED0_ON_L + 4 * ch;
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(base);
  Wire.write(on & 0xFF);
  Wire.write(on >> 8);
  Wire.write(off & 0xFF);
  Wire.write(off >> 8);
  return Wire.endTransmission() == 0;
}

void setPWM12(uint8_t ch, uint16_t val){
  if(val > 4095) val = 4095;
  setPWM(ch, 0, val);
}

void setRGB_u8(uint8_t r, uint8_t g, uint8_t b){
  auto map12 = [](uint8_t v)->uint16_t { return (uint16_t)((uint32_t)v * 4095u / 255u); };
  uint16_t pr = map12(r), pg = map12(g), pb = map12(b);
  if (COMMON_ANODE) { pr = 4095 - pr; pg = 4095 - pg; pb = 4095 - pb; }
  setPWM12(CH_R, pr);
  setPWM12(CH_G, pg);
  setPWM12(CH_B, pb);
}

void setFreq(float hz){
  if(hz < 1) hz = 1;
  if(hz > 3500) hz = 3500;

  float prescale_f = 25000000.0f / (4096.0f * hz) - 1.0f;
  uint8_t prescale = (uint8_t)(prescale_f + 0.5f);

  uint8_t oldmode = read8(PCA_ADDR, MODE1);
  uint8_t sleepmode = (oldmode & 0x7F) | 0x10; // SLEEP=1
  write8(PCA_ADDR, MODE1, sleepmode);
  delay(1);

  write8(PCA_ADDR, PRESCALE, prescale);

  uint8_t wake = (oldmode & ~0x10);            // SLEEP=0
  write8(PCA_ADDR, MODE1, wake);
  delay(1);

  write8(PCA_ADDR, MODE1, wake | 0x80);        // RESTART
  delay(1);
}

void setup(){
  Serial.begin(115200);

  // 1) OE# sofort aktivieren: LOW = Outputs enabled
  pinMode(OE_PIN, OUTPUT);
  digitalWrite(OE_PIN, LOW);

  delay(50);
  Serial.println("\nPCA9685 + TCA9548A test with OE# on GPIO15");

  Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQ);

  Serial.print("Select MUX ch "); Serial.print(MUX_CH); Serial.print(" ... ");
  if(!tcaSelect(MUX_CH)){ Serial.println("FAIL"); while(true) delay(1000); }
  Serial.println("OK");

  uint8_t m1 = read8(PCA_ADDR, MODE1);
  Serial.print("PCA MODE1 before: 0x"); printHex2(m1); Serial.println();

  // MODE2 OUTDRV=1
  write8(PCA_ADDR, MODE2, 0x04);

  // MODE1: AI=1, ALLCALL=1, SLEEP=0
  write8(PCA_ADDR, MODE1, 0x21);
  delay(2);

  setFreq(1000);

  uint8_t m1a = read8(PCA_ADDR, MODE1);
  Serial.print("PCA MODE1 after : 0x"); printHex2(m1a); Serial.println();

  setRGB_u8(0,0,0);
  Serial.println("Ready.");
}

void loop(){
  // OE# safety (falls irgendwer es verstellt)
  digitalWrite(OE_PIN, LOW);

  setRGB_u8(255,0,0); delay(500);
  setRGB_u8(0,255,0); delay(500);
  setRGB_u8(0,0,255); delay(500);
  setRGB_u8(0,0,0);   delay(500);
}
