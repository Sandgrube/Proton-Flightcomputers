#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>

// ============================================================================
// 1) TYPES FIRST
// ============================================================================
struct PID {
  float kp, ki, kd;
  float i;
  float prev_e;
};

struct Calib {
  float ax_off, ay_off, az_off;   // g
  float gx_bias, gy_bias, gz_bias; // rad/s
  float T0;
};

struct BMP3Cal {
  float par_t1, par_t2, par_t3;
  float par_p1, par_p2, par_p3, par_p4, par_p5, par_p6, par_p7, par_p8, par_p9, par_p10, par_p11;
};

// ============================================================================
// 2) HELPERS
// ============================================================================
static inline float clampf(float x, float a, float b){
  return (x < a) ? a : (x > b) ? b : x;
}
static inline float wrap_pi(float a){
  while (a >  PI) a -= 2.0f*PI;
  while (a < -PI) a += 2.0f*PI;
  return a;
}
static inline int16_t le16(const uint8_t *p){
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static inline int32_t le24u(const uint8_t *p){
  return (int32_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16));
}

// ============================================================================
// 3) PINS / CONSTANTS
// ============================================================================
static const int PIN_SCK  = 12;
static const int PIN_MOSI = 11;
static const int PIN_MISO = 13;

static const int CS_ACC   = 47;
static const int CS_GYRO  = 38;

static const int PIN_SDA  = 8;
static const int PIN_SCL  = 9;

static const uint32_t SPI_HZ = 1000000;
static const uint32_t I2C_HZ = 400000;

static const float G0 = 9.80665f;

// If your gyro read is still broken, flip this.
static const bool  GYRO_NEEDS_DUMMY = false;

// Complementary filter
static const float TAU = 0.7f;        // s
static const float AMAG_MIN = 0.92f;
static const float AMAG_MAX = 1.08f;

// sanity
static const float MAX_RATE = 15.0f;  // rad/s
static const float DT_MAX   = 0.02f;
static const float DT_MIN   = 0.001f;

// visual smoothing
static const float OUT_ALPHA = 0.08f;
static const float DEADBAND  = 0.5f * (PI/180.0f);

// altitude smoothing
static const float ALT_ALPHA = 0.05f;
static const float VZ_ALPHA  = 0.12f;

// ============================================================================
// 4) REGISTERS
// ============================================================================
// BMI088 ACC
static const uint8_t REG_ACC_CHIP_ID   = 0x00;
static const uint8_t REG_ACC_X_LSB     = 0x12;
static const uint8_t REG_ACC_CONF      = 0x40;
static const uint8_t REG_ACC_RANGE     = 0x41;
static const uint8_t REG_ACC_PWR_CONF  = 0x7C;
static const uint8_t REG_ACC_PWR_CTRL  = 0x7D;
static const uint8_t REG_ACC_SOFTRESET = 0x7E;

// BMI088 GYRO
static const uint8_t REG_GYR_CHIP_ID   = 0x00;
static const uint8_t REG_GYR_X_LSB     = 0x02;
static const uint8_t REG_GYR_RANGE     = 0x0F;
static const uint8_t REG_GYR_BW        = 0x10;
static const uint8_t REG_GYR_LPM1      = 0x11;
static const uint8_t REG_GYR_SOFTRESET = 0x14;

// BMP388
static const uint8_t BMP_ADDR_1 = 0x76;
static const uint8_t BMP_ADDR_2 = 0x77;

static const uint8_t BMP_REG_CHIP_ID  = 0x00;
static const uint8_t BMP_REG_DATA     = 0x04;
static const uint8_t BMP_REG_PWR_CTRL = 0x1B;
static const uint8_t BMP_REG_OSR      = 0x1C;
static const uint8_t BMP_REG_ODR      = 0x1D;
static const uint8_t BMP_REG_CFG      = 0x1F;
static const uint8_t BMP_REG_CALIB    = 0x31;

// ============================================================================
// 5) PROTOTYPES (NO ARDUINO AUTOPROTO SURPRISES)
// ============================================================================
static inline void spi_begin();
static inline void spi_end();

void spi_rb_acc(uint8_t startReg, uint8_t *buf, size_t n);
uint8_t spi_r8_acc(uint8_t reg);
void spi_w8_acc(uint8_t reg, uint8_t val);

void spi_rb_gyr(uint8_t startReg, uint8_t *buf, size_t n);
uint8_t spi_r8_gyr(uint8_t reg);
void spi_w8_gyr(uint8_t reg, uint8_t val);

bool i2c_write8(uint8_t addr, uint8_t reg, uint8_t val);
bool i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n);

bool bmi_acc_init();
bool bmi_gyr_init();
bool read_acc_g(float &ax_g, float &ay_g, float &az_g);
bool read_gyr_rads(float &gx, float &gy, float &gz);

bool bmp_init();
bool bmp_detect();
bool bmp_read_calib();
bool bmp_read_PT(float &temp_C, float &press_Pa);
float bmp_comp_temp(int32_t adc_t);
float bmp_comp_press(int32_t adc_p);

void calibrate_stationary();

float pid_update(PID &c, float e, float dt);
float alt_from_press(float p);

void attitude_step(float gx, float gy, float gz,
                   float ax_g, float ay_g, float az_g,
                   float dt);

// ============================================================================
// 6) GLOBAL STATE
// ============================================================================
static float acc_lsb_per_g   = 32768.0f / 6.0f;
static float gyr_lsb_per_dps = 16.4f;

static uint8_t bmp_addr = 0;
static BMP3Cal bmpCal;
static float bmp_t_lin = 0.0f;
static float p0_ref = 101325.0f;

static Calib cal = {0};

static float roll=0, pitch=0, yaw=0;
static float roll_vis=0, pitch_vis=0, yaw_vis=0;

static float alt_lp=0, vz_lp=0;
static float alt_prev=0;

static PID pid_roll  = {1.6f, 0.05f, 0.04f, 0.0f, 0.0f};
static PID pid_pitch = {1.6f, 0.05f, 0.04f, 0.0f, 0.0f};

static uint32_t t_prev_us = 0;

// ============================================================================
// 7) SPI
// ============================================================================
static inline void spi_begin(){
  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE3));
}
static inline void spi_end(){
  SPI.endTransaction();
}

// ACC: dummy required
void spi_rb_acc(uint8_t startReg, uint8_t *buf, size_t n){
  spi_begin();
  digitalWrite(CS_ACC, LOW);
  SPI.transfer(startReg | 0x80);
  SPI.transfer(0x00);
  for (size_t i=0;i<n;i++) buf[i] = SPI.transfer(0x00);
  digitalWrite(CS_ACC, HIGH);
  spi_end();
}

uint8_t spi_r8_acc(uint8_t reg){
  uint8_t v=0;
  spi_begin();
  digitalWrite(CS_ACC, LOW);
  SPI.transfer(reg | 0x80);
  SPI.transfer(0x00);
  v = SPI.transfer(0x00);
  digitalWrite(CS_ACC, HIGH);
  spi_end();
  return v;
}

void spi_w8_acc(uint8_t reg, uint8_t val){
  spi_begin();
  digitalWrite(CS_ACC, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(CS_ACC, HIGH);
  spi_end();
}

// GYRO: dummy optional
void spi_rb_gyr(uint8_t startReg, uint8_t *buf, size_t n){
  spi_begin();
  digitalWrite(CS_GYRO, LOW);
  SPI.transfer(startReg | 0x80);
  if (GYRO_NEEDS_DUMMY) SPI.transfer(0x00);
  for (size_t i=0;i<n;i++) buf[i] = SPI.transfer(0x00);
  digitalWrite(CS_GYRO, HIGH);
  spi_end();
}

uint8_t spi_r8_gyr(uint8_t reg){
  uint8_t v=0;
  spi_begin();
  digitalWrite(CS_GYRO, LOW);
  SPI.transfer(reg | 0x80);
  if (GYRO_NEEDS_DUMMY) SPI.transfer(0x00);
  v = SPI.transfer(0x00);
  digitalWrite(CS_GYRO, HIGH);
  spi_end();
  return v;
}

void spi_w8_gyr(uint8_t reg, uint8_t val){
  spi_begin();
  digitalWrite(CS_GYRO, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(val);
  digitalWrite(CS_GYRO, HIGH);
  spi_end();
}

// ============================================================================
// 8) I2C
// ============================================================================
bool i2c_write8(uint8_t addr, uint8_t reg, uint8_t val){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

bool i2c_read(uint8_t addr, uint8_t reg, uint8_t *buf, size_t n){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t got = Wire.requestFrom((int)addr, (int)n);
  if (got != n) return false;
  for (size_t i=0;i<n;i++) buf[i] = Wire.read();
  return true;
}

// ============================================================================
// 9) BMI088
// ============================================================================
bool bmi_acc_init(){
  spi_w8_acc(REG_ACC_SOFTRESET, 0xB6);
  delay(50);

  spi_w8_acc(REG_ACC_PWR_CONF, 0x00);
  delay(10);

  spi_w8_acc(REG_ACC_PWR_CTRL, 0x04);
  delay(10);

  spi_w8_acc(REG_ACC_RANGE, 0x01); // ±6g
  delay(2);

  spi_w8_acc(REG_ACC_CONF, 0xA8);
  delay(2);

  uint8_t range = spi_r8_acc(REG_ACC_RANGE);
  float g_range = 6.0f;
  if      (range == 0x00) g_range = 3.0f;
  else if (range == 0x01) g_range = 6.0f;
  else if (range == 0x02) g_range = 12.0f;
  else if (range == 0x03) g_range = 24.0f;
  acc_lsb_per_g = 32768.0f / g_range;
  return true;
}

bool bmi_gyr_init(){
  spi_w8_gyr(REG_GYR_SOFTRESET, 0xB6);
  delay(50);

  spi_w8_gyr(REG_GYR_LPM1, 0x00);
  delay(2);

  spi_w8_gyr(REG_GYR_RANGE, 0x00); // 2000 dps
  delay(2);

  spi_w8_gyr(REG_GYR_BW, 0x07);
  delay(2);

  uint8_t range = spi_r8_gyr(REG_GYR_RANGE);
  if      (range == 0x00) gyr_lsb_per_dps = 16.4f;
  else if (range == 0x01) gyr_lsb_per_dps = 32.8f;
  else if (range == 0x02) gyr_lsb_per_dps = 65.6f;
  else if (range == 0x03) gyr_lsb_per_dps = 131.2f;
  else if (range == 0x04) gyr_lsb_per_dps = 262.4f;
  return true;
}

bool read_acc_g(float &ax_g, float &ay_g, float &az_g){
  uint8_t b[6];
  spi_rb_acc(REG_ACC_X_LSB, b, 6);
  int16_t ax = le16(&b[0]);
  int16_t ay = le16(&b[2]);
  int16_t az = le16(&b[4]);
  ax_g = (float)ax / acc_lsb_per_g;
  ay_g = (float)ay / acc_lsb_per_g;
  az_g = (float)az / acc_lsb_per_g;
  return true;
}

bool read_gyr_rads(float &gx, float &gy, float &gz){
  uint8_t b[6];
  spi_rb_gyr(REG_GYR_X_LSB, b, 6);

  int16_t rx = le16(&b[0]);
  int16_t ry = le16(&b[2]);
  int16_t rz = le16(&b[4]);

  float gx_dps = (float)rx / gyr_lsb_per_dps;
  float gy_dps = (float)ry / gyr_lsb_per_dps;
  float gz_dps = (float)rz / gyr_lsb_per_dps;

  gx = gx_dps * (PI / 180.0f);
  gy = gy_dps * (PI / 180.0f);
  gz = gz_dps * (PI / 180.0f);

  if (fabsf(gx) > MAX_RATE) gx = 0;
  if (fabsf(gy) > MAX_RATE) gy = 0;
  if (fabsf(gz) > MAX_RATE) gz = 0;

  return true;
}

// ============================================================================
// 10) BMP388
// ============================================================================
bool bmp_detect(){
  uint8_t id=0;
  if (i2c_read(BMP_ADDR_1, BMP_REG_CHIP_ID, &id, 1)) { bmp_addr = BMP_ADDR_1; return true; }
  if (i2c_read(BMP_ADDR_2, BMP_REG_CHIP_ID, &id, 1)) { bmp_addr = BMP_ADDR_2; return true; }
  return false;
}

bool bmp_read_calib(){
  uint8_t c[21];
  if (!i2c_read(bmp_addr, BMP_REG_CALIB, c, 21)) return false;

  uint16_t T1 = (uint16_t)c[0] | ((uint16_t)c[1] << 8);
  uint16_t T2 = (uint16_t)c[2] | ((uint16_t)c[3] << 8);
  int8_t   T3 = (int8_t)c[4];

  int16_t  P1 = (int16_t)((uint16_t)c[5]  | ((uint16_t)c[6]  << 8));
  int16_t  P2 = (int16_t)((uint16_t)c[7]  | ((uint16_t)c[8]  << 8));
  int8_t   P3 = (int8_t)c[9];
  int8_t   P4 = (int8_t)c[10];
  uint16_t P5 = (uint16_t)c[11] | ((uint16_t)c[12] << 8);
  uint16_t P6 = (uint16_t)c[13] | ((uint16_t)c[14] << 8);
  int8_t   P7 = (int8_t)c[15];
  int8_t   P8 = (int8_t)c[16];
  int16_t  P9 = (int16_t)((uint16_t)c[17] | ((uint16_t)c[18] << 8));
  int8_t   P10= (int8_t)c[19];
  int8_t   P11= (int8_t)c[20];

  bmpCal.par_t1  = (float)T1 / 0.00390625f;
  bmpCal.par_t2  = (float)T2 / 1073741824.0f;
  bmpCal.par_t3  = (float)T3 / 281474976710656.0f;

  bmpCal.par_p1  = ((float)P1 - 16384.0f) / 1048576.0f;
  bmpCal.par_p2  = ((float)P2 - 16384.0f) / 536870912.0f;
  bmpCal.par_p3  = (float)P3 / 4294967296.0f;
  bmpCal.par_p4  = (float)P4 / 137438953472.0f;
  bmpCal.par_p5  = (float)P5 / 0.125f;
  bmpCal.par_p6  = (float)P6 / 64.0f;
  bmpCal.par_p7  = (float)P7 / 256.0f;
  bmpCal.par_p8  = (float)P8 / 32768.0f;
  bmpCal.par_p9  = (float)P9 / 281474976710656.0f;
  bmpCal.par_p10 = (float)P10 / 281474976710656.0f;
  bmpCal.par_p11 = (float)P11 / 36893488147419103232.0f;

  return true;
}

float bmp_comp_temp(int32_t adc_t){
  float partial1 = (float)adc_t - bmpCal.par_t1;
  float partial2 = partial1 * bmpCal.par_t2;
  bmp_t_lin = partial2 + (partial1 * partial1) * bmpCal.par_t3;
  return bmp_t_lin;
}

float bmp_comp_press(int32_t adc_p){
  float partial1 = bmpCal.par_p6 * bmp_t_lin;
  float partial2 = bmpCal.par_p7 * (bmp_t_lin * bmp_t_lin);
  float partial3 = bmpCal.par_p8 * (bmp_t_lin * bmp_t_lin * bmp_t_lin);
  float partial_out1 = bmpCal.par_p5 + partial1 + partial2 + partial3;

  float partial4 = bmpCal.par_p2 * bmp_t_lin;
  float partial5 = bmpCal.par_p3 * (bmp_t_lin * bmp_t_lin);
  float partial6 = bmpCal.par_p4 * (bmp_t_lin * bmp_t_lin * bmp_t_lin);
  float partial_out2 = (float)adc_p * (bmpCal.par_p1 + partial4 + partial5 + partial6);

  float partial7 = (float)adc_p * (float)adc_p;
  float partial8 = bmpCal.par_p9 + bmpCal.par_p10 * bmp_t_lin;
  float partial9 = partial7 * partial8;
  float partial10 = partial9 + (float)adc_p * partial7 * bmpCal.par_p11;

  return partial_out1 + partial_out2 + partial10;
}

bool bmp_init(){
  if (!bmp_detect()) return false;
  if (!bmp_read_calib()) return false;

  i2c_write8(bmp_addr, BMP_REG_OSR, (0b011) | (0b001 << 3));
  i2c_write8(bmp_addr, BMP_REG_ODR, 0x05);
  i2c_write8(bmp_addr, BMP_REG_CFG, 0x04);
  i2c_write8(bmp_addr, BMP_REG_PWR_CTRL, 0x33);
  delay(10);
  return true;
}

bool bmp_read_PT(float &temp_C, float &press_Pa){
  uint8_t d[6];
  if (!i2c_read(bmp_addr, BMP_REG_DATA, d, 6)) return false;
  int32_t adc_p = le24u(&d[0]);
  int32_t adc_t = le24u(&d[3]);
  temp_C = bmp_comp_temp(adc_t);
  press_Pa = bmp_comp_press(adc_p);
  return true;
}

// ============================================================================
// 11) Calibration + filters
// ============================================================================
void calibrate_stationary(){
  Serial.println("CAL: hold still for ~4s...");

  const uint32_t N=800;
  float sum_ax=0,sum_ay=0,sum_az=0;
  float sum_gx=0,sum_gy=0,sum_gz=0;
  float sum_T=0,sum_p=0;
  uint32_t good=0;

  for(uint32_t i=0;i<N;i++){
    float ax,ay,az,gx,gy,gz,T,p;
    read_acc_g(ax,ay,az);
    read_gyr_rads(gx,gy,gz);
    if (!bmp_read_PT(T,p)) { delay(5); continue; }
    sum_ax+=ax; sum_ay+=ay; sum_az+=az;
    sum_gx+=gx; sum_gy+=gy; sum_gz+=gz;
    sum_T+=T; sum_p+=p;
    good++;
    delay(5);
  }

  float axm=sum_ax/good, aym=sum_ay/good, azm=sum_az/good;
  float gxm=sum_gx/good, gym=sum_gy/good, gzm=sum_gz/good;
  float Tm=sum_T/good, pm=sum_p/good;

  cal.ax_off=axm;
  cal.ay_off=aym;
  cal.az_off=azm - 1.0f;

  cal.gx_bias=gxm;
  cal.gy_bias=gym;
  cal.gz_bias=gzm;

  cal.T0=Tm;
  p0_ref=pm;

  roll=pitch=yaw=0;
  roll_vis=pitch_vis=yaw_vis=0;

  alt_lp=0; alt_prev=0; vz_lp=0;

  Serial.println("CAL: done.");
}

float pid_update(PID &c, float e, float dt){
  c.i += e*dt;
  c.i = clampf(c.i, -0.25f, 0.25f);
  float de = (dt>0.0f) ? (e - c.prev_e)/dt : 0.0f;
  c.prev_e = e;
  return c.kp*e + c.ki*c.i + c.kd*de;
}

float alt_from_press(float p){
  return 44330.0f * (1.0f - powf(p / p0_ref, 0.19029495f));
}

void attitude_step(float gx, float gy, float gz,
                   float ax_g, float ay_g, float az_g,
                   float dt)
{
  gx -= cal.gx_bias;
  gy -= cal.gy_bias;
  gz -= cal.gz_bias;

  roll  = wrap_pi(roll  + gx*dt);
  pitch = wrap_pi(pitch + gy*dt);
  yaw   = wrap_pi(yaw   + gz*dt);

  float amag = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  if (amag < AMAG_MIN || amag > AMAG_MAX) return;

  float roll_a  = atan2f(ay_g, az_g);
  float pitch_a = atan2f(-ax_g, sqrtf(ay_g*ay_g + az_g*az_g));

  float alpha = TAU / (TAU + dt);
  roll  = wrap_pi(alpha*roll  + (1.0f-alpha)*roll_a);
  pitch = wrap_pi(alpha*pitch + (1.0f-alpha)*pitch_a);
}

// ============================================================================
// 12) Arduino
// ============================================================================
void setup(){
  Serial.begin(115200);
  delay(1200);

  pinMode(CS_ACC, OUTPUT);
  pinMode(CS_GYRO, OUTPUT);
  digitalWrite(CS_ACC, HIGH);
  digitalWrite(CS_GYRO, HIGH);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_HZ);

  Serial.println("\n--- Visualizer Stable ---");

  bmi_acc_init();
  bmi_gyr_init();
  bmp_init();

  Serial.print("ACC_ID=0x"); Serial.println(spi_r8_acc(REG_ACC_CHIP_ID), HEX);
  Serial.print("GYR_ID=0x"); Serial.println(spi_r8_gyr(REG_GYR_CHIP_ID), HEX);

  calibrate_stationary();
  t_prev_us = micros();

  Serial.println("t_ms,roll,pitch,yaw,gx,gy,gz,ax,ay,az,tempC,pressPa,alt_m,vz_mps,uR,uP");
  Serial.println("Start.");
}

void loop(){
  uint32_t now = micros();
  float dt = (now - t_prev_us) * 1e-6f;
  t_prev_us = now;
  dt = clampf(dt, DT_MIN, DT_MAX);

  float ax_g, ay_g, az_g;
  float gx, gy, gz;
  float T = cal.T0, pPa = p0_ref;

  read_acc_g(ax_g, ay_g, az_g);
  read_gyr_rads(gx, gy, gz);
  (void)bmp_read_PT(T, pPa);

  ax_g -= cal.ax_off;
  ay_g -= cal.ay_off;
  az_g -= cal.az_off;

  attitude_step(gx, gy, gz, ax_g, ay_g, az_g, dt);

  float eR = -roll;
  float eP = -pitch;
  float uR = clampf(pid_update(pid_roll,  eR, dt), -0.5f, 0.5f);
  float uP = clampf(pid_update(pid_pitch, eP, dt), -0.5f, 0.5f);

  float r = (fabsf(roll)  < DEADBAND) ? 0.0f : roll;
  float p = (fabsf(pitch) < DEADBAND) ? 0.0f : pitch;
  float y = (fabsf(yaw)   < DEADBAND) ? 0.0f : yaw;

  roll_vis  += OUT_ALPHA * (r - roll_vis);
  pitch_vis += OUT_ALPHA * (p - pitch_vis);
  yaw_vis   += OUT_ALPHA * (y - yaw_vis);

  float alt = alt_from_press(pPa);
  alt_lp += ALT_ALPHA * (alt - alt_lp);
  float vz = (alt_lp - alt_prev) / dt;
  alt_prev = alt_lp;
  vz_lp += VZ_ALPHA * (vz - vz_lp);

  uint32_t t_ms = millis();

  Serial.print(t_ms); Serial.print(",");
  Serial.print(roll_vis, 6); Serial.print(",");
  Serial.print(pitch_vis, 6); Serial.print(",");
  Serial.print(yaw_vis, 6); Serial.print(",");

  Serial.print(gx, 6); Serial.print(",");
  Serial.print(gy, 6); Serial.print(",");
  Serial.print(gz, 6); Serial.print(",");

  Serial.print(ax_g, 6); Serial.print(",");
  Serial.print(ay_g, 6); Serial.print(",");
  Serial.print(az_g, 6); Serial.print(",");

  Serial.print(T, 2); Serial.print(",");
  Serial.print(pPa, 1); Serial.print(",");

  Serial.print(alt_lp, 3); Serial.print(",");
  Serial.print(vz_lp, 3); Serial.print(",");

  Serial.print(uR, 6); Serial.print(",");
  Serial.println(uP, 6);

  delay(5);
}
