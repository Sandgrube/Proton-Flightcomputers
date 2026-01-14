// TRACER TVC Control Script mit Kalman-Filter, PID-Regelung, Auto-Inkrement SD-Logging @20Hz & GPS-Integration
// System: ESP32 + MPU6050 + optional NEO-6M + 2x Servos
// Profil: aggressiv, modular, anpassbar
//
// CSV-Header:
// millis,set_pitch,set_yaw,pitch,yaw,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,pwm_x,pwm_y,gps_fix,lat,lon,alt_m,speed_kmh,sats
//
// Hinweise:
// * Auto-Inkrement: /flight_0000.csv, /flight_0001.csv, ... nächster freier Index wird gewählt.
// * Schreibfrequenz: 20 Hz (alle 50 ms) -> Logging getaktet, Regelschleife läuft so schnell wie möglich.
// * Flush-Intervall: 250 ms (5 Samples) -> Balance zwischen Datensicherheit & Performance.
// * GPS_TX_PIN ist -1, damit kein Pin-Konflikt mit SERVO_Y_PIN=26. Wir empfangen nur (RX) vom GPS.
//   Wenn Du unbedingt senden musst, ändere SERVO_Y_PIN oder wähle anderen TX-Pin und setze GPS_TX_PIN entsprechend.
//
// TODO-Optionen (sag Bescheid, wenn Du sie willst):
// - Logging in Ringbuffer + Hintergrund-Task (geringere Jitter im Regelkreis)
// - Binäres Logging (kleiner, schneller)
// - Flight-Phasen-Markierungen (Arming, Ignition, Boost, Coast, Recovery)
// - EEPROM/FRAM Failover

#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <SD.h>

// ========================= SERVO-PARAMETER =========================
#define SERVO_X_PIN   25
#define SERVO_Y_PIN   26
#define SERVO_MIN_US  1000
#define SERVO_MAX_US  2000
#define SERVO_MID_US  1500
#define AGGRESSIVENESS 1.5f   // 1.0 = normal, >1.0 = aggressiv

// PID (aggressiv skaliert)
float kp = 5.0f * AGGRESSIVENESS;
float ki = 0.2f * AGGRESSIVENESS;
float kd = 0.8f * AGGRESSIVENESS;

// Kalman-Parameter
float q_angle   = 0.001f;
float q_bias    = 0.003f;
float r_measure = 0.03f;

// ========================= SD / SPI =========================
#define SD_CS_PIN   17
#define SD_MOSI_PIN 23
#define SD_CLK_PIN  18
#define SD_MISO_PIN 19
SPIClass spi = SPIClass(VSPI);
File logFile;
bool sd_ok = false;

// Auto-Inkrementiertes Logfile: /flight_XXXX.csv
// Max Index -> 9999 (änderbar)
#define LOGFILE_PREFIX "/flight_"
#define LOGFILE_EXT    ".csv"
#define LOGFILE_MAX_IDX 9999

// Logging-Takt
const uint32_t LOG_PERIOD_MS = 50;   // 20 Hz
const uint32_t LOG_FLUSH_INTERVAL_MS = 250; // ~5 Zeilen
uint32_t last_log_ms = 0;
uint32_t last_flush_ms = 0;

// ========================= GPS =========================
// UART2 (HardwareSerial(2))
#define GPS_RX_PIN 14   // vom GPS-TX kommend -> ESP32 RX
#define GPS_TX_PIN -1   // nicht genutzt, Pin-Konflikt mit Servo; ändere wenn nötig
#define GPS_BAUD   9600
HardwareSerial GPS_Serial(2);
TinyGPSPlus gps;

// ========================= OBJEKTE =========================
Adafruit_MPU6050 mpu;
Servo servoX, servoY;

// ========================= SOLLWERTE =========================
float setpoint_pitch = 0.0f;
float setpoint_yaw   = 0.0f;

// ========================= KALMAN-ZUSTAND =========================
struct Kalman {
  float angle;
  float bias;
  float rate;
  float P[2][2];
};
Kalman kal_pitch, kal_yaw;

// ========================= PID-ZUSTAND =========================
float last_error_pitch = 0, integral_pitch = 0;
float last_error_yaw   = 0, integral_yaw   = 0;
unsigned long last_time = 0;

// ========================= IMU-KALIBRIERUNG =========================
bool  imu_calibrated   = false;
float imu_offset_pitch = 0.0f;
float imu_offset_yaw   = 0.0f;  // Gyro-Z (Yaw-Rate) Offset

// -------------------------------------------------------------
// Kalman init
// -------------------------------------------------------------
void kalmanInit(Kalman &kal, float init_angle = 0.0f) {
  kal.angle = init_angle;
  kal.bias  = 0.0f;
  kal.rate  = 0.0f;
  kal.P[0][0] = 1.0f;
  kal.P[0][1] = 0.0f;
  kal.P[1][0] = 0.0f;
  kal.P[1][1] = 1.0f;
}

// -------------------------------------------------------------
// Kalman update
// -------------------------------------------------------------
float kalmanUpdate(Kalman &kal, float new_angle, float new_rate, float dt) {
  kal.rate = new_rate - kal.bias;
  kal.angle += dt * kal.rate;

  kal.P[0][0] += dt * (dt*kal.P[1][1] - kal.P[0][1] - kal.P[1][0] + q_angle);
  kal.P[0][1] -= dt * kal.P[1][1];
  kal.P[1][0] -= dt * kal.P[1][1];
  kal.P[1][1] += q_bias * dt;

  float S = kal.P[0][0] + r_measure;
  float K[2];
  K[0] = kal.P[0][0] / S;
  K[1] = kal.P[1][0] / S;

  float y = new_angle - kal.angle;
  kal.angle += K[0] * y;
  kal.bias  += K[1] * y;

  float P00_temp = kal.P[0][0];
  float P01_temp = kal.P[0][1];

  kal.P[0][0] -= K[0] * P00_temp;
  kal.P[0][1] -= K[0] * P01_temp;
  kal.P[1][0] -= K[1] * P00_temp;
  kal.P[1][1] -= K[1] * P01_temp;

  return kal.angle;
}

// -------------------------------------------------------------
// PID update
// -------------------------------------------------------------
float pidUpdate(float setpoint, float measured, float &last_error, float &integral, float dt) {
  float error = setpoint - measured;
  integral += error * dt;
  float derivative = (error - last_error) / dt;
  last_error = error;
  return kp * error + ki * integral + kd * derivative;
}

// -------------------------------------------------------------
// Nächstes freies Logfile finden -> Pfad in outPath schreiben
// -------------------------------------------------------------
void buildLogFilename(uint16_t idx, char *outPath, size_t len) {
  snprintf(outPath, len, LOGFILE_PREFIX "%04u" LOGFILE_EXT, idx);
}

bool openNextLogFile() {
  char path[32];
  for (uint16_t i = 0; i <= LOGFILE_MAX_IDX; ++i) {
    buildLogFilename(i, path, sizeof(path));
    if (!SD.exists(path)) {
      logFile = SD.open(path, FILE_WRITE);
      if (logFile) {
        Serial.print("[LOG] Neues Logfile: ");
        Serial.println(path);
        return true;
      } else {
        Serial.print("[ERROR] Kann Logfile nicht öffnen: ");
        Serial.println(path);
        return false;
      }
    }
  }
  Serial.println("[ERROR] Kein freier Log-Index mehr!");
  return false;
}

// -------------------------------------------------------------
// SD: Header schreiben
// -------------------------------------------------------------
void writeLogHeader() {
  if (!sd_ok) return;
  if (!logFile) return;
  logFile.println("millis,set_pitch,set_yaw,pitch,yaw,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,pwm_x,pwm_y,gps_fix,lat,lon,alt_m,speed_kmh,sats");
  logFile.flush();  // Header sofort sichern
}

// -------------------------------------------------------------
// SD: Datenzeile schreiben
// -------------------------------------------------------------
void logDataCSV(uint32_t ms,
                float setP, float setY,
                float pitch, float yaw,
                float accX, float accY, float accZ,
                float gyroX, float gyroY, float gyroZ,
                int pwmX, int pwmY,
                bool gps_fix,
                double lat, double lon,
                double alt_m, double speed_kmh,
                uint32_t sats) {
  if (!sd_ok) return;
  if (!logFile) return;

  logFile.printf("%lu,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%u,%.7f,%.7f,%.2f,%.2f,%u\n",
                 (unsigned long)ms,
                 setP, setY,
                 pitch, yaw,
                 accX, accY, accZ,
                 gyroX, gyroY, gyroZ,
                 pwmX, pwmY,
                 (unsigned)gps_fix,
                 lat, lon,
                 alt_m, speed_kmh,
                 (unsigned)sats);

  // periodisch flushen
  uint32_t now = millis();
  if (now - last_flush_ms >= LOG_FLUSH_INTERVAL_MS) {
    logFile.flush();
    last_flush_ms = now;
  }
}

// -------------------------------------------------------------
// GPS Poll (nicht blockierend) -> globaler Parser-State wird aktualisiert
// -------------------------------------------------------------
void pollGPS() {
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }
}

// -------------------------------------------------------------
// SETUP
// -------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // --- IMU ---
  if (!mpu.begin()) {
    Serial.println("[ERROR] MPU6050 nicht gefunden!");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("[INFO] MPU6050 initialisiert.");
  }

  // --- Servos ---
  servoX.setPeriodHertz(50);
  servoY.setPeriodHertz(50);
  servoX.attach(SERVO_X_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoY.attach(SERVO_Y_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoX.writeMicroseconds(SERVO_MID_US);
  servoY.writeMicroseconds(SERVO_MID_US);

  // --- SD ---
  spi.begin(SD_CLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, spi)) {
    Serial.println("[ERROR] SD-Karte konnte nicht initialisiert werden.");
    sd_ok = false;
  } else {
    Serial.println("[INFO] SD-Karte erfolgreich initialisiert.");
    if (openNextLogFile()) {
      sd_ok = true;
      writeLogHeader();
    } else {
      sd_ok = false;
    }
  }

  // --- GPS ---
  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("[INFO] GPS-Daten werden geparst...");

  // --- IMU-Kalibrierung ---
  Serial.println("[INFO] Starte IMU-Kalibrierung. Rakete senkrecht und ruhig halten...");
  delay(3000);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  imu_offset_pitch = atan2(a.acceleration.x, a.acceleration.z) * RAD_TO_DEG;
  imu_offset_yaw   = g.gyro.z * RAD_TO_DEG; // Yaw-Rate-Offset

  imu_calibrated = true;
  Serial.println("[INFO] Kalibrierung abgeschlossen.");
  Serial.printf("[CAL] pitch_offset=%.3f deg, yaw_rate_offset=%.3f deg/s\n", imu_offset_pitch, imu_offset_yaw);

  // Kalman initialisieren auf kalibrierte Startlage
  kalmanInit(kal_pitch, 0.0f);
  kalmanInit(kal_yaw,   0.0f);

  last_time = millis();
  last_log_ms = last_time;
  last_flush_ms = last_time;
}

// -------------------------------------------------------------
// LOOP
// -------------------------------------------------------------
void loop() {
  // GPS non-blocking einlesen
  pollGPS();

  // IMU-Daten holen
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = millis();
  float dt = (now - last_time) / 1000.0f;
  last_time = now;

  // Rohdaten in nutzbare Einheiten
  float accX = a.acceleration.x;
  float accY = a.acceleration.y;
  float accZ = a.acceleration.z;

  float gyroX = g.gyro.x * RAD_TO_DEG;
  float gyroY = g.gyro.y * RAD_TO_DEG;
  float gyroZ = g.gyro.z * RAD_TO_DEG;

  // Offsets anwenden
  float acc_pitch = atan2(accX, accZ) * RAD_TO_DEG - imu_offset_pitch;
  float yaw_rate  = gyroZ - imu_offset_yaw;

  // Kalman: Pitch mit Acc + GyroY; Yaw nur Gyro (kein Acc-Winkel)
  float pitch = kalmanUpdate(kal_pitch, acc_pitch, gyroY, dt);
  float yaw   = kalmanUpdate(kal_yaw,   0.0f,      yaw_rate, dt);

  // PID -> Servos
  float out_pitch = pidUpdate(setpoint_pitch, pitch, last_error_pitch, integral_pitch, dt);
  float out_yaw   = pidUpdate(setpoint_yaw,   yaw,   last_error_yaw,   integral_yaw,   dt);

  int pwm_pitch = constrain((int)(SERVO_MID_US + out_pitch), SERVO_MIN_US, SERVO_MAX_US);
  int pwm_yaw   = constrain((int)(SERVO_MID_US + out_yaw),   SERVO_MIN_US, SERVO_MAX_US);

  servoX.writeMicroseconds(pwm_pitch);
  servoY.writeMicroseconds(pwm_yaw);

  // === GPS DATEN AUSLESEN ===
  bool gps_fix = gps.location.isValid();
  double lat   = gps_fix ? gps.location.lat() : NAN;
  double lon   = gps_fix ? gps.location.lng() : NAN;
  double alt_m = gps.altitude.isValid() ? gps.altitude.meters() : NAN;
  double speed_kmh = gps.speed.isValid() ? gps.speed.kmph() : NAN;
  uint32_t sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

  // --- SERIAL DEBUG (20 Hz getaktet) ---
  static uint32_t last_serial_ms = 0;
  if (now - last_serial_ms >= LOG_PERIOD_MS) {
    last_serial_ms = now;
    Serial.printf("Pitch: %.2f°, Yaw: %.2f°, PWM X: %d, Y: %d\n", pitch, yaw, pwm_pitch, pwm_yaw);
    Serial.printf("RAW Accel(m/s^2): X=%.2f Y=%.2f Z=%.2f | Gyro(deg/s): X=%.2f Y=%.2f Z=%.2f\n",
                  accX, accY, accZ, gyroX, gyroY, gyroZ);
    Serial.printf("Servo PWM -> X:%dµs Y:%dµs | setP:%.2f setY:%.2f\n",
                  pwm_pitch, pwm_yaw, setpoint_pitch, setpoint_yaw);
    if (gps_fix) {
      Serial.printf("GPS: lat=%.7f lon=%.7f alt=%.2fm spd=%.2fkm/h sats=%u\n", lat, lon, alt_m, speed_kmh, sats);
    } else {
      Serial.println("GPS: kein Fix");
    }
  }

  // --- SD LOGGING @20Hz ---
  if (now - last_log_ms >= LOG_PERIOD_MS) {
    last_log_ms += LOG_PERIOD_MS; // stabiler Intervall, nicht now
    logDataCSV(now,
               setpoint_pitch, setpoint_yaw,
               pitch, yaw,
               accX, accY, accZ,
               gyroX, gyroY, gyroZ,
               pwm_pitch, pwm_yaw,
               gps_fix,
               lat, lon,
               alt_m, speed_kmh,
               sats);
  }
}
