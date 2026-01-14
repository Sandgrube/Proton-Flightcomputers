// TRACER TVC Control Script + Webserver
// ESP32 | Servos, MPU6050, GPS, SD-Logging, Batterie | WebGUI zum Starten
// 2025-07-24 | Quirin

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <SD.h>

Servo chuteServo;


// === KONFIGURATION ===
const char APOGEE_AXIS = 'Y';               // 'X', 'Y' oder 'Z' → Richtung des Aufstiegs
const float APOGEE_ACCEL_THRESHOLD = 0.3;   // [g] < 0.3g gilt als freier Fall
const int REQUIRED_FREEFALL_MS = 300;       // Dauer für Freefall (in ms)
const int SMOOTHING_WINDOW = 10;            // Mittelwert über N Messwerte

const int SERVO_PIN = 32;                   // GPIO für Servo (z. B. D33) oder D32
const int SERVO_DEPLOY_ANGLE = 95;          // Zielwinkel zur Auslösung
const int SERVO_IDLE_ANGLE = 0;             // Ruheposition

// === STATUSVARIABLEN ===
float accelBuffer[SMOOTHING_WINDOW];
int bufferIndex = 0;
bool ascentDetected = false;
unsigned long freefallStart = 0;
bool apogeeDetected = false;




// ========================= KALMAN-ZUSTAND =========================
struct Kalman {
  float angle;
  float bias;
  float rate;
  float P[2][2];
};
Kalman kal_pitch, kal_yaw;


// ========================= WIFI =========================
#define WIFI_SSID     "ESP32-TRACER"
#define WIFI_PASS     "1234"
WebServer server(80);
String currentState = "idle";
bool startTriggered = false;
bool emergencyStop = false;

// ========================= I/O PINS =========================
#define BUZZER_PIN 16
#define LED1_PIN 4
#define LED2_PIN 2
#define LED3_PIN 15



// ========================= WEB HANDLER =========================
void handleRoot() {
  String html = "<html><head><meta charset='utf-8'><style>body{font-family:sans-serif;}h2{color:#005;}form{margin-bottom:1em;}table{border-collapse:collapse;}td,th{border:1px solid #ccc;padding:4px;}</style></head><body>";
  html += "<h2>TRACER - Flight Control</h2>";
  html += "<div style='margin:10px 0;'><img src='/logo.png' style='max-width:200px;'></div>";

  html += "<form method='POST' action='/set'>";
  html += "Password: <input type='password' name='pwd'><br>";
  html += String("State: <select name='state'><option value='idle'") +
          (currentState == "idle" ? " selected" : "") +
          ">idle</option><option value='armed'" +
          (currentState == "armed" ? " selected" : "") +
          ">armed</option></select><br>";
  html += "<input type='submit' value='Set State'>";
  html += "</form>";

  html += "<form method='POST' action='/resetKalman'><input type='submit' value='Reset Kalman Filter'></form>";
  html += "<form method='POST' action='/emergency'><input type='submit' value='EMERGENCY STOP' style='background:red;color:white;'></form>";

  html += "<p><strong>Current State:</strong> " + currentState + "</p>";
  html += "<p><a href='/data'>Live Telemetrie anzeigen</a></p>";
  html += "<p><strong>Batteriespannung:</strong> " + String(readBatteryVoltage(), 2) + " V</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleSet() {
  if (server.hasArg("pwd") && server.arg("pwd") == WIFI_PASS) {
    if (server.hasArg("state")) {
      currentState = server.arg("state");
      if (currentState == "armed") startTriggered = true;
    }
    server.sendHeader("Location", "/");
    server.send(302, "text/plain", "");
  } else {
    server.send(403, "text/plain", "Wrong password");
  }
}

// ----------------------------------------------------------------------------
// Kalman init
// ----------------------------------------------------------------------------
void kalmanInit(Kalman &kal, float init_angle = 0.0f) {
  kal.angle = init_angle;
  kal.bias  = 0.0f;
  kal.rate  = 0.0f;
  kal.P[0][0] = 1.0f; kal.P[0][1] = 0.0f;
  kal.P[1][0] = 0.0f; kal.P[1][1] = 1.0f;
}

void handleResetKalman() {
  kalmanInit(kal_pitch, 0.0f);
  kalmanInit(kal_yaw, 0.0f);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "Kalman reset");
}

void handleEmergency() {
  emergencyStop = true;
  currentState = "idle";
  startTriggered = false;
  digitalWrite(LED1_PIN, HIGH);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "Emergency Stop ausgelöst");
}

String latestTelemetry = "no data";
void handleData() {
  server.send(200, "text/html", latestTelemetry);
}

// ========================= STARTUP MELODY =========================
void playStartupMelody() {
  tone(BUZZER_PIN, 880, 150);
  delay(200);
  tone(BUZZER_PIN, 988, 150);
  delay(200);
  tone(BUZZER_PIN, 1319, 300);
  delay(400);
  noTone(BUZZER_PIN);
}

// ========================= RGB LED STATUS =========================
void setLEDState(const String &state) {
  if (state == "idle") {
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED3_PIN, LOW);
  } else if (state == "armed") {
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, HIGH);
    digitalWrite(LED3_PIN, LOW);
  } else if (state == "active") {
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED3_PIN, HIGH);
  } else {
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED3_PIN, LOW);
  }
}

// ========================= HOOKS =========================
void extendedSetupHooks() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);

  setLEDState("idle");
  playStartupMelody();

  server.on("/resetKalman", HTTP_POST, handleResetKalman);
  server.on("/emergency", HTTP_POST, handleEmergency);
}

// Muss in setup() aufgerufen werden:
// extendedSetupHooks();

// ========================= INIT-FLAG =========================
bool isInitialized = false;



#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513f
#endif
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.01745329252f
#endif

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

#define LOGFILE_PREFIX "/flight_"
#define LOGFILE_EXT    ".csv"
#define LOGFILE_MAX_IDX 9999

// Logging-Takt & Flush
const uint32_t LOG_PERIOD_MS = 50;   // 20 Hz
const uint32_t LOG_FLUSH_INTERVAL_MS = 250; // ~5 Zeilen
uint32_t last_log_ms = 0;
uint32_t last_flush_ms = 0;

// Optionaler Zeilen-Puffer (reduziert Fragmentierung)
#define LOG_BUFFER_SIZE 512
static char logBuffer[LOG_BUFFER_SIZE];
static size_t logBufferIndex = 0;
static uint16_t logBufferedLines = 0;

// ========================= GPS =========================
// UART2 (HardwareSerial(2))
#define GPS_RX_PIN 5   // vom GPS-TX kommend -> ESP32 RX
#define GPS_TX_PIN -1   // unbenutzt; Pin 26 ist Servo -> keine Kollission
#define GPS_BAUD   9600
HardwareSerial GPS_Serial(2);
TinyGPSPlus gps;

// ========================= BATTERIE =========================
const int adcPin = 34;      // ADC1_CH6 -> stabil, kein WiFi Noise
const float R1 = 22000.0;   // Oben -> +Bat
const float R2 = 6800.0;    // Unten -> ADC
const float Vref = 3.3;     // ADC Referenz (ESP32 intern ~3.3V, Kalibrierung empf.)
const int   ADC_Resolution = 4095;   // 12 Bit
const float correctionFactor = 1.045; // empirisch
const float voltageMax = 11.4;  // 100 % (3S LiPo unter Last konservativ)
const float voltageMin = 9.6;   // 0 %


// ========================= BATTERIESPANNUNG =========================
float readBatteryVoltage() {
  int raw = analogRead(adcPin);
  float voltageAtAdc = (raw / float(ADC_Resolution)) * Vref;
  float batteryVoltage = voltageAtAdc * (R1 + R2) / R2 * correctionFactor;
  return batteryVoltage;
}



int calcBatteryPercent(float voltage) {
  float percent = (voltage - voltageMin) / (voltageMax - voltageMin) * 100.0f;
  percent = constrain(percent, 0.0f, 100.0f);
  return int(percent + 0.5f);
}

// ========================= OBJEKTE =========================
Adafruit_MPU6050 mpu;
Servo servoX, servoY;

// ========================= SOLLWERTE =========================
float setpoint_pitch = 0.0f;  // deg
float setpoint_yaw   = 0.0f;  // deg


// ========================= PID-ZUSTAND =========================
float last_error_pitch = 0, integral_pitch = 0;
float last_error_yaw   = 0, integral_yaw   = 0;
unsigned long last_time = 0;  // ms

// ========================= IMU-KALIBRIERUNG =========================
bool  imu_calibrated   = false;
float imu_offset_pitch = 0.0f;  // deg
float imu_offset_yaw   = 0.0f;  // deg/s (Gyro-Z Bias)



// ----------------------------------------------------------------------------
// Kalman update
// ----------------------------------------------------------------------------
float kalmanUpdate(Kalman &kal, float new_angle, float new_rate, float dt) {
  // Vorhersage
  kal.rate = new_rate - kal.bias;
  kal.angle += dt * kal.rate;

  // Kovarianz vorhersagen
  kal.P[0][0] += dt * (dt*kal.P[1][1] - kal.P[0][1] - kal.P[1][0] + q_angle);
  kal.P[0][1] -= dt * kal.P[1][1];
  kal.P[1][0] -= dt * kal.P[1][1];
  kal.P[1][1] += q_bias * dt;

  // Messung einfließen lassen
  float S = kal.P[0][0] + r_measure;
  float K0 = kal.P[0][0] / S;
  float K1 = kal.P[1][0] / S;

  float y = new_angle - kal.angle; // Innovationsresiduum
  kal.angle += K0 * y;
  kal.bias  += K1 * y;

  float P00_temp = kal.P[0][0];
  float P01_temp = kal.P[0][1];

  kal.P[0][0] -= K0 * P00_temp;
  kal.P[0][1] -= K0 * P01_temp;
  kal.P[1][0] -= K1 * P00_temp;
  kal.P[1][1] -= K1 * P01_temp;

  return kal.angle;
}

// ----------------------------------------------------------------------------
// PID update
// ----------------------------------------------------------------------------
float pidUpdate(float setpoint, float measured, float &last_error, float &integral, float dt) {
  float error = setpoint - measured;
  integral += error * dt;
  float derivative = (error - last_error) / dt;
  last_error = error;
  return kp * error + ki * integral + kd * derivative;
}

// ----------------------------------------------------------------------------
// SD: Hilfsfunktionen
// ----------------------------------------------------------------------------
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
        Serial.print("[LOG] Neues Logfile: "); Serial.println(path);
        return true;
      } else {
        Serial.print("[ERROR] Kann Logfile nicht öffnen: "); Serial.println(path);
        return false;
      }
    }
  }
  Serial.println("[ERROR] Kein freier Log-Index mehr!");
  return false;
}

void flushLogBuffer() {
  if (!sd_ok || !logFile || logBufferIndex == 0) return;
  logFile.write((const uint8_t*)logBuffer, logBufferIndex);
  logFile.flush();
  logBufferIndex = 0;
  logBufferedLines = 0;
}

void logWriteLine(const char *line) {
  if (!sd_ok || !logFile) return;
  size_t len = strlen(line);
  if (len + 1 + logBufferIndex >= LOG_BUFFER_SIZE) {
    flushLogBuffer();
  }
  memcpy(&logBuffer[logBufferIndex], line, len);
  logBufferIndex += len;
  logBuffer[logBufferIndex++] = '\n';
  logBufferedLines++;

  if (logBufferedLines >= 10) { // harte Grenze nach 10 Zeilen
    flushLogBuffer();
  }
}

void writeLogHeader() {
  if (!sd_ok) return;
  const char *hdr = "millis,set_pitch,set_yaw,pitch,yaw,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,pwm_x,pwm_y,gps_fix,lat,lon,alt_m,speed_kmh,sats,batt_v,batt_pct";
  logWriteLine(hdr);
  flushLogBuffer(); // Header sofort sichern
}

void logDataCSV(uint32_t ms,
                float setP, float setY,
                float pitch, float yaw,
                float accX, float accY, float accZ,
                float gyroX, float gyroY, float gyroZ,
                int pwmX, int pwmY,
                bool gps_fix,
                double lat, double lon,
                double alt_m, double speed_kmh,
                uint32_t sats,
                float batt_v, int batt_pct) {
  if (!sd_ok) return;
  char line[256];
  snprintf(line, sizeof(line),
           "%lu,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%u,%.7f,%.7f,%.2f,%.2f,%u,%.2f,%d",
           (unsigned long)ms,
           setP, setY,
           pitch, yaw,
           accX, accY, accZ,
           gyroX, gyroY, gyroZ,
           pwmX, pwmY,
           (unsigned)gps_fix,
           lat, lon,
           alt_m, speed_kmh,
           (unsigned)sats,
           batt_v, batt_pct);
  logWriteLine(line);

  uint32_t now = millis();
  if (now - last_flush_ms >= LOG_FLUSH_INTERVAL_MS) {
    flushLogBuffer();
    last_flush_ms = now;
  }
}

// ----------------------------------------------------------------------------
// GPS Poll (nicht blockierend)
// ----------------------------------------------------------------------------
void pollGPS() {
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }
}


void setup() {
  Serial.begin(115200);
  delay(200);

  // WiFi Init
  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.print("[WIFI] IP: "); Serial.println(WiFi.softAPIP());

  extendedSetupHooks();

  server.on("/", handleRoot);
  server.on("/set", HTTP_POST, handleSet);
  server.on("/data", handleData);
  server.begin();
  server.serveStatic("/logo.png", SD, "/logo.png");


 // --- I2C / IMU ---
  Wire.begin();
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
    }
  }

  // --- GPS ---
  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("[INFO] GPS-Daten werden geparst...");

  // --- Batterie-ADC ---
  analogReadResolution(12);        // sicherstellen (manchmal default)
  analogSetAttenuation(ADC_11db);  // bis ~3.9V
  pinMode(adcPin, INPUT);

  // --- IMU-Kalibrierung ---
  Serial.println("[INFO] Starte IMU-Kalibrierung. Rakete senkrecht und ruhig halten...");
  delay(3000);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  imu_offset_pitch = atan2(a.acceleration.x, a.acceleration.z) * RAD_TO_DEG;
  imu_offset_yaw   = g.gyro.z * RAD_TO_DEG; // Gyro-Z Bias
  imu_calibrated = true;
  Serial.printf("[CAL] pitch_offset=%.3f deg, yaw_rate_offset=%.3f deg/s\n", imu_offset_pitch, imu_offset_yaw);

  // Kalman initialisieren
  kalmanInit(kal_pitch, 0.0f);
  kalmanInit(kal_yaw,   0.0f);

  last_time = millis();
  last_log_ms = last_time;
  last_flush_ms = last_time;
  isInitialized = true;



  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  chuteServo.setPeriodHertz(50);      // 50 Hz Standardfrequenz für Servos
  chuteServo.attach(SERVO_PIN, 500, 2400); // Standardpulse: 0°–180°
  chuteServo.write(SERVO_IDLE_ANGLE); // Inaktiv-Position

  Serial.println("MPU6050 bereit. Starte Apogäumerkennung...");
  Serial.print("Genutzte Achse: ");
  Serial.println(APOGEE_AXIS);


}

// ========================= LOOP =========================
void loop() {
  server.handleClient();

  // Notfall: sofort alles stoppen
  if (emergencyStop) {
    servoX.writeMicroseconds(SERVO_MID_US);
    servoY.writeMicroseconds(SERVO_MID_US);
    return;
  }

  // Startbedingungen prüfen
  if (!isInitialized || !startTriggered || currentState != "armed") {
    delay(10);
    return;
  }

  // GPS Polling
  pollGPS();


  checkApogee();

  // IMU lesen
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = millis();
  float dt = (now - last_time) * 0.001f;
  if (dt <= 0.0f) dt = 0.001f;
  last_time = now;

  // Rohdaten
  float accX = a.acceleration.x;
  float accY = a.acceleration.y;
  float accZ = a.acceleration.z;
  float gyroX = g.gyro.x * RAD_TO_DEG;
  float gyroY = g.gyro.y * RAD_TO_DEG;
  float gyroZ = g.gyro.z * RAD_TO_DEG;

  // Offsets
  float acc_pitch = atan2(accX, accZ) * RAD_TO_DEG - imu_offset_pitch;
  float yaw_rate  = gyroZ - imu_offset_yaw;

  // Kalman
  float pitch = kalmanUpdate(kal_pitch, acc_pitch, gyroY, dt);
  float yaw   = kalmanUpdate(kal_yaw, 0.0f, yaw_rate, dt);

  // PID-Regler
  float out_pitch = pidUpdate(setpoint_pitch, pitch, last_error_pitch, integral_pitch, dt);
  float out_yaw   = pidUpdate(setpoint_yaw, yaw, last_error_yaw, integral_yaw, dt);

  int pwm_pitch = constrain(int(SERVO_MID_US + out_pitch), SERVO_MIN_US, SERVO_MAX_US);
  int pwm_yaw   = constrain(int(SERVO_MID_US + out_yaw),   SERVO_MIN_US, SERVO_MAX_US);

  servoX.writeMicroseconds(pwm_pitch);
  servoY.writeMicroseconds(pwm_yaw);

  // GPS-Daten
  bool gps_fix = gps.location.isValid();
  double lat = gps_fix ? gps.location.lat() : 0.0;
  double lon = gps_fix ? gps.location.lng() : 0.0;
  double alt_m = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
  double speed_kmh = gps.speed.isValid() ? gps.speed.kmph() : 0.0;
  uint32_t sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

  // Batterie
  float batt_v = readBatteryVoltage();
  int batt_pct = calcBatteryPercent(batt_v);

  // Debug-Ausgabe
  static uint32_t last_serial_ms = 0;
  if (now - last_serial_ms >= LOG_PERIOD_MS) {
    last_serial_ms = now;
    Serial.printf("Pitch: %.2f°, Yaw: %.2f°, PWM X: %d, Y: %d\n", pitch, yaw, pwm_pitch, pwm_yaw);
    Serial.printf("RAW Accel: X=%.2f Y=%.2f Z=%.2f | Gyro: X=%.2f Y=%.2f Z=%.2f\n", accX, accY, accZ, gyroX, gyroY, gyroZ);
    Serial.printf("Servo PWM: X=%dµs Y=%dµs | SetP=%.2f SetY=%.2f\n", pwm_pitch, pwm_yaw, setpoint_pitch, setpoint_yaw);
    Serial.printf("Batterie: %.2f V (%d%%)\n", batt_v, batt_pct);
    if (gps_fix) {
      Serial.printf("GPS: lat=%.7f lon=%.7f alt=%.2fm spd=%.2fkm/h sats=%u\n", lat, lon, alt_m, speed_kmh, sats);
    } else {
      Serial.println("GPS: kein Fix");
    }
  }

  // SD Logging
  if (now - last_log_ms >= LOG_PERIOD_MS) {
    last_log_ms += LOG_PERIOD_MS;
    logDataCSV(now,
               setpoint_pitch, setpoint_yaw,
               pitch, yaw,
               accX, accY, accZ,
               gyroX, gyroY, gyroZ,
               pwm_pitch, pwm_yaw,
               gps_fix,
               lat, lon,
               alt_m, speed_kmh,
               sats,
               batt_v, batt_pct);
  }

  // Live-Telemetrie HTML generieren
  char html[1024];
  snprintf(html, sizeof(html),
    "<html><head><meta http-equiv='refresh' content='1'><style>body{font-family:sans-serif;}table{border-collapse:collapse;}td,th{border:1px solid #ccc;padding:4px;}</style></head><body><h2>TRACER Telemetrie</h2><table>"
    "<tr><th>Pitch</th><th>Yaw</th><th>SetP</th><th>SetY</th><th>Battery</th><th>Sats</th><th>Speed</th><th>Alt</th></tr>"
    "<tr><td>%.2f&deg;</td><td>%.2f&deg;</td><td>%.2f</td><td>%.2f</td><td>%.2fV</td><td>%u</td><td>%.2fkm/h</td><td>%.2fm</td></tr>"
    "</table><p><a href='/'>Zurück</a></p></body></html>",
    pitch, yaw, setpoint_pitch, setpoint_yaw, batt_v, sats, speed_kmh, alt_m);
  latestTelemetry = html;
}

void checkApogee() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float selectedAcc = 0.0;
  switch (APOGEE_AXIS) {
    case 'X': selectedAcc = a.acceleration.x; break;
    case 'Y': selectedAcc = a.acceleration.y; break;
    case 'Z': selectedAcc = a.acceleration.z; break;
    default:  selectedAcc = a.acceleration.x; break;
  }

  float accG = selectedAcc / 9.81;

  accelBuffer[bufferIndex] = accG;
  bufferIndex = (bufferIndex + 1) % SMOOTHING_WINDOW;
  float avgAcc = 0;
  for (int i = 0; i < SMOOTHING_WINDOW; i++) avgAcc += accelBuffer[i];
  avgAcc /= SMOOTHING_WINDOW;

  if (!ascentDetected && avgAcc > 0.5) {
    ascentDetected = true;
    Serial.println("Aufstieg erkannt.");
  }

  if (ascentDetected && fabs(avgAcc) < APOGEE_ACCEL_THRESHOLD) {
    if (freefallStart == 0) freefallStart = millis();
    else if ((millis() - freefallStart > REQUIRED_FREEFALL_MS) && !apogeeDetected) {
      apogeeDetected = true;
      Serial.println("🚀 APOGÄUM ERKANNT!");
      deployParachutes();  // Fallschirmauslösung
    }
  } else {
    freefallStart = 0;
  }

  Serial.print("Accel[");
  Serial.print(APOGEE_AXIS);
  Serial.print("]: ");
  Serial.print(avgAcc, 3);
  Serial.print(" g | Apogee: ");
  Serial.println(apogeeDetected ? "JA" : "NEIN");
}

void deployParachutes() {
  Serial.println("🎯 Fallschirmauslösung...");
  chuteServo.write(SERVO_DEPLOY_ANGLE);  // Servo auf 95°
}
