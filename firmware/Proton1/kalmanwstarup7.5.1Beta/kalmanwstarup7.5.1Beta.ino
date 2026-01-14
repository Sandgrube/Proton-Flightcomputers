#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <esp_now.h>
#include <WiFi.h>

// Pin Definitionen
#define BUZZER_PIN 27
#define CS_PIN 15
#define MOSI_PIN 23
#define CLK_PIN 18
#define MISO_PIN 19
#define LED_PIN 34  // LED auf Pin D18

// MAC-Adresse des Empfängers
uint8_t receiverMac[] = {0xD0, 0xEF, 0x76, 0xEF, 0xCC, 0x64};

// Globale Variablen
int counter = 0; // Sendecounter
MPU6050 mpu;
Servo servoX; // Servo für X-Achse (Pin 17)
Servo servoY; // Servo für Y-Achse (Pin 5)

// BMP180 Initialisierung
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(); 

// Kalman-Filter Variablen
float Q_angle = 0.001;
float Q_bias = 0.003;
float R_measure = 0.03;

float angleX = 0, biasX = 0, rateX = 0;
float angleY = 0, biasY = 0, rateY = 0;

float P[2][2] = {{0, 0}, {0, 0}}; // Fehler-Kovarianzmatrix für X
float P_Y[2][2] = {{0, 0}, {0, 0}}; // Fehler-Kovarianzmatrix für Y

// Zeitvariablen
unsigned long lastTime;
unsigned long startTime;
unsigned long previousMillis = 0;
const long interval = 25; // 25 ms -> 40 mal pro Sekunde

// SD-Karte
File logFile;

// Raketenstatus
bool rocketLaunched = false;
unsigned long launchTime;

// Servo-Positionen
int servoXPos = 90; // Standardposition
int servoYPos = 90; 

// Setup-Funktion
void setup() {
  // Initialisierung von ESP-NOW
  setupESPNow();

  // Serielle Kommunikation und I2C starten
  Serial.begin(115200);
  Wire.begin();

  // Initialisierung der Sensoren und SD-Karte
  initSensors();
  initSDCard();

  // Servos und Buzzer einrichten
  initServos();
  initBuzzer();

  // Startzeit für Logging setzen
  startTime = millis();
  lastTime = micros();
}

// Hauptprogrammloop
void loop() {
  sendSensorData();  // Senden von Sensor- und Höhenwerten per ESP-NOW
  handleServosAndLogging(); // Verarbeiten der Servopositionen und Logging
}

// --- ESP-NOW Setup ---
void setupESPNow() {
  WiFi.mode(WIFI_STA); // WiFi in den STA-Modus schalten
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Initialisierung fehlgeschlagen");
    return;
  }

  // Empfänger als Peer hinzufügen
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Empfänger konnte nicht hinzugefügt werden");
    return;
  }

  // Startnachricht senden
  char message[32];
  sprintf(message, "Rakete Startbereit");
  esp_now_send(receiverMac, (uint8_t *)message, strlen(message));
}

// --- Sensoren und SD-Karte Initialisierung ---
void initSensors() {
  // MPU6050 Initialisierung
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 nicht gefunden!");
    while (1);
  }

  // BMP180 Initialisierung
  if (!bmp.begin()) {
    Serial.println("BMP180 nicht gefunden!");
    while (1);
  }
}

void initSDCard() {
  // SPI und SD-Karte initialisieren
  SPI.begin(CLK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  if (!SD.begin(CS_PIN)) {
    Serial.println("SD-Karte konnte nicht initialisiert werden!");
    return;
  }

  logFile = SD.open("/sensor_data.txt", FILE_WRITE);
  if (!logFile) {
    Serial.println("Konnte sensor_data.txt nicht öffnen!");
    return;
  }

  logFile.println("Zeit, WinkelX, WinkelY, ServoX, ServoY, Luftdruck");
  logFile.close();
}

// --- Servos und Buzzer Initialisierung ---
void initServos() {
  servoX.attach(17);
  servoY.attach(5);
  servoX.write(servoXPos);
  servoY.write(servoYPos);
}

void initBuzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  playMelody(); // Ton abspielen und Servos bewegen
}

// --- Sensor- und Servodaten senden ---
void sendSensorData() {
  char message[64];
  float altitude = calculateAltitude(); // Berechnung der Höhe basierend auf BMP180
  sprintf(message, "Hallo %d, Höhe: %.2f m", counter, altitude);
  esp_now_send(receiverMac, (uint8_t *)message, strlen(message));
  counter++;
  delay(100); // 10 Nachrichten pro Sekunde
}

float calculateAltitude() {
  float seaLevelPressure = 1013.25;
  float pressure;
  bmp.getPressure(&pressure);  
  return 44330.0 * (1.0 - pow(pressure / (seaLevelPressure * 100.0), 0.1903));  
}

// --- Servos und Logging verarbeiten ---
void handleServosAndLogging() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    int16_t rawAccelX, rawAccelY, rawAccelZ;
    int16_t rawGyroX, rawGyroY, rawGyroZ;

    // Sensordaten lesen
    mpu.getAcceleration(&rawAccelX, &rawAccelY, &rawAccelZ);
    mpu.getRotation(&rawGyroX, &rawGyroY, &rawGyroZ);

    // Umwandeln der rohen Sensordaten in "echte" Werte
    float accelX = rawAccelX / 16384.0;
    float accelY = rawAccelY / 16384.0;
    float accelZ = rawAccelZ / 16384.0;

    float gyroX = rawGyroX / 131.0;
    float gyroY = rawGyroY / 131.0;

    // Kalman-Filter anwenden
    unsigned long dt = micros() - lastTime;
    lastTime = micros();
    applyKalmanFilter(accelX, gyroX, accelY, gyroY, dt);

    // Start erkennen und LED aktivieren
    detectLaunch(accelZ);

    // Servos bewegen und Daten loggen
    if (rocketLaunched && (millis() - launchTime > 500)) {
      updateServos();
      logData();
    }
  }
}

// --- Kalman-Filter anwenden ---
void applyKalmanFilter(float accelX, float gyroX, float accelY, float gyroY, unsigned long dt) {
  float accelAngleX = atan2(accelY, 1.0) * 180 / PI;
  float accelAngleY = atan2(-accelX, 1.0) * 180 / PI;

  angleX = kalmanFilter(accelAngleX, gyroX, &biasX, &P[0][0], dt / 1000000.0);
  angleY = kalmanFilter(accelAngleY, gyroY, &biasY, &P_Y[0][0], dt / 1000000.0);
}

float kalmanFilter(float newAngle, float newRate, float *bias, float *P, float dt) {
  float rate = newRate - *bias;
  float angle = newAngle + dt * rate;

  P[0] += dt * (dt * P[1] - P[0] - P[1] + Q_angle);
  P[1] += -dt * P[1];
  P[1] += dt * Q_bias;

  float S = P[0] + R_measure;
  float K[2];
  K[0] = P[0] / S;
  K[1] = P[1] / S;

  float y = newAngle - angle;
  angle += K[0] * y;
  *bias += K[1] * y;

  float P00_temp = P[0];
  P[0] -= K[0] * P00_temp;
  P[0] -= K[1] * P[0];
  P[1] -= K[1] * P[1];

  return angle;
}

// --- Start erkennen ---
void detectLaunch(float accelZ) {
  if (accelZ > 1.5 && !rocketLaunched) {
    rocketLaunched = true;
    launchTime = millis();
    digitalWrite(LED_PIN, HIGH);
  }
}

// --- Servopositionen aktualisieren ---
void updateServos() {
  // Aktualisierung der Servos basierend auf Kalman-Filter-Werten
  servoXPos = constrain(map(angleX, -30, 30, 0, 180), 0, 180);
  servoYPos = constrain(map(angleY, -30, 30, 0, 180), 0, 180);

  servoX.write(servoXPos);
  servoY.write(servoYPos);
}

// --- Daten auf SD-Karte loggen ---
void logData() {
  logFile = SD.open("/sensor_data.txt", FILE_WRITE);
  if (logFile) {
    logFile.print(millis() - startTime);
    logFile.print(",");
    logFile.print(angleX);
    logFile.print(",");
    logFile.print(angleY);
    logFile.print(",");
    logFile.print(servoXPos);
    logFile.print(",");
    logFile.print(servoYPos);
    logFile.print(",");
    logFile.println(calculateAltitude());
    logFile.close();
  }
}

// --- Ton beim Start abspielen ---
void playMelody() {
  tone(BUZZER_PIN, 1000, 100);
  delay(100);
  tone(BUZZER_PIN, 1200, 100);
  delay(100);
  tone(BUZZER_PIN, 1500, 100);
  delay(100);
  tone(BUZZER_PIN, 2000, 100);
  delay(100);
  noTone(BUZZER_PIN);
}
