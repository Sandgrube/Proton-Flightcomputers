#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h> // Korrektur hier
#include <esp_now.h>
#include <WiFi.h>

#define BUZZER_PIN 27
#define CS_PIN 15
#define MOSI_PIN 23
#define CLK_PIN 18
#define MISO_PIN 19
#define LED_PIN 34  // LED auf Pin D18

// Empfänger-MAC-Adresse
uint8_t receiverMac[] = {0xD0, 0xEF, 0x76, 0xEF, 0xCC, 0x64};
int counter = 0; //sendecode

MPU6050 mpu;
Servo servoX; // Servo für X-Achse (Pin 17)
Servo servoY; // Servo für Y-Achse (Pin 5)

// BMP180 Initialisierung
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(); // Korrektur hier

float Q_angle = 0.001;  // Prozessrauschen (Q-Winkel)
float Q_bias = 0.003;   // Prozessrauschen (Q-Bias)
float R_measure = 0.03; // Messrauschen (R-Messung)

float angleX = 0, biasX = 0, rateX = 0;
float angleY = 0, biasY = 0, rateY = 0;

float P[2][2] = {{0, 0}, {0, 0}}; // Fehler-Kovarianzmatrix für X
float P_Y[2][2] = {{0, 0}, {0, 0}}; // Fehler-Kovarianzmatrix für Y

unsigned long lastTime;
unsigned long startTime;
unsigned long previousMillis = 0;
const long interval = 25; // 25 ms -> 40 mal pro Sekunde
File logFile;

bool rocketLaunched = false;  // Variable, um den Raketenstart zu verfolgen
unsigned long launchTime; // Zeit, zu der die Rakete abgehoben ist

// Hinzufügen von Variablen für Servo-Positionen
int servoXPos = 90; // Standardposition für Servo X
int servoYPos = 90; // Standardposition für Servo Y

void setup() {

    //ESPNOWTEST
    // WiFi in den Modus für ESP-NOW setzen
  WiFi.mode(WIFI_STA);
  // ESP-NOW initialisieren
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

  Serial.begin(115200);

  // I2C Initialisierung
  Wire.begin();

  char message[32];
  sprintf(message, "Rakete Startbereit");

  // Nachricht senden
  esp_now_send(receiverMac, (uint8_t *)message, strlen(message));


  
  // MPU6050 Initialisierung
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 nicht gefunden!");
    while (1);
  }

  // BMP180 Initialisierung
  if (!bmp.begin()) { // Korrektur hier
    Serial.println("BMP180 nicht gefunden!");
    while (1);
  }

  // SPI mit den richtigen Pins initialisieren
  SPI.begin(CLK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

  // SD-Karte initialisieren
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

  // Buzzer initialisieren
  pinMode(BUZZER_PIN, OUTPUT);
  
  // LED Pin initialisieren
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // LED zu Beginn ausschalten

  // Servos initialisieren
  servoX.attach(17); // Servo auf Pin 17
  servoY.attach(5);  // Servo auf Pin 5

  // Setze Servos auf die neutrale Position
  servoX.write(servoXPos);
  servoY.write(servoYPos);
  delay(1000); // Kurze Pause, um die Position zu stabilisieren

  // Startzeit für Logging
  startTime = millis();

  // Eine kurze Pause vor dem Abspielen der Töne
  delay(1000);

  // Töne abspielen und Servos bewegen
  playMelody();

  lastTime = micros(); // Startzeit für die Gyro-Integration
}

void loop() {

  char message[64];  // Erhöhe den Puffer für die Nachricht

// Berechnung der Höhe basierend auf dem Drucksensor BMP180
  float seaLevelPressure = 1013.25;  // Standard-Meeresspiegel-Luftdruck in hPa
  float pressure;
  bmp.getPressure(&pressure);  // Druck in Pa (getPressure erwartet einen Zeiger)

// Berechnung der Höhe in Metern
  float altitude = 44330.0 * (1.0 - pow(pressure / (seaLevelPressure * 100.0), 0.1903));  

// Erstelle die Nachricht, die den Counter und die Höhe enthält
  sprintf(message, "Hallo %d, Höhe: %.2f m", counter, altitude);

// Nachricht senden
  esp_now_send(receiverMac, (uint8_t *)message, strlen(message));  // Nachricht wird gesendet


  
  counter++;  // Zähler erhöhen

  delay(100);  // 10 Nachrichten pro Sekunde 

  unsigned long currentMillis = millis();

  // Alle 25 ms (40 mal pro Sekunde)
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Gyroskop- und Beschleunigungsdaten lesen
    int16_t rawAccelX, rawAccelY, rawAccelZ;
    int16_t rawGyroX, rawGyroY, rawGyroZ;

    mpu.getAcceleration(&rawAccelX, &rawAccelY, &rawAccelZ);
    mpu.getRotation(&rawGyroX, &rawGyroY, &rawGyroZ);

    // Umwandeln der rohen Sensordaten in "echte" Werte
    float accelX = rawAccelX / 16384.0; // 1g = 16384 LSB für den MPU6050
    float accelY = rawAccelY / 16384.0;
    float accelZ = rawAccelZ / 16384.0;

    float gyroX = rawGyroX / 131.0; // Gyro-Werte in Grad/s umwandeln
    float gyroY = rawGyroY / 131.0;

    // Zeit berechnen (in Sekunden) seit letztem Update
    unsigned long dt = micros() - lastTime;
    lastTime = micros();

    // Berechne die Neigung (Roll/Pitch) aus den Beschleunigungswerten
    float accelAngleX = atan2(accelY, accelZ) * 180 / PI;
    float accelAngleY = atan2(-accelX, accelZ) * 180 / PI;

    // Kalman-Filter anwenden
    angleX = kalmanFilter(accelAngleX, gyroX, &biasX, &P[0][0], dt / 1000000.0);
    angleY = kalmanFilter(accelAngleY, gyroY, &biasY, &P_Y[0][0], dt / 1000000.0);

    // Check if the rocket has launched (acceleration on Z-axis > threshold)
    if (!rocketLaunched && accelZ > 1.5) {  // Threshold value for launch detection
      rocketLaunched = true;
      launchTime = millis(); // Startzeit speichern
      digitalWrite(LED_PIN, HIGH); // LED einschalten
      Serial.println("Rakete hat abgehoben!");
    }

    // Servo-Positionen berechnen, nur wenn 0.5 Sekunden nach dem Abheben vergangen sind
    if (rocketLaunched && (millis() - launchTime > 500)) {
      // Servo-Positionen berechnen
      servoXPos = map(angleX, -6, 6, 60, 120);
      servoYPos = map(angleY, -6, 6, 60, 120);

      servoXPos = constrain(servoXPos, 60, 120);
      servoYPos = constrain(servoYPos, 60, 120);

      // Servos bewegen
      servoX.write(servoXPos);
      servoY.write(servoYPos);
    }

    // Luftdruck lesen
    float pressure;
    sensors_event_t event;
    bmp.getEvent(&event);
    if (event.pressure) {
      pressure = event.pressure; // Druck in hPa
    } else {
      pressure = 0; // Falls der Druck nicht gelesen werden kann
    }

    // Daten in die Datei schreiben
    logFile = SD.open("/sensor_data.txt", FILE_APPEND);
    if (logFile) {
      logFile.print(millis() - startTime);
      logFile.print(", ");
      logFile.print(angleX);
      logFile.print(", ");
      logFile.print(angleY);
      logFile.print(", ");
      logFile.print(servoXPos); // Verwendung der deklarierten Variablen
      logFile.print(", ");
      logFile.print(servoYPos); // Verwendung der deklarierten Variablen
      logFile.print(", ");
      logFile.println(pressure);
      logFile.close();
    }

    // Ausgabe auf der seriellen Konsole
    Serial.print("Zeit: "); Serial.print(millis() - startTime);
    Serial.print(", WinkelX: "); Serial.print(angleX);
    Serial.print(", WinkelY: "); Serial.print(angleY);
    Serial.print(", ServoX: "); Serial.print(servoXPos);
    Serial.print(", ServoY: "); Serial.print(servoYPos);
    Serial.print(", Luftdruck: "); Serial.println(pressure);
  }
}

// Kalman-Filter-Funktion für einen einzelnen Winkel
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

// Funktion zur Wiedergabe der Melodie und Servo-Bewegung
void playMelody() {
  int frequency = 554; // Frequenz für C#5

  for (int i = 0; i < 3; i++) {
    tone(BUZZER_PIN, frequency, 500); // 500ms für jeden Ton
    moveServosDuringBuzzer();
    delay(500); // kurze Pause zwischen den Tönen
  }

  delay(2000); // 2 Sekunden Pause nach den Tönen
}

// Funktion, um die Servos während des Buzzers zu bewegen
void moveServosDuringBuzzer() {
  // Servo X auf 6 Grad
  servoX.write(84); // 90 - 6
  servoY.write(84);
  delay(500);
  
  // Servo X auf -6 Grad
  servoX.write(96);
  servoY.write(96); // 90 + 6
  delay(500);

  // Zurück zur mittleren Position
  servoX.write(90);
}


