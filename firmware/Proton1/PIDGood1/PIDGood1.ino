//--- Bibliotheken für Sensoren, Kommunikation und Servo-Steuerung ---
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>  // ESP32Servo Bibliothek verwenden

//--- Sensor Objekte erstellen ---
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;

//--- Struktur für die zu sendenden Daten ---
typedef struct {
  float accelX, accelY, accelZ;  // Beschleunigung (X, Y, Z)
  float gyroX, gyroY, gyroZ;     // Gyroskopdaten (X, Y, Z)
  float pressure;                // Barometerdruck
  float altitude;                // Höhe
  float temperature;             // Temperatur
} SensorData;

SensorData sensorData;           // Variable für die Sensordaten

//--- MAC-Adresse des Empfängers ---
uint8_t broadcastAddress[] = {0xD0, 0xEF, 0x76, 0xEF, 0xCC, 0x64};

//--- Servo Objekte für TVC-Mount ---
Servo servoX;
Servo servoY;

//--- PID-Parameter (können nach Bedarf angepasst werden) ---
float Kp = 5.0;   // Proportional
float Ki = 1.0;   // Integral
float Kd = 0.5;   // Derivativ

//--- Variablen für PID-Steuerung der X- und Y-Achsen ---
float previousErrorX = 0, previousErrorY = 0;
float integralX = 0, integralY = 0;

//--- Sollwerte für die Raketenstabilisierung ---
float targetAngleX = 0.0;
float targetAngleY = 0.0;

//--- Callback für Sendestatus ---
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Daten wurden gesendet: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Erfolgreich" : "Fehlgeschlagen");
}

//--- Setup-Funktion zur Initialisierung ---
void setup() {
  Serial.begin(115200);

  // Initialisierung der I2C-Schnittstelle
  Wire.begin();

  //--- MPU6050 Initialisierung ---
  if (!mpu.begin()) {
    Serial.println("MPU6050 nicht gefunden!");
    while (1);
  }

  //--- BMP180 Initialisierung ---
  if (!bmp.begin()) {
    Serial.println("BMP180 nicht gefunden!");
    while (1);
  }

  //--- WiFi und ESP-NOW Initialisierung ---
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Fehler bei ESP-NOW Initialisierung");
    return;
  }

  // Registrieren des Callback für gesendete Daten
  esp_now_register_send_cb(OnDataSent);

  // Empfänger hinzufügen
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Fehler beim Hinzufügen des Peers");
    return;
  }

  //--- Servo-Pins initialisieren ---
  servoX.attach(17);  // Servo für X-Achse
  servoY.attach(5);   // Servo für Y-Achse
}

//--- Funktion zum Sammeln und Senden der Sensordaten ---
void sendeSensordaten() {
  //--- MPU6050 Werte auslesen ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  sensorData.accelX = a.acceleration.x;
  sensorData.accelY = a.acceleration.y;
  sensorData.accelZ = a.acceleration.z;
  sensorData.gyroX = g.gyro.x;
  sensorData.gyroY = g.gyro.y;
  sensorData.gyroZ = g.gyro.z;
  sensorData.temperature = temp.temperature;

  //--- BMP180 Werte auslesen ---
  sensorData.pressure = bmp.readPressure();
  sensorData.altitude = bmp.readAltitude();

  //--- Senden der Daten über ESP-NOW ---
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&sensorData, sizeof(sensorData));
  if (result == ESP_OK) {
    Serial.println("Daten erfolgreich gesendet");
  } else {
    Serial.println("Fehler beim Senden der Daten");
  }
}

//--- PID-Steuerungsfunktion für TVC-Mount ---
void PID_Controller() {
  //--- Fehler berechnen (Sollwinkel - aktueller Winkel aus Gyro) ---
  float errorX = targetAngleX - sensorData.gyroX;
  float errorY = targetAngleY - sensorData.gyroY;

  //--- Proportionalterm berechnen ---
  float PoutX = Kp * errorX;
  float PoutY = Kp * errorY;

  //--- Integralterm berechnen ---
  integralX += errorX;
  integralY += errorY;
  float IoutX = Ki * integralX;
  float IoutY = Ki * integralY;

  //--- Derivativterm berechnen ---
  float derivativeX = errorX - previousErrorX;
  float derivativeY = errorY - previousErrorY;
  float DoutX = Kd * derivativeX;
  float DoutY = Kd * derivativeY;

  //--- Gesamtausgabe berechnen ---
  float outputX = PoutX + IoutX + DoutX;
  float outputY = PoutY + IoutY + DoutY;

  //--- Servo-Positionen entsprechend den Ausgaben setzen ---
  servoX.write(90 + outputX);  // 90 als neutraler Mittelwert
  servoY.write(90 + outputY);

  //--- Aktuelle Fehler speichern ---
  previousErrorX = errorX;
  previousErrorY = errorY;
}

void loop() {
  //--- Sensordaten sammeln und senden ---
  sendeSensordaten();

  //--- PID-Steuerung für TVC-Mount durchführen ---
  PID_Controller();

  delay(100); // Alle 100 ms wiederholen
}
