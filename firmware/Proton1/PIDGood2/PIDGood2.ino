//--- Bibliotheken für Sensoren, Kommunikation und Servo-Steuerung ---
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>  // ESP32Servo Bibliothek verwenden
#include <SD.h>   // Bibliothek für SD-Karten
#include <SPI.h>  // SPI-Bibliothek für SD-Kommunikation

// --- Pins für SD-Kartenleser ---
#define CS_PIN 15
#define MOSI_PIN 23
#define CLK_PIN 18
#define MISO_PIN 19

//--- Sensor Objekte erstellen ---
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;

//--- Struktur für die zu sendenden Daten ---
typedef struct {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float temperature;
  float pressure;
  float altitude;
  float pidOutputX;   // X-Achse PID-Ausgabe
  float pidOutputY;   // Y-Achse PID-Ausgabe
  float servoXPos;    // Aktuelle Servo-Position auf der X-Achse
  float servoYPos;    // Aktuelle Servo-Position auf der Y-Achse
} SensorData;


SensorData sensorData;           // Variable für die Sensordaten

//--- MAC-Adresse des Empfängers ---
uint8_t broadcastAddress[] = {0xD0, 0xEF, 0x76, 0xEF, 0xCC, 0x64};

// Zähler für die Eintragsnummer für die MicroSd Karten Logik
unsigned long entryNumber = 1;

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

  initSD(); // ---SD-Karte initialisieren---

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
// --- SD-Karte initialisieren ---
void initSD() {
  if (!SD.begin(CS_PIN)) {
    Serial.println("SD-Karteninitialisierung fehlgeschlagen!");
    return;
  }

  // Datei erstellen und Kopfzeile hinzufügen, falls sie noch nicht existiert
  File dataFile = SD.open("/SensorUndMessdaten.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println("Eintrag,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Temperature,Pressure,Altitude,PID_X,PID_Y,ServoX,ServoY");
    dataFile.close();
  } else {
    Serial.println("Fehler beim Erstellen der Datei auf der SD-Karte");
  }
}

// --- Funktion zum Loggen der Daten auf die SD-Karte ---
void loggeDaten() {
  static int eintragNummer = 1;  // Zähler für die Eintragsnummer

  File dataFile = SD.open("/SensorUndMessdaten.txt", FILE_APPEND);
  if (dataFile) {
    // Nummer
    dataFile.print(eintragNummer); dataFile.print(",");

    // Beschleunigungsdaten
    dataFile.print(sensorData.accelX); dataFile.print(",");
    dataFile.print(sensorData.accelY); dataFile.print(",");
    dataFile.print(sensorData.accelZ); dataFile.print(",");

    // Gyroskopdaten
    dataFile.print(sensorData.gyroX); dataFile.print(",");
    dataFile.print(sensorData.gyroY); dataFile.print(",");
    dataFile.print(sensorData.gyroZ); dataFile.print(",");

    // Temperatur
    dataFile.print(sensorData.temperature); dataFile.print(",");

    // Druck und Höhe
    dataFile.print(sensorData.pressure); dataFile.print(",");
    dataFile.print(sensorData.altitude); dataFile.print(",");

    // PID-Ausgaben
    dataFile.print(sensorData.pidOutputX); dataFile.print(",");
    dataFile.print(sensorData.pidOutputY); dataFile.print(",");

    // Servo-Positionen
    dataFile.print(sensorData.servoXPos); dataFile.print(",");
    dataFile.println(sensorData.servoYPos);

    dataFile.close();
    eintragNummer++;
  } else {
    Serial.println("Fehler beim Öffnen der Datei auf der SD-Karte!");
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
    //--- Sensordaten auf SD-Karte loggen ---
  loggeDaten();

  delay(25); // Alle 25 ms (40 Hz) wiederholen
}
