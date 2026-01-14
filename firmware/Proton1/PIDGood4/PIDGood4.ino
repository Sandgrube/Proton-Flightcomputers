//--- Bibliotheken für Sensoren, Kommunikation und Servo-Steuerung ---
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h> 
#include <SD.h>   
#include <SPI.h>  

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
  float pidOutputX;   
  float pidOutputY;   
  float servoXPos;    
  float servoYPos;    
} SensorData;

SensorData sensorData;           // Variable für die Sensordaten

//--- MAC-Adresse des Empfängers ---
uint8_t broadcastAddress[] = {0xD0, 0xEF, 0x76, 0xEF, 0xCC, 0x64};

//--- Servo Objekte für TVC-Mount ---
Servo servoX;
Servo servoY;

//--- PID-Parameter (können nach Bedarf angepasst werden) ---
float Kp = 5.0;  
float Ki = 1.0;  
float Kd = 0.5;  

//--- Variablen für PID-Steuerung der X- und Y-Achsen ---
float previousErrorX = 0, previousErrorY = 0;
float integralX = 0, integralY = 0;

//--- Sollwerte für die Raketenstabilisierung ---
float targetAngleX = 0.0;
float targetAngleY = 0.0;

//--- SD-Karten Initialisierung ---
bool initSD() {
  if (!SD.begin(CS_PIN)) {
    Serial.println("SD-Karteninitialisierung fehlgeschlagen!");
    return false;
  }
  File dataFile = SD.open("/SensorUndMessdaten.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println("Eintrag,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Temperature,Pressure,Altitude,PID_X,PID_Y,ServoX,ServoY");
    dataFile.close();
  } else {
    Serial.println("Fehler beim Erstellen der Datei auf der SD-Karte");
    return false;
  }
  return true;
}

//--- Callback für Sendestatus ---
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Daten wurden gesendet: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Erfolgreich" : "Fehlgeschlagen");
}

//--- Setup-Funktion zur Initialisierung ---
void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("MPU6050 nicht gefunden!");
    while (1);
  }

  if (!bmp.begin()) {
    Serial.println("BMP180 nicht gefunden!");
    while (1);
  }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Fehler bei ESP-NOW Initialisierung");
    return;
  }
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Fehler beim Hinzufügen des Peers");
    return;
  }

  if (!initSD()) {
    Serial.println("SD-Karte konnte nicht initialisiert werden.");
  }

  servoX.attach(17);  
  servoY.attach(5);   
}

//--- Funktion zum Sammeln und Senden der Sensordaten ---
void sendeSensordaten() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  sensorData.accelX = a.acceleration.x;
  sensorData.accelY = a.acceleration.y;
  sensorData.accelZ = a.acceleration.z;
  sensorData.gyroX = g.gyro.x;
  sensorData.gyroY = g.gyro.y;
  sensorData.gyroZ = g.gyro.z;
  
  // Temperatur umwandeln:  z.B. 270 = 27.0 °C
  sensorData.temperature = temp.temperature; // Temperatur aus MPU6050 (korrigiert)
  
  // Verwende den BMP180, um den Druck und die Höhe zu erfassen
  sensorData.pressure = bmp.readPressure();
  sensorData.altitude = bmp.readAltitude();

  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&sensorData, sizeof(sensorData));
  Serial.println(result == ESP_OK ? "Daten erfolgreich gesendet" : "Fehler beim Senden der Daten");
}


// --- Funktion zum Loggen der Daten auf die SD-Karte ---
void loggeDaten() {
  static int eintragNummer = 1;  
  File dataFile = SD.open("/SensorUndMessdaten.txt", FILE_APPEND);
  if (dataFile) {
    dataFile.print(eintragNummer++); dataFile.print(",");
    dataFile.print(sensorData.accelX); dataFile.print(",");
    dataFile.print(sensorData.accelY); dataFile.print(",");
    dataFile.print(sensorData.accelZ); dataFile.print(",");
    dataFile.print(sensorData.gyroX); dataFile.print(",");
    dataFile.print(sensorData.gyroY); dataFile.print(",");
    dataFile.print(sensorData.gyroZ); dataFile.print(",");
    dataFile.print(sensorData.temperature); dataFile.print(",");
    dataFile.print(sensorData.pressure); dataFile.print(",");
    dataFile.print(sensorData.altitude); dataFile.print(",");
    dataFile.print(sensorData.pidOutputX); dataFile.print(",");
    dataFile.print(sensorData.pidOutputY); dataFile.print(",");
    dataFile.print(sensorData.servoXPos); dataFile.print(",");
    dataFile.println(sensorData.servoYPos);
    dataFile.close();
  } else {
    Serial.println("Fehler beim Öffnen der Datei auf der SD-Karte!");
  }
}

//--- PID-Steuerungsfunktion für TVC-Mount ---
void PID_Controller() {
  float errorX = targetAngleX - sensorData.gyroX;
  float errorY = targetAngleY - sensorData.gyroY;

  float PoutX = Kp * errorX;
  float PoutY = Kp * errorY;

  integralX += errorX;
  integralY += errorY;
  float IoutX = Ki * integralX;
  float IoutY = Ki * integralY;

  float derivativeX = errorX - previousErrorX;
  float derivativeY = errorY - previousErrorY;
  float DoutX = Kd * derivativeX;
  float DoutY = Kd * derivativeY;

  float outputX = PoutX + IoutX + DoutX;
  float outputY = PoutY + IoutY + DoutY;

  sensorData.pidOutputX = outputX;
  sensorData.pidOutputY = outputY;

  sensorData.servoXPos = 90 + outputX;
  sensorData.servoYPos = 90 + outputY;

  servoX.write(sensorData.servoXPos);
  servoY.write(sensorData.servoYPos);

  previousErrorX = errorX;
  previousErrorY = errorY;
}

void loop() {
  static unsigned long lastDataSendTime = 0;
  static unsigned long lastDataLogTime = 0;

  unsigned long currentMillis = millis();

  // Sende Sensordaten alle 100 ms (10 Mal pro Sekunde)
  if (currentMillis - lastDataSendTime >= 100) {
    sendeSensordaten();
    PID_Controller();
    lastDataSendTime = currentMillis;
  }

  // Logge Daten alle 25 ms (40 Mal pro Sekunde)
  if (currentMillis - lastDataLogTime >= 25) {
    loggeDaten();
    lastDataLogTime = currentMillis;
  }
}

