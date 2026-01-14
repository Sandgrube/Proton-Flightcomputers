//--- Bibliotheken für Sensoren, Kommunikation und Servo-Steuerung ---
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h> 
#include <SD.h>   
#include <SPI.h>  


//--- Fallschirm-Auslöse Konstanten---

// Schwellenwerte für Erkennung des Liftoffs und des Falls
const float liftoffThreshold = 2.00;  // Beschleunigung für Start (z.B. 2g über der Schwerkraft)
const float fallThreshold = 0.50;     // Beschleunigung für Fall (z.B. -1g)

// Variable zur Statusüberwachung
bool liftoffDetected = false;
float lastAccelerationY = 0.0;  // Letzte Beschleunigung auf der Y-Achse



// --- Pins für SD-Kartenleser ---
#define CS_PIN 15
#define MOSI_PIN 23
#define CLK_PIN 18
#define MISO_PIN 19


// --- Pin für den Buzzer ---
#define BUZZER_PIN 27


// --- Sensor Objekte erstellen ---
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;


// --- Struktur für die zu sendenden Daten ---
typedef struct {
  float accelX, accelY, accelZ;
  float gyroX, gyroZ, gyroY; // Nur X- und Z-Achse für den TVC
  float temperature;
  float pressure;
  float altitude;
  float pidOutputX;   
  float pidOutputZ;   // PID für Z-Achse
  float servoXPos;    
  float servoZPos;    // Servo für Z-Achse
} SensorData;

SensorData sensorData;           // Variable für die Sensordaten


// --- MAC-Adresse des Empfängers ---
uint8_t broadcastAddress[] = {0xD0, 0xEF, 0x76, 0xEF, 0xCC, 0x64};

// --- Servo Objekte für TVC-Mount ---
Servo servoX;
Servo servoZ;  // Servo für Z-Achse

Servo parachuteServo;



// --- PID-Parameter (können nach Bedarf angepasst werden) ---
float Kp = 5.0;  // Reduziertes Kp für sanftere Reaktionen
float Ki = 0.5;  // Niedriges Ki, um das Überschwingen zu reduzieren
float Kd = 1.0;  // Höheres Kd, um schneller auf Änderungen zu reagieren

// --- Variablen für PID-Steuerung der X- und Z-Achsen ---
float previousErrorX = 0, previousErrorZ = 0;
float integralX = 0, integralZ = 0;
float previousPIDOutputX = 0, previousPIDOutputZ = 0;

// --- Sollwerte für die Raketenstabilisierung ---
float targetAngleX = 0.0;
float targetAngleZ = 0.0;  // Sollwert für Z-Achse

// --- Variablen für Kalibrierung ---
float RateCalibrationRoll = 0.0, RateCalibrationPitch = 0.0, RateCalibrationYaw = 0.0;
int RateCalibrationNumber = 0;



// --- SD-Karten Initialisierung ---
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

// --- Callback für Sendestatus ---
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Daten wurden gesendet: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Erfolgreich" : "Fehlgeschlagen");
}

// --- Funktion zum Sammeln der Gyroskop-Signale ---
void gyro_signals() {
  Wire.beginTransmission(0x68); 
  Wire.write(0x1A); // CONFIG register
  Wire.write(0x05); // Set to 5 for slight signal dampening
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68); 
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x08); // Set gyro sensitivity to ±500°/s
  Wire.endTransmission();
  
  Wire.beginTransmission(0x68); 
  Wire.write(0x43); // GYRO_XOUT_H register (start of gyro data)
  Wire.endTransmission();
  
  Wire.requestFrom(0x68, 6); // Request 6 bytes of gyro data
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  
  sensorData.gyroX = (float)GyroX / 65.5 - RateCalibrationRoll;
  sensorData.gyroY = (float)GyroY / 65.5 - RateCalibrationPitch;
  sensorData.gyroZ = (float)GyroZ / 65.5 - RateCalibrationYaw;
}

// --- MPU6050 Kalibrierungsfunktion ---
void calibrateMPU6050() {
  RateCalibrationRoll = 0.0;
  RateCalibrationYaw = 0.0;
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += sensorData.gyroX;
    RateCalibrationYaw += sensorData.gyroZ;
    delay(1);
  }
  RateCalibrationRoll /= 2000.0;
  RateCalibrationYaw /= 2000.0;
  Serial.println("MPU6050 Kalibrierung abgeschlossen.");
}

// --- Anpassung für Temperaturkonvertierung ---
float leseTemperaturAusMPU6050() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  return bmp.readTemperature();
}

// --- Setup-Funktion zur Initialisierung ---
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

 
  
  Serial.println("Setup abgeschlossen. Warten auf Liftoff...");

  INITSERVO();


  calibrateMPU6050();  // Führe die Kalibrierung aus

  parachuteServo.write(0);

  startupSequence();

}
void INITSERVO() {

  servoX.attach(17);  
  servoZ.attach(5);  
  parachuteServo.attach(2); 
   
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

  // Direkte Temperaturmessung vom BMP180 verwenden
  sensorData.temperature = bmp.readTemperature();

  // Verwende BMP180, um den Druck und die Höhe zu erfassen
  sensorData.pressure = bmp.readPressure();
  sensorData.altitude = bmp.readAltitude(101325); // Meereshöhe kalibriert
  // Prüfen, ob Druck normal ist
  if(sensorData.pressure < 10000) { 
      Serial.println("Druckwert liegt unter 10000 Pa, bitte Sensor prüfen."); 
  }
  
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
    dataFile.print(sensorData.pidOutputZ); dataFile.print(",");
    dataFile.print(sensorData.servoXPos); dataFile.print(",");
    dataFile.println(sensorData.servoZPos);
    dataFile.close();
  } else {
    Serial.println("Fehler beim Öffnen der Datei auf der SD-Karte!");
  }
}

// --- PID-Berechnungsfunktion ---
void calculatePID() {
  // Berechne den Fehler
  float errorX = targetAngleX - sensorData.gyroX;
  float errorZ = targetAngleZ - sensorData.gyroZ;

  // Berechne Integral
  integralX += errorX;
  integralZ += errorZ;

  // Berechne den Derivat
  float derivativeX = errorX - previousErrorX;
  float derivativeZ = errorZ - previousErrorZ;

  // PID-Ausgabe
  sensorData.pidOutputX = Kp * errorX + Ki * integralX + Kd * derivativeX;
  sensorData.pidOutputZ = Kp * errorZ + Ki * integralZ + Kd * derivativeZ;

  // Begrenze die PID-Ausgabe, um Überschwinger zu verhindern
  sensorData.pidOutputX = constrain(sensorData.pidOutputX, -45, 45); // Begrenze PID-Ausgabe auf ±45
  sensorData.pidOutputZ = constrain(sensorData.pidOutputZ, -45, 45);

  // Speichere Fehler für den nächsten Durchgang
  previousErrorX = errorX;
  previousErrorZ = errorZ;
  previousPIDOutputX = sensorData.pidOutputX;
  previousPIDOutputZ = sensorData.pidOutputZ;

  // Servo-Bewegung auf Basis der PID-Ausgabe
  sensorData.servoXPos = map(sensorData.gyroX, -500, 500, 0, 180) + sensorData.pidOutputX;
  sensorData.servoZPos = map(sensorData.gyroZ, -500, 500, 0, 180) + sensorData.pidOutputZ;

  servoX.write(sensorData.servoXPos);
  servoZ.write(sensorData.servoZPos);
}

// ---Startup-Sequenz-Funktion---
void startupSequence() {

    // Parachute-Servo an Pin D2
  // Initialposition des Servos

  // Kurze Bewegungen der Servos und Buzzer-Töne
  for (int i = 0; i < 3; i++) {
    servoX.write(80);   // Servo leicht nach links
    servoZ.write(80);   // Servo leicht nach unten
    tone(BUZZER_PIN, 1000, 100); // Buzzer-Piepton bei 1 kHz für 100 ms
    delay(100);
    
    servoX.write(100);  // Servo leicht nach rechts
    servoZ.write(100);  // Servo leicht nach oben
    tone(BUZZER_PIN, 1500, 100); // Buzzer-Piepton bei 1.5 kHz für 100 ms
    delay(100);
  }
  
  // Servos in die Startposition zurücksetzen
  servoX.write(90);
  servoZ.write(90);
  noTone(BUZZER_PIN);   // Buzzer-Ton ausschalten
}

void loop() {
  static unsigned long lastDataSendTime = 0;
  static unsigned long lastDataLogTime = 0;
  static unsigned long lastUpdateNew = 0;

  unsigned long currentMillis = millis();

  

  if (currentMillis - lastUpdateNew >= 500){
    parachuteServo.attach(2);
    lastUpdateNew = currentMillis;
  }


  // Sende Sensordaten alle 100 ms (10 Mal pro Sekunde)
  if (currentMillis - lastDataSendTime >= 100) {
    sendeSensordaten();
    calculatePID();
    gyro_signals();      // Holen Sie sich die Gyroskopdaten für X und Z
    sensorData.temperature = leseTemperaturAusMPU6050();  // Temperatur vom Sensor auslesen
    lastDataSendTime = currentMillis;
  }

  // Logge Daten alle 25 ms (40 Mal pro Sekunde)
  if (currentMillis - lastDataLogTime >= 25) {
    loggeDaten();


    if (!liftoffDetected) {
    liftoffDetected = detectLiftoff();
    } else {
    deployingChutes();
    Serial.print("LOL");
      }
    
    lastDataLogTime = currentMillis;
  }

 
  delay(100);  // Leichte Verzögerung für Stabilität
}

// Funktion zur Erkennung des Liftoffs
bool detectLiftoff() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Ausgabe der Beschleunigungswerte zur Überprüfung
  Serial.print("Beschleunigung Y-Achse: ");
  Serial.println(a.acceleration.y);

  // Berechnung der Beschleunigung ohne Schwerkraft
  float accelY = a.acceleration.y - 9.81; // Schwerkraft subtrahieren

  // Ausgabe der korrigierten Y-Beschleunigung
  Serial.print("Korrigierte Y-Beschleunigung: ");
  Serial.println(accelY);

  // Prüfen auf schnelle Änderung in der Y-Beschleunigung
  if (abs(accelY) > liftoffThreshold && abs(accelY - lastAccelerationY) > 1.0) {  // Änderung der Beschleunigung
    Serial.println("Liftoff erkannt!");
    lastAccelerationY = accelY;  // Aktualisiere die letzte Beschleunigung
    delay(5000); // 5 Sekunden warten nach Starterkennung
    return true;
  }

  // Update der letzten Beschleunigung
  lastAccelerationY = accelY;
  return false;
}

// Funktion zum Auslösen des Fallschirms
void deployingChutes() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

   //Ausgabe der Beschleunigungswerte zur Überprüfung
  Serial.print("Beschleunigung Y-Achse: ");
  Serial.println(a.acceleration.y);

  // Prüfen, ob die Y-Beschleunigung einen Fall anzeigt
  if (a.acceleration.y < fallThreshold) {
    Serial.println("Fall erkannt! Fallschirm wird ausgelöst.");
    parachuteServo.write(90);  // Servo um 90 Grad drehen
    delay(2000);               // Position für 2 Sekunden halten
  }
}






