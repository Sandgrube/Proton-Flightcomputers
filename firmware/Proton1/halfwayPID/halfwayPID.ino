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

// ---Pin für den Buzzer---
#define BUZZER_PIN 27

//--- Sensor Objekte erstellen ---
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;

//--- Servo Objekt für Fallschirm --- 
Servo servoFallschirm;

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

//--- Anpassung für Temperaturkonvertierung ---
float leseTemperaturAusMPU6050() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // MPU6050 gibt die Rohtemperatur zurück, für unsere Berechnung nur um +/- Abweichung zu sehen
  float rohTemperatur = temp.temperature;
  
  // Für präzise Werte BMP180 verwenden
  return bmp.readTemperature();
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

  servoX.attach(17);  //TVC
  servoY.attach(5);   //TVC

//---Fallschirm Scheisse---
  servoFallschirm.attach(2);  // Servo an Pin D2 verbinden
  servoFallschirm.write(0);   // Servo auf Ausgangsposition setzen

  // ---Start-Up-Sequenz---
  startupSequence();
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

// ---Startup-Sequenz-Funktion---
void startupSequence() {
  // Kurze Bewegungen der Servos und Buzzer-Töne
  for (int i = 0; i < 3; i++) {
    servoX.write(80);   // Servo leicht nach links
    servoY.write(80);   // Servo leicht nach unten
    tone(BUZZER_PIN, 1000, 100); // Buzzer-Piepton bei 1 kHz für 100 ms
    delay(100);
    
    servoX.write(100);  // Servo leicht nach rechts
    servoY.write(100);  // Servo leicht nach oben
    tone(BUZZER_PIN, 1500, 100); // Buzzer-Piepton bei 1.5 kHz für 100 ms
    delay(100);
  }
  
  // Servos in die Startposition zurücksetzen
  servoX.write(90);
  servoY.write(90);
  noTone(BUZZER_PIN);   // Buzzer-Ton ausschalten
}

// --- Funktion zur Auslösung des Fallschirms ---
void FallschirmDeploy() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Überprüfen auf negative Beschleunigung
  if (a.acceleration.y < -5.0) {  // Schwellenwert für negative Beschleunigung, anpassbar je nach Bedarf
    Serial.println("Fallschirm-Bedingung erkannt! Fallschirm wird ausgelöst.");
    servoFallschirm.write(180);  // Servo 90 Grad nach rechts drehen
    delay(1000);  // kurze Verzögerung für das Debugging
  } else {
    Serial.print("Y-Beschleunigung: ");
    Serial.println(a.acceleration.y);  // Ausgabe der Y-Beschleunigung für Debugging
  }
}



void loop() {
  static unsigned long lastDataSendTime = 0;
  static unsigned long lastDataLogTime = 0;

  unsigned long currentMillis = millis();

  // Sende Sensordaten alle 100 ms (10 Mal pro Sekunde)
  if (currentMillis - lastDataSendTime >= 100) {
    sendeSensordaten();
    PID_Controller();
    FallschirmDeploy();
    lastDataSendTime = currentMillis;
  }

  // Logge Daten alle 25 ms (40 Mal pro Sekunde)
  if (currentMillis - lastDataLogTime >= 25) {
    loggeDaten();
    lastDataLogTime = currentMillis;
  }
}
