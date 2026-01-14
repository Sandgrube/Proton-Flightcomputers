//--- Bibliotheken für Sensoren und Kommunikation ---
// SendeScript
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <esp_now.h>
#include <WiFi.h>

//--- Sensorobjekte initialisieren ---
Adafruit_MPU6050 mpu;  // Beschleunigungssensor und Gyroskop
Adafruit_BMP085 bmp;   // Barometrischer Drucksensor

//--- Struktur der zu sendenden Sensordaten ---
typedef struct {
  float accelX, accelY, accelZ;  // Beschleunigungsdaten (X, Y, Z)
  float gyroX, gyroY, gyroZ;     // Gyroskopdaten (X, Y, Z)
  float pressure;                // Barometerdruck
  float altitude;                // Höhe
  float temperature;             // Temperatur
} SensorData;

SensorData sensorData;           // Erstellen der Datenstruktur für den Versand

//--- MAC-Adresse des Empfängers einstellen ---
uint8_t broadcastAddress[] = {0xD0, 0xEF, 0x76, 0xEF, 0xCC, 0x64};

//--- Callback für die Sendebestätigung ---
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Daten wurden gesendet: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Erfolgreich" : "Fehlgeschlagen");
}

//--- Setup: Initialisierung aller Komponenten ---
void setup() {
  Serial.begin(115200);          // Starten der seriellen Kommunikation

  // Initialisieren des I2C-Busses für die Sensoren
  Wire.begin();

  //--- MPU6050-Sensor initialisieren ---
  if (!mpu.begin()) {
    Serial.println("MPU6050 nicht gefunden!"); 
    while (1); // Bleibt hier, falls Sensor nicht gefunden wird
  }

  //--- BMP180-Sensor initialisieren ---
  if (!bmp.begin()) {
    Serial.println("BMP180 nicht gefunden!"); 
    while (1); // Bleibt hier, falls Sensor nicht gefunden wird
  }

  //--- WiFi im Station-Modus starten (erforderlich für ESP-NOW) ---
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  //--- ESP-NOW initialisieren ---
  if (esp_now_init() != ESP_OK) {
    Serial.println("Fehler bei ESP-NOW Initialisierung");
    return;
  }

  //--- Registrieren des Callback für gesendete Daten ---
  esp_now_register_send_cb(OnDataSent);

  //--- Hinzufügen des Empfängers mit MAC-Adresse ---
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;           // Kanal 0 (Standard)
  peerInfo.encrypt = false;       // Keine Verschlüsselung
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Fehler beim Hinzufügen des Peers");
    return;
  }
}

//--- Hauptprogramm: Daten erfassen und senden ---
void loop() {
  //--- MPU6050-Sensorwerte erfassen ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Zuordnen der Beschleunigungs- und Gyroskopwerte zur Datenstruktur
  sensorData.accelX = a.acceleration.x;
  sensorData.accelY = a.acceleration.y;
  sensorData.accelZ = a.acceleration.z;
  sensorData.gyroX = g.gyro.x;
  sensorData.gyroY = g.gyro.y;
  sensorData.gyroZ = g.gyro.z;
  sensorData.temperature = temp.temperature;  // Temperaturdaten hinzufügen

  //--- BMP180-Sensorwerte erfassen ---
  sensorData.pressure = bmp.readPressure();         // Barometerdruck
  sensorData.altitude = bmp.readAltitude();         // Höhe berechnet aus Druck

  //--- Senden der erfassten Sensordaten ---
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&sensorData, sizeof(sensorData));

  // Statusmeldung nach dem Sendeversuch
  if (result == ESP_OK) {
    Serial.println("Daten erfolgreich gesendet");
  } else {
    Serial.println("Fehler beim Senden der Daten");
  }

  delay(100); // Wiederholen alle 100 ms
}
