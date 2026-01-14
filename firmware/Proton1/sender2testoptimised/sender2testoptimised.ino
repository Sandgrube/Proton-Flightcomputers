//--- Bibliotheken für Sensoren und Kommunikation ---
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <esp_now.h>
#include <WiFi.h>

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

// MAC-Adresse des Empfängers
uint8_t broadcastAddress[] = {0xD0, 0xEF, 0x76, 0xEF, 0xCC, 0x64};

//--- Callback für Sendestatus ---
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Daten wurden gesendet: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Erfolgreich" : "Fehlgeschlagen");
}

//--- Setup-Funktion zur Initialisierung ---
void setup() {
  Serial.begin(115200);  // Start der seriellen Kommunikation

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
  sensorData.temperature = temp.temperature;  // Temperatur hinzufügen

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

void loop() {
  // Rufe die Funktion auf, um Sensordaten zu sammeln und zu senden
  sendeSensordaten();

  delay(100); // Sende alle 100 ms
}
