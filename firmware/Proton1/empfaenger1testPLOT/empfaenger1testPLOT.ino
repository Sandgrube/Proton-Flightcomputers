#include <esp_now.h>
#include <WiFi.h>

// Struktur für die empfangenen Daten
typedef struct {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float pressure;
  float altitude;
} SensorData;

SensorData sensorData;

// Callback, wenn Daten empfangen wurden
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&sensorData, incomingData, sizeof(sensorData));

  // Ausgabe der Gyroskop-Daten für den Serial Plotter
  Serial.print("gyroX:");
  Serial.print(sensorData.gyroX);
  Serial.print("\tgyroY:");
  Serial.print(sensorData.gyroY);
  Serial.print("\tgyroZ:");
  Serial.println(sensorData.gyroZ);  // Verwende println für den letzten Wert
}

void setup() {
  Serial.begin(115200);

  // WiFi Initialisierung
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // ESP-NOW Initialisierung
  if (esp_now_init() != ESP_OK) {
    Serial.println("Fehler bei ESP-NOW Initialisierung");
    return;
  }

  // Registrieren des Callback für empfangene Daten
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Der ESP32 bleibt im Leerlauf, Daten werden automatisch im Callback empfangen
}
