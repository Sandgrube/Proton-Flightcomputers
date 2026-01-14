#include <esp_now.h>
#include <WiFi.h>

// Struktur für die empfangenen Daten (muss mit der Struktur im Sender übereinstimmen)
typedef struct {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float pressure;
  float altitude;
} SensorData;

SensorData sensorData;

// Callback, wenn Daten empfangen wurden (angepasst an neue Bibliotheksversion)
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&sensorData, incomingData, sizeof(sensorData));

  // Ausgabe der empfangenen Daten
  Serial.print("Beschleunigung [X, Y, Z]: ");
  Serial.print(sensorData.accelX); Serial.print(", ");
  Serial.print(sensorData.accelY); Serial.print(", ");
  Serial.println(sensorData.accelZ);

  Serial.print("Gyro [X, Y, Z]: ");
  Serial.print(sensorData.gyroX); Serial.print(", ");
  Serial.print(sensorData.gyroY); Serial.print(", ");
  Serial.println(sensorData.gyroZ);

  Serial.print("Druck: ");
  Serial.print(sensorData.pressure);
  Serial.println(" Pa");

  Serial.print("Höhe: ");
  Serial.print(sensorData.altitude);
  Serial.println(" m");
}

void setup() {
  Serial.begin(115200);

  // WiFi Initialisierung (ESP-NOW erfordert aktiviertes WiFi)
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
