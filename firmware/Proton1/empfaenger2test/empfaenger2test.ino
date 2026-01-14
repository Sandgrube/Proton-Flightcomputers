//--- Bibliotheken für Kommunikation ---
#include <esp_now.h>
#include <WiFi.h>

//--- Struktur für die empfangenen Daten (muss mit der Senderstruktur übereinstimmen) ---
typedef struct {
  float accelX, accelY, accelZ;  // Beschleunigung (X, Y, Z)
  float gyroX, gyroY, gyroZ;     // Gyroskopdaten (X, Y, Z)
  float pressure;                // Barometerdruck
  float altitude;                // Höhe
  float temperature;             // Temperatur
} SensorData;

SensorData sensorData;           // Variable für die empfangenen Sensordaten

//--- Callback-Funktion bei Empfang neuer Daten ---
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  memcpy(&sensorData, incomingData, sizeof(sensorData));  // Kopieren der empfangenen Daten in sensorData

  //--- Ausgabe der empfangenen Sensordaten ---
  Serial.print("Beschleunigung [X, Y, Z]: ");
  Serial.print(sensorData.accelX); Serial.print(", ");
  Serial.print(sensorData.accelY); Serial.print(", ");
  Serial.println(sensorData.accelZ);

  Serial.print("Gyroskop [X, Y, Z]: ");
  Serial.print(sensorData.gyroX); Serial.print(", ");
  Serial.print(sensorData.gyroY); Serial.print(", ");
  Serial.println(sensorData.gyroZ);

  Serial.print("Druck: ");
  Serial.print(sensorData.pressure);
  Serial.println(" Pa");

  Serial.print("Höhe: ");
  Serial.print(sensorData.altitude);
  Serial.println(" m");

  Serial.print("Temperatur: ");
  Serial.print(sensorData.temperature);
  Serial.println(" °C");
}

//--- Setup-Funktion zur Initialisierung des Empfängers ---
void setup() {
  Serial.begin(115200);  // Start der seriellen Kommunikation für Debug-Ausgaben

  //--- WiFi im Station-Modus starten (erforderlich für ESP-NOW) ---
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  //--- ESP-NOW initialisieren ---
  if (esp_now_init() != ESP_OK) {
    Serial.println("Fehler bei ESP-NOW Initialisierung");
    return;
  }

  //--- Registrieren des Callbacks für empfangene Daten ---
  esp_now_register_recv_cb(OnDataRecv);
}

//--- Hauptprogramm im Leerlauf lassen ---
// Der ESP32 bleibt im Leerlauf, Daten werden automatisch im Callback empfangen
void loop() {
}
