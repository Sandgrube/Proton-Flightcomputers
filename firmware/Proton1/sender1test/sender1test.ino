#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <esp_now.h>
#include <WiFi.h>

// Sensor Objekte
Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;

// Struktur für die zu sendenden Daten
typedef struct {
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float pressure;
  float altitude;
} SensorData;

SensorData sensorData;

// MAC-Adresse des Empfängers (einzusetzen)
uint8_t broadcastAddress[] = {0xD0, 0xEF, 0x76, 0xEF, 0xCC, 0x64};

// Callback, um zu überprüfen, ob Daten gesendet wurden
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Daten wurden gesendet: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Erfolgreich" : "Fehlgeschlagen");
}

void setup() {
  Serial.begin(115200);

  // Initialize I2C
  Wire.begin();

  // MPU6050 Initialisierung
  if (!mpu.begin()) {
    Serial.println("MPU6050 nicht gefunden!");
    while (1);
  }
  
  // BMP180 Initialisierung
  if (!bmp.begin()) {
    Serial.println("BMP180 nicht gefunden!");
    while (1);
  }

  // WiFi Initialisierung (ESP-NOW erfordert aktiviertes WiFi)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // ESP-NOW Initialisierung
  if (esp_now_init() != ESP_OK) {
    Serial.println("Fehler bei ESP-NOW Initialisierung");
    return;
  }

  // Registrieren des Callback für gesendete Daten
  esp_now_register_send_cb(OnDataSent);

  // Fügen Sie den Empfänger hinzu
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Fehler beim Hinzufügen des Peers");
    return;
  }
}

void loop() {
  // MPU6050 Werte auslesen
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  sensorData.accelX = a.acceleration.x;
  sensorData.accelY = a.acceleration.y;
  sensorData.accelZ = a.acceleration.z;
  sensorData.gyroX = g.gyro.x;
  sensorData.gyroY = g.gyro.y;
  sensorData.gyroZ = g.gyro.z;

  // BMP180 Werte auslesen
  sensorData.pressure = bmp.readPressure();
  sensorData.altitude = bmp.readAltitude();

  // Senden der Daten an den Empfänger
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&sensorData, sizeof(sensorData));

  if (result == ESP_OK) {
    Serial.println("Daten erfolgreich gesendet");
  } else {
    Serial.println("Fehler beim Senden der Daten");
  }

  delay(100); // Alle .1 Sekunde senden
}
