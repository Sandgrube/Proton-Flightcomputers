#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

const float ACCEL_THRESHOLD = 1.3;               // Schwellwert in g
const unsigned long REQUIRED_DURATION_MS = 10;  // 0.1 Sekunden

bool aboveThreshold = false;
unsigned long startTime = 0;
bool launched = false;

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Standard: SDA=21, SCL=22 beim ESP32

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 nicht verbunden!");
    while (1);
  }

  Serial.println("MPU6050 bereit.");
}

void loop() {
  if (launched) return; // Wenn gestartet → nichts mehr tun

  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  float accelZ = az / 16384.0; // Umrechnung in g

  // → Debug-Ausgabe
  Serial.print("Z-Beschleunigung: ");
  Serial.print(accelZ);
  Serial.println(" g");

  if (accelZ >= ACCEL_THRESHOLD) {
    if (!aboveThreshold) {
      startTime = millis();
      aboveThreshold = true;
    } else if (millis() - startTime >= REQUIRED_DURATION_MS) {
      launched = true;
      Serial.println("Launched!");
    }
  } else {
    aboveThreshold = false;
  }

  delay(10); // 100 Hz
}
