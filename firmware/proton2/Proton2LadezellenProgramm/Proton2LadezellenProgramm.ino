#include <Arduino.h>
#include <HX711.h>
#include <SPI.h>
#include <SD.h>

// HX711
#define DOUT 25
#define CLK 26
HX711 scale;

// RGB LED Pins
#define LED_R 4
#define LED_G 2
#define LED_B 15

// Buzzer
#define BUZZER_PIN 16

// Pyro Channel
#define PYRO_PIN 12

// SD Card Pins
#define CS_PIN 17
#define MOSI_PIN 23
#define MISO_PIN 19
#define SCK_PIN 18

// Trigger Pins
#define TRIG_PIN1 32
#define TRIG_PIN2 33

// Globals
bool calibrated = false;
bool triggered = false;
File dataFile;

SPIClass spi = SPIClass(VSPI);

void setLED(bool r, bool g, bool b) {
  digitalWrite(LED_R, r);
  digitalWrite(LED_G, g);
  digitalWrite(LED_B, b);
}

void beep(int duration_ms) {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(duration_ms);
  digitalWrite(BUZZER_PIN, LOW);
}

void logDataForDuration(unsigned long duration_ms) {
  Serial.println("[LOGGING] Startet Logging...");
  unsigned long start = millis();
  while (millis() - start < duration_ms) {
    long reading = scale.get_units();
    unsigned long timestamp = millis();
    dataFile.print(timestamp);
    dataFile.print(",");
    dataFile.println(reading);
    delayMicroseconds(500); // ~2kHz
  }
  Serial.println("[LOGGING] Logging abgeschlossen.");
}

void setup() {
  Serial.begin(115200);
  Serial.println("[SETUP] Initialisierung...");

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PYRO_PIN, OUTPUT);
  pinMode(TRIG_PIN1, INPUT_PULLUP);
  pinMode(TRIG_PIN2, INPUT_PULLUP);

  digitalWrite(PYRO_PIN, LOW);
  setLED(true, true, false); // Orange

  scale.begin(DOUT, CLK);
  scale.set_scale();
  scale.tare();
  Serial.println("[HX711] Tare abgeschlossen, warte auf Stabilität...");

  delay(3000);
  if (scale.is_ready()) {
    calibrated = true;
    setLED(false, true, false); // Grün
    Serial.println("[HX711] Kalibrierung erfolgreich.");
  } else {
    Serial.println("[HX711] Fehler bei Kalibrierung.");
  }

  spi.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  if (!SD.begin(CS_PIN, spi)) {
    Serial.println("[SD] SD-Karte nicht erkannt!");
    while (1);
  } else {
    Serial.println("[SD] SD-Karte initialisiert.");
  }

  // Liste vorhandener Dateien (Debug)
  File root = SD.open("/");
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    Serial.print("[SD] Datei gefunden: ");
    Serial.println(entry.name());
    entry.close();
  }
}

void loop() {
  if (!calibrated) return;

  Serial.println("[DEBUG] Warte auf Trigger...");
  delay(1000);

  if (!triggered && digitalRead(TRIG_PIN1) == LOW && digitalRead(TRIG_PIN2) == LOW) {
    Serial.println("[TRIGGER] Startsignal erkannt. Starte Countdown...");
    triggered = true;

    for (int i = 0; i < 20; ++i) {
      beep(100);
      delay(400);
    }

    Serial.println("[PYRO] Zündung aktiviert!");
    digitalWrite(PYRO_PIN, HIGH);

    dataFile = SD.open("/log.csv", FILE_WRITE);
    if (dataFile) {
      Serial.println("[SD] Datei geöffnet.");
      logDataForDuration(3000);
      dataFile.close();
      Serial.println("[SD] Daten gespeichert.");
    } else {
      Serial.println("[SD] Fehler beim Öffnen der Datei!");
    }

    digitalWrite(PYRO_PIN, LOW);
    Serial.println("[PYRO] Zündung deaktiviert.");

    delay(5000);

    setLED(false, true, false); // Grün
    for (int i = 0; i < 3; ++i) {
      beep(200);
      delay(200);
    }
    Serial.println("[SYSTEM] Vorgang abgeschlossen.");
  }
}
