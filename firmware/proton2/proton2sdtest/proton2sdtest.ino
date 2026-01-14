#include <SPI.h>
#include <SD.h>

#define CS_PIN 17
#define MOSI_PIN 23
#define CLK_PIN 18
#define MISO_PIN 19

SPIClass spi = SPIClass(VSPI);

void setup() {
  Serial.begin(115200);
  
  // SPI-Pins explizit definieren
  spi.begin(CLK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

  if (!SD.begin(CS_PIN, spi)) {
    Serial.println("SD-Karte konnte nicht initialisiert werden.");
    return;
  }
  Serial.println("SD-Karte erfolgreich initialisiert.");

  // Test: Datei schreiben
  File file = SD.open("/test.txt", FILE_WRITE);
  if (file) {
    file.println("Hallo von ESP32!");
    file.close();
    Serial.println("Datei erfolgreich geschrieben.");
  } else {
    Serial.println("Datei konnte nicht erstellt werden.");
  }

  // Test: Datei lesen
  file = SD.open("/test.txt");
  if (file) {
    Serial.println("Dateiinhalt:");
    while (file.available()) {
      Serial.write(file.read());
    }
    file.close();
  } else {
    Serial.println("Datei konnte nicht geöffnet werden.");
  }
}

void loop() {
  // Hier könnte man z.B. regelmäßig auf der SD-Karte schreiben/lesen
}
