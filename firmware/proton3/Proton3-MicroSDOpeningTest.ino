#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

static const int SD_CS = 14;
static const int SD_CD = 21;

static const int SPI_SCK  = 12;
static const int SPI_MOSI = 11;
static const int SPI_MISO = 13;

// andere SPI CS (BMI088)
static const int CS_ACC  = 47;
static const int CS_GYRO = 38;

static const bool CD_ACTIVE_LOW = true;

bool cardPresent() {
  pinMode(SD_CD, INPUT_PULLUP);
  return CD_ACTIVE_LOW ? (digitalRead(SD_CD) == LOW) : (digitalRead(SD_CD) == HIGH);
}

bool initSD() {
  // Bus ruhigstellen
  pinMode(SD_CS, OUTPUT);   digitalWrite(SD_CS, HIGH);
  pinMode(CS_ACC, OUTPUT);  digitalWrite(CS_ACC, HIGH);
  pinMode(CS_GYRO, OUTPUT); digitalWrite(CS_GYRO, HIGH);

  SPI.end();
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS);
  delay(5);

  // SD SPI wake: CS HIGH + >=74 clocks
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  digitalWrite(SD_CS, HIGH);
  for (int i=0;i<10;i++) SPI.transfer(0xFF);
  SPI.endTransaction();
  delay(2);

  // langsam init -> dann klappt’s auch bei “zickigen” Karten/Layouts
  if (!SD.begin(SD_CS, SPI, 400000)) {
    if (!SD.begin(SD_CS, SPI, 100000)) return false;
  }
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1200);

  Serial.println("\nSD final test");

  if (!cardPresent()) {
    Serial.println("Keine SD (CD).");
    return;
  }

  if (!initSD()) {
    Serial.println("SD init FAIL");
    return;
  }

  Serial.println("SD init OK");

  File f = SD.open("/test.txt", FILE_WRITE);
  if (!f) { Serial.println("open failed"); return; }

  f.print("boot_ms="); f.println(millis());
  f.println("hello from flight computer");
  f.close();

  Serial.println("wrote /test.txt");
}

void loop() {}
