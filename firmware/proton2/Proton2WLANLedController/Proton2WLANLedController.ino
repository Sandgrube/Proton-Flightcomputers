#include <WiFi.h>
#include <WebServer.h>
#include <SD.h>
#include <SPI.h>

// RGB-LED Pins
#define RED_PIN 15
#define GREEN_PIN 2
#define BLUE_PIN 4

// SD-Karten Pins (SPI manuell angegeben)
#define SD_CS     17
#define SD_MOSI   23
#define SD_CLK    18
#define SD_MISO   19

const char* ssid = "Proton2";
const char* password = "1234";

WebServer server(80);

String currentColor = "#000000"; // Standardfarbe

String htmlPage() {
  return "<!DOCTYPE html><html><body><h1>LED Farbe wählen</h1>"
         "<form action=\"/set\" method=\"POST\">"
         "<input type=\"color\" name=\"color\" value=\"" + currentColor + "\">"
         "<br><br><input type=\"submit\" value=\"Annehmen\">"
         "</form></body></html>";
}

void setColorFromHex(String hex) {
  if (hex.length() != 7 || hex[0] != '#') return;

  long color = strtol(hex.substring(1).c_str(), NULL, 16);
  int r = (color >> 16) & 0xFF;
  int g = (color >> 8) & 0xFF;
  int b = color & 0xFF;

  // analogWrite auf ESP32 (funktioniert seit Core 2.0+)
  // Wenn Common Anode LED, invertiere: 255 - wert
  analogWrite(RED_PIN, 255 - r);
  analogWrite(GREEN_PIN, 255 - g);
  analogWrite(BLUE_PIN, 255 - b);
}

void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

void saveColorToSD(String color) {
  File file = SD.open("/color.txt", FILE_WRITE);
  if (file) {
    file.print(color);
    file.close();
  }
}

String loadColorFromSD() {
  if (!SD.exists("/color.txt")) return "#000000";
  File file = SD.open("/color.txt");
  if (!file) return "#000000";

  String color = file.readStringUntil('\n');
  file.close();
  if (color.length() == 7 && color[0] == '#') return color;
  return "#000000";
}

void handleSet() {
  if (server.hasArg("color")) {
    String newColor = server.arg("color");
    if (newColor != currentColor) {
      currentColor = newColor;
      setColorFromHex(currentColor);
      saveColorToSD(currentColor);
      server.send(200, "text/html", "<html><body><h1>Farbe übernommen</h1></body></html>");
      delay(1000);
      ESP.restart();
    } else {
      server.send(200, "text/html", "<html><body><h1>Keine Änderung</h1></body></html>");
    }
  } else {
    server.send(400, "text/plain", "Fehler: Keine Farbe übergeben.");
  }
}

void setupSD() {
  SPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS)) {
    Serial.println("SD-Karte konnte nicht initialisiert werden.");
  } else {
    Serial.println("SD-Karte erfolgreich gestartet.");
  }
}

void setup() {
  Serial.begin(115200);

  // Pins als PWM Ausgang definieren (ESP32 Core 2.0+)
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  setupSD();

  currentColor = loadColorFromSD();
  setColorFromHex(currentColor);

  WiFi.softAP(ssid, password);
  Serial.println("AP gestartet: " + String(ssid));
  Serial.println("IP-Adresse: " + WiFi.softAPIP().toString());

  server.on("/", handleRoot);
  server.on("/set", HTTP_POST, handleSet);
  server.begin();
  Serial.println("Webserver läuft");
}

void loop() {
  server.handleClient();
}
