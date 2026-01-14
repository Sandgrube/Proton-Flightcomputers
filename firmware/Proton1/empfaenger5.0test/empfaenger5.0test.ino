#include <esp_now.h>
#include <WiFi.h>

// Empfänger-MAC-Adresse
uint8_t receiverMac[] = {0x30, 0xc9, 0x22, 0x33, 0x38, 0xb0};

//--- Struktur für die empfangenen Daten (muss mit der Senderstruktur übereinstimmen) ---

}

// Struktur für die Datenübertragung
typedef struct {
  char command[32];
} Command;

// Funktion für den Status der Datenübertragung
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Nachricht an ");
  for (int i = 0; i < 6; i++) {
    Serial.print(mac_addr[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.print(" wurde ");
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("erfolgreich gesendet.");
  } else {
    Serial.println("NICHT gesendet.");
  }
}

// Initialisierung
void setup() {
  Serial.begin(115200);

  // WLAN im Station-Modus starten
  WiFi.mode(WIFI_STA);

  // ESP-NOW initialisieren
  if (esp_now_init() != ESP_OK) {
    Serial.println("Fehler beim Initialisieren von ESP-NOW.");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);
  // Callback für Sendestatus registrieren
  esp_now_register_send_cb(OnDataSent);

  // Empfänger zu ESP-NOW hinzufügen
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0; // Verwende den Standardkanal
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Fehler beim Hinzufügen des Empfängers.");
    return;
  }
  Serial.println("ESP-NOW Sender bereit.");
}

// Nachricht senden
void sendCommand(const char *cmd) {
  Command command;
  strncpy(command.command, cmd, sizeof(command.command));
  command.command[sizeof(command.command) - 1] = '\0'; // Sicherstellen, dass der String terminiert ist

  // Nachricht senden
  esp_err_t result = esp_now_send(receiverMac, (uint8_t *)&command, sizeof(command));
  if (result == ESP_OK) {
    Serial.println("Nachricht erfolgreich gesendet: " + String(cmd));
  } else {
    Serial.println("Fehler beim Senden der Nachricht.");
  }
}

// Loop für Befehle
void loop() {
  Serial.println("Gib einen Befehl ein: (z.B. DEPLOY_PARACHUTE, BUZZER_ON, BUZZER_OFF)");
  while (!Serial.available()) {
    delay(100); // Warten auf Eingabe
  }

  String input = Serial.readStringUntil('\n');
  input.trim(); // Entferne Leerzeichen und neue Zeilen

  if (!input.isEmpty()) {
    sendCommand(input.c_str());
  }
}
