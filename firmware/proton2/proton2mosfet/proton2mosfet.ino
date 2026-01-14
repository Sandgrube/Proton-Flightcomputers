#define MOSFET_PIN 13 //12 13 14 27

void setup() {
  pinMode(MOSFET_PIN, OUTPUT); // Setze GPIO12 als Ausgang
}

void loop() {
  digitalWrite(MOSFET_PIN, HIGH); // MOSFET aktivieren (Gate HIGH)
  delay(1000);                    // 1 Sekunde warten
  digitalWrite(MOSFET_PIN, LOW);  // MOSFET deaktivieren (Gate LOW)
  delay(1000);                    // 1 Sekunde warten
}
