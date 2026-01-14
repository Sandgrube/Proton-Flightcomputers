#define BUZZER_PIN 16

void setup() {
  // 3 Töne kurz hintereinander
  tone(BUZZER_PIN, 440); // Ton A4
  delay(200);            // 200ms
  noTone(BUZZER_PIN);
  delay(100);            // kurze Pause

  tone(BUZZER_PIN, 523); // Ton C5
  delay(200);
  noTone(BUZZER_PIN);
  delay(100);

  tone(BUZZER_PIN, 659); // Ton E5
  delay(200);
  noTone(BUZZER_PIN);
}

void loop() {
  // Nichts
}
