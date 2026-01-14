#define LED1_PIN 4
#define LED2_PIN 2
#define LED3_PIN 15

void setup() {
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);

  // Alle aus
  digitalWrite(LED1_PIN, HIGH);
  digitalWrite(LED2_PIN, HIGH);
  digitalWrite(LED3_PIN, HIGH);
}

void loop() {
  // LED1 ein
  digitalWrite(LED1_PIN, LOW);
  delay(200);
  digitalWrite(LED1_PIN, HIGH);

  // LED2 ein
  digitalWrite(LED2_PIN, LOW);
  delay(200);
  digitalWrite(LED2_PIN, HIGH);

  // LED3 ein
  digitalWrite(LED3_PIN, LOW);
  delay(200);
  digitalWrite(LED3_PIN, HIGH);

  // Alle zusammen ein
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  digitalWrite(LED3_PIN, LOW);
  delay(400);
  digitalWrite(LED1_PIN, HIGH);
  digitalWrite(LED2_PIN, HIGH);
  digitalWrite(LED3_PIN, HIGH);

  // Blinken alle gleichzeitig
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED3_PIN, LOW);
    delay(150);
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, HIGH);
    digitalWrite(LED3_PIN, HIGH);
    delay(150);
  }
}

