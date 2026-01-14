#include <Arduino.h>

// Pins für Sensor 1
const int trigPin1 = 12;  // angepasst auf D12
const int echoPin1 = 14;  // angepasst auf D14

// Pins für Sensor 2
const int trigPin2 = 17;
const int echoPin2 = 16;

// Variablen zur Entfernungsmessung
long duration1, duration2;
float distance1, distance2;

void setup() {
  Serial.begin(115200);
  
  // Pins für Sensor 1 festlegen
  pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
  
  // Pins für Sensor 2 festlegen
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
}

void loop() {
  // Entfernungsmessung für Sensor 1
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
  duration1 = pulseIn(echoPin1, HIGH);
  distance1 = duration1 * 0.034 / 2; // Entfernung in cm
  
  // Entfernungsmessung für Sensor 2
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);
  duration2 = pulseIn(echoPin2, HIGH);
  distance2 = duration2 * 0.034 / 2; // Entfernung in cm

  // Daten seriell senden
  Serial.print("Sensor1:");
  Serial.println(distance1);

  Serial.print("Sensor2:");
  Serial.println(distance2);

  // Kurze Verzögerung, um die Datenrate zu regulieren
  delay(50);  // alle 50 ms eine Messung
}
