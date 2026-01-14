#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>

Adafruit_MPU6050 mpu;
Servo parachuteServo;


// Schwellenwerte für Erkennung des Liftoffs und des Falls
const float liftoffThreshold = 2.00;  // Beschleunigung für Start (z.B. 2g über der Schwerkraft)
const float fallThreshold = 0.50;     // Beschleunigung für Fall (z.B. -1g)

// Variable zur Statusüberwachung
bool liftoffDetected = false;
float lastAccelerationY = 0.0;  // Letzte Beschleunigung auf der Y-Achse

void setup() {
  Serial.begin(115200);
  if (!mpu.begin()) {
    Serial.println("Fehler beim Initialisieren des MPU6050!");
    while (1);  // Endlosschleife bei Initialisierungsfehler
  }
  
  parachuteServo.attach(2);  // Parachute-Servo an Pin D2
  parachuteServo.write(0);   // Initialposition des Servos
  
  Serial.println("Setup abgeschlossen. Warten auf Liftoff...");
}

void loop() {
  if (!liftoffDetected) {
    liftoffDetected = detectLiftoff();
  } else {
    deployingChutes();
  }
  delay(100);  // Leichte Verzögerung für Stabilität
}

// Funktion zur Erkennung des Liftoffs
bool detectLiftoff() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Ausgabe der Beschleunigungswerte zur Überprüfung
  Serial.print("Beschleunigung Y-Achse: ");
  Serial.println(a.acceleration.y);

  // Berechnung der Beschleunigung ohne Schwerkraft
  float accelY = a.acceleration.y - 9.81; // Schwerkraft subtrahieren

  // Ausgabe der korrigierten Y-Beschleunigung
  Serial.print("Korrigierte Y-Beschleunigung: ");
  Serial.println(accelY);

  // Prüfen auf schnelle Änderung in der Y-Beschleunigung
  if (abs(accelY) > liftoffThreshold && abs(accelY - lastAccelerationY) > 1.0) {  // Änderung der Beschleunigung
    Serial.println("Liftoff erkannt!");
    lastAccelerationY = accelY;  // Aktualisiere die letzte Beschleunigung
    delay(5000); // 5 Sekunden warten nach Starterkennung
    return true;
  }

  // Update der letzten Beschleunigung
  lastAccelerationY = accelY;
  return false;
}

// Funktion zum Auslösen des Fallschirms
void deployingChutes() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Ausgabe der Beschleunigungswerte zur Überprüfung
  Serial.print("Beschleunigung Y-Achse: ");
  Serial.println(a.acceleration.y);

  // Prüfen, ob die Y-Beschleunigung einen Fall anzeigt
  if (a.acceleration.y < fallThreshold) {
    Serial.println("Fall erkannt! Fallschirm wird ausgelöst.");
    parachuteServo.write(90);  // Servo um 90 Grad drehen
    delay(2000);               // Position für 2 Sekunden halten
  }
}
