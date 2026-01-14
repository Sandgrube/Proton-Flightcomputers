#include <ESP32Servo.h>

// ---ServoStartupSequenz---
void ServoStartupSequenz(Servo& servoX, Servo& servoY) {
  // Servos in Mittelstellung bringen
  servoX.write(90);  
  servoY.write(90);  
  delay(500);  // Kurz warten

  // Kalibrierung: leichte Bewegungen zum Test
  for (int i = 0; i < 2; i++) {
    servoX.write(100);  // +10 Grad
    servoY.write(100);  
    delay(200);

    servoX.write(80);   // -10 Grad
    servoY.write(80);   
    delay(200);

    servoX.write(90);   // Zurück zur Mittelstellung
    servoY.write(90);   
    delay(200);
  }

  // Abschluss in Mittelstellung
  servoX.write(90);  
  servoY.write(90);  
  delay(500);
}

Servo servoX;  // Servo für X-Achse
Servo servoY;  // Servo für Y-Achse

void setup() {
  servoX.attach(17);  // Servo X an Pin 17
  servoY.attach(5);   // Servo Y an Pin 5
  
  // ---ServoStartupSequenz Aufruf---
  ServoStartupSequenz(servoX, servoY);
}

void loop() {
  // Hauptcode
}
