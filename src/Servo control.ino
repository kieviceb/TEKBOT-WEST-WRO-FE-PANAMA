#include <Servo.h>   // Library to control servo motors
#include <math.h>    // Included for compatibility (not used directly here)

Servo miServo;             // Create servo motor object
int pinServo = 2;          // Define the pin connected to the servo signal wire
String comando = "";       // Buffer for incoming serial commands

unsigned long ultimoEnvio = 0;
const unsigned long intervalo = 100;  // (Reserved) Time interval for potential future tasks

void setup() {
  miServo.attach(pinServo);          // Attach the servo to its control pin
  Serial1.begin(9600);               // Serial communication with Raspberry Pi (UART)
  Serial.begin(9600);                // USB serial monitor for debugging
  delay(1000);                       // Wait for everything to stabilize

  Serial.println("Send angle (0 to 180) via Serial1 to move the servo.");
  miServo.write(90);                 // Initialize servo in the center (90°)
}

void loop() {
  // --- Read and process commands from Raspberry Pi ---
  while (Serial1.available() > 0) {
    char c = Serial1.read();         // Read one character at a time

    if (c == '\n' || c == '\r') {    // Command ends when newline or carriage return is received
      comando.trim();                // Remove whitespace or extra characters

      if (comando.length() > 0) {
        int angulo = comando.toInt();  // Convert the command to an integer angle

        // Check if the angle is valid (between 0° and 180°)
        if (angulo >= 0 && angulo <= 180) {
          miServo.write(angulo);     // Move the servo to the requested angle
          Serial.print("Servo moved to ");
          Serial.print(angulo);
          Serial.println(" degrees");
        } else {
          Serial.println("Angle out of range (0–180)");
        }
      }

      comando = "";                  // Clear the command string
    } else {
      comando += c;                  // Build the command character by character
    }
  }
}
