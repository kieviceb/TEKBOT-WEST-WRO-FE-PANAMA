#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

// —— PINOUT ——
// HC-SR04 Ultrasonic Sensors
const int TRIG_LEFT   = 6;   // Left trigger pin (D6)
const int ECHO_LEFT   = 7;   // Left echo pin    (D7)
const int TRIG_CENT   = 8;   // Center trigger   (D8)
const int ECHO_CENT   = 9;   // Center echo      (D9)
const int TRIG_RIGHT  = 10;  // Right trigger    (D10)
const int ECHO_RIGHT  = 11;  // Right echo       (D11)

// Actuators
const int SERVO_PIN      = 2;   // Steering servo (D2)
const int ENB            = 3;   // Motor enable PWM (D3)
const int IN3            = 4;   // Motor direction (D4)
const int IN4            = 5;   // Motor direction (D5)

// —— CONTROL PARAMETERS —— 
const float Kp           = 0.5; // Proportional gain
const float Kd           = 0.1; // Derivative gain

// —— SAFETY THRESHOLDS (cm) —— 
const float SIDE_MIN_LEFT   = 17.0;  // Min distance to left wall
const float SIDE_MIN_RIGHT  = 16.0;  // Min distance to right wall
const float FRONT_MIN       = 16.0;  // Min front distance

// —— SERVO LIMITS —— 
const int SERVO_CENTER      = 90;   // Center position
const int SERVO_MAX_LEFT    = 60;   // Max left turn
const int SERVO_MAX_RIGHT   = 120;  // Max right turn
const int SERVO_DEADZONE    = 30;   // ±30° around center for PD

// —— MOTOR SPEED ——  
const int MOTOR_SPEED       = 200;  // Fixed PWM (0–255)

Servo steeringServo;
MPU6050 imu;                     // MPU6050 IMU object

// State variables
bool started       = false;     // Has 's' been received?
int curveCount     = 0;         // Number of curves detected
bool turning       = false;     // Are we currently in a turn?
float prevError    = 0;         
unsigned long prevTime = 0;

void setup() {
  // Begin Serial1 for start command
  Serial1.begin(115200);
  Serial1.println("Waiting for 's' on Serial1 to start...");

  // Initialize IMU (MPU6050)
  Wire.begin();
  imu.initialize();
  if (!imu.testConnection()) {
    Serial1.println("IMU connection failed!");
    while (true); // Halt if IMU is missing
  }
  Serial1.println("IMU initialized.");

  // Initialize ultrasonic sensor pins
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_CENT, OUTPUT);
  pinMode(ECHO_CENT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  // Initialize and center steering servo
  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER);

  // Initialize DC motor (H-bridge)
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN3, HIGH);  // Forward direction
  digitalWrite(IN4, LOW);

  prevTime = micros();
}

void loop() {
  // ——— Wait for 's' on Serial1 to begin ———
  if (!started) {
    if (Serial1.available() > 0 && Serial1.read() == 's') {
      started = true;
      Serial1.println("Start command received. Running...");
    } else {
      return; // Do nothing until start
    }
  }

  // ——— 1) Read ultrasonic distances ———
  float dL = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  float dC = readUltrasonic(TRIG_CENT,  ECHO_CENT);
  float dR = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  // ——— 2) Front obstacle check ———
  if (dC <= FRONT_MIN) {
    analogWrite(ENB, 0);  // Too close → stop motor
  } else {
    analogWrite(ENB, MOTOR_SPEED);  // Otherwise fixed speed
  }

  // ——— 3) Emergency side avoidance ———
  if (dL <= SIDE_MIN_LEFT) {
    // Left wall too close → turn right hard
    steeringServo.write(SERVO_MAX_RIGHT);
  }
  else if (dR <= SIDE_MIN_RIGHT) {
    // Right wall too close → turn left hard
    steeringServo.write(SERVO_MAX_LEFT);
  }
  else {
    // ——— 4) PD steering control ———
    unsigned long now = micros();
    float dt = (now - prevTime) / 1e6;
    prevTime = now;

    float error  = dR - dL;
    float dError = (error - prevError) / dt;
    prevError    = error;

    float control = Kp * error + Kd * dError;
    int delta = constrain(int(control), -SERVO_DEADZONE, SERVO_DEADZONE);
    steeringServo.write(SERVO_CENTER + delta);
  }

  // ——— 5) Turn detection using IMU ———
  // Read gyroscope Z axis (°/s)
  float gyroZ = imu.getRotationZ() / 131.0;
  const float TURN_THRESHOLD = 30.0; // °/s threshold for a turn

  if (abs(gyroZ) > TURN_THRESHOLD) {
    if (!turning) {
      turning = true;         // We just started a turn
      curveCount++;
      Serial1.print("Curve #");
      Serial1.println(curveCount);
    }
  } else {
    turning = false;          // No longer turning
  }

  // ——— 6) Stop after 12 curves ———
  if (curveCount >= 12) {
    // Halt motor and center steer
    analogWrite(ENB, 0);
    steeringServo.write(SERVO_CENTER);
    Serial1.println("12 curves detected. Robot stopped.");
    while (true) {
      // Do nothing — robot remains stopped
    }
  }

  delay(50);  // ~20 Hz loop
}

// Generic HC-SR04 reading function (returns cm)
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 300;               // No echo: assume far
  return duration * 0.0343 / 2.0;              // Convert to cm
}
