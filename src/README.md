
## 1. **Open Challenge code**
- **Include Libraries**
  ```cpp
  #include <Wire.h>
  #include <MPU6050.h>
  #include <Servo.h>
These headers let us use I²C (Wire) to communicate with the MPU6050 IMU, access the IMU’s functions, and drive a steering servo.
-Define Pins & Constants
```cpp
// HC-SR04 ultrasonic sensors
const int TRIG_LEFT   = 6;
const int ECHO_LEFT   = 7;
const int TRIG_CENT   = 8;
const int ECHO_CENT   = 9;
const int TRIG_RIGHT  = 10;
const int ECHO_RIGHT  = 11;

// Steering servo and motor H-bridge
const int SERVO_PIN   = 2;
const int ENB         = 3;
const int IN3         = 4;
const int IN4         = 5;

// PD control gains
const float Kp = 0.5;
const float Kd = 0.1;

// Safety thresholds (cm)
const float SIDE_MIN_LEFT  = 17.0;
const float SIDE_MIN_RIGHT = 16.0;
const float FRONT_MIN      = 16.0;

// Servo limits
const int SERVO_CENTER    = 90;
const int SERVO_MAX_LEFT  = 60;
const int SERVO_MAX_RIGHT = 120;
const int SERVO_DEADZONE  = 30;

// Motor speed (0–255 PWM)
const int MOTOR_SPEED = 200;
```
These constants map the sensor and actuator pins, set the PD controller gains, safety distance thresholds, servo angle limits, and the fixed motor speed.
-Global Objects & State
```cpp
Servo    steeringServo;
MPU6050  imu;

bool          started    = false;  // Has 's' been received on Serial1?
int           curveCount = 0;      // Number of curves detected
bool          turning    = false;  // Currently in a turn?
float         prevError  = 0;
unsigned long prevTime;
```
We create the servo and IMU objects, plus variables to track whether the robot has started, how many curves it’s made (via gyro), the turning state, and timing for the PD controller.

-Setup Function
```cpp
void setup() {
  // 1) Serial1 for start command
  Serial1.begin(115200);
  Serial1.println("Waiting for 's' to start…");

  // 2) Initialize IMU over I²C
  Wire.begin();
  imu.initialize();
  if (!imu.testConnection()) {
    Serial1.println("IMU not found!");
    while(true); // Halt if IMU missing
  }
  Serial1.println("IMU initialized.");

  // 3) Configure sensor & actuator pins
  pinMode(TRIG_LEFT, OUTPUT);  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_CENT, OUTPUT);  pinMode(ECHO_CENT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  steeringServo.attach(SERVO_PIN);
  steeringServo.write(SERVO_CENTER);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  digitalWrite(IN3, HIGH); // forward
  digitalWrite(IN4, LOW);

  prevTime = micros();
}
```
-Serial1 listens for 's' to begin
-IMU is initialized and tested
-Pins for sensors, servo, and motor are configured and centered
Main Loop
```cpp
void loop() {
  // A) Wait for 's' on Serial1
  if (!started) {
    if (Serial1.available() && Serial1.read() == 's') {
      started = true;
      Serial1.println("Starting navigation…");
    } else {
      return; // idle until start
    }
  }

  // B) Read distances
  float dL = readUltrasonic(TRIG_LEFT,  ECHO_LEFT);
  float dC = readUltrasonic(TRIG_CENT,  ECHO_CENT);
  float dR = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);

  // C) Front obstacle → stop or go
  if (dC <= FRONT_MIN)        analogWrite(ENB, 0);
  else                        analogWrite(ENB, MOTOR_SPEED);

  // D) Emergency side avoidance or PD steering
  if (dL <= SIDE_MIN_LEFT)    steeringServo.write(SERVO_MAX_RIGHT);
  else if (dR <= SIDE_MIN_RIGHT) steeringServo.write(SERVO_MAX_LEFT);
  else {
    unsigned long now = micros();
    float dt = (now - prevTime) / 1e6;
    prevTime = now;
    float error = dR - dL;
    float dErr  = (error - prevError) / dt;
    prevError   = error;
    int delta = constrain(int(Kp*error + Kd*dErr),
                          -SERVO_DEADZONE, SERVO_DEADZONE);
    steeringServo.write(SERVO_CENTER + delta);
  }

  // E) Curve detection via IMU gyro Z-axis
  float gyroZ = imu.getRotationZ() / 131.0;  // °/s
  const float TURN_THRESHOLD = 30.0;
  if (abs(gyroZ) > TURN_THRESHOLD) {
    if (!turning) {
      turning = true;
      curveCount++;
      Serial1.print("Curve #"); Serial1.println(curveCount);
    }
  } else {
    turning = false;
  }

  // F) Stop after 12 curves
  if (curveCount >= 12) {
    analogWrite(ENB, 0);
    steeringServo.write(SERVO_CENTER);
    Serial1.println("12 curves reached—stopping.");
    while(true);
  }

  delay(50); // ~20 Hz
}
```
-Start check: waits for 's'
-Read sensors: left, center, right
-Front control: stop or run motor
-Side avoidance & PD steering: emergency or smooth control
-Curve detection: increment count when gyro Z exceeds threshold
-Stop condition: halt when 12 curves detected
-Utility: Ultrasonic Read
```cpp
float readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return 300; 
  return duration * 0.0343 / 2.0;  // cm
}
```
Triggers the HC-SR04 sensor, measures echo pulse width, and converts it to distance in centimeters.

## 2. **Obstacle Challenge code**
Import libraries
```python
import cv2
import numpy as np
import serial
import time
from gpiozero import Motor, PWMOutputDevice
```
Handles camera access, image processing, numerical operations, motor control, and serial communication with Arduino.

- Serial connection setup
```python
arduino = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)
time.sleep(2)
```
Establishes a UART connection between the Raspberry Pi and Arduino with a short delay to stabilize communication.

- Motor and speed setup
```python
motor = Motor(forward=20, backward=21)
velocidad = PWMOutputDevice(12)
velocidad.value = 0.85
```
Configures GPIO pins for motor direction and sets PWM speed to 85%.

- Camera setup
```python
cap = cv2.VideoCapture(0)
cv2.namedWindow("Vista Completa", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Vista Completa", 800, 600)
```
Initializes the camera and creates a display window for visual feedback.

- Control and state variables
```python
lineas_detectadas = 0
umbral_linea = 1000
umbral_bloque = 800
linea_detectada = False
girando = False
direccion_giro = None

red_limit_x = 170
green_limit_x = 470
```
Tracks the state of line detection, block turns, and defines positional thresholds to determine when a block has passed.

- Helper function to find mask centroid
```python
def obtener_centro(mask):
    M = cv2.moments(mask)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        return cx
    return None
```
Calculates the X-coordinate of the centroid of a binary mask. Used to know where a block is in the frame.

- Frame capture
```python
ret, frame = cap.read()
if not ret:
    continue
```
Reads the current frame from the camera. If the read fails, the loop skips to the next iteration.

- Block detection (red and green)
```python
roi_bloques = frame[200:320, :]
hsv_bloques = cv2.cvtColor(roi_bloques, cv2.COLOR_BGR2HSV)

# Red mask
lower_red1 = np.array([0, 120, 120])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([160, 120, 120])
upper_red2 = np.array([179, 255, 255])
mask_red = cv2.inRange(hsv_bloques, lower_red1, upper_red1) | cv2.inRange(hsv_bloques, lower_red2, upper_red2)

# Green mask
lower_green = np.array([85, 100, 100])
upper_green = np.array([100, 255, 255])
mask_green = cv2.inRange(hsv_bloques, lower_green, upper_green)

area_rojo = cv2.countNonZero(mask_red)
area_green = cv2.countNonZero(mask_green)
cx_rojo = obtener_centro(mask_red)
cx_green = obtener_centro(mask_green)
```
Detects red and green blocks based on HSV color ranges and calculates their positions and areas.

- Black line detection
```python
roi_nav = frame[160:280, :]
gray = cv2.cvtColor(roi_nav, cv2.COLOR_BGR2GRAY)
_, mask_black = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

width = mask_black.shape[1]
third = width // 3
left_black = cv2.countNonZero(mask_black[:, 0:third])
center_black = cv2.countNonZero(mask_black[:, third:2*third])
right_black = cv2.countNonZero(mask_black[:, 2*third:3*third])
```
Uses a grayscale region to isolate the black navigation line and counts how many black pixels are in each section.

- Turning and navigation logic
```python
direction = 'C'
```
Initial direction is centered.

If turning based on block previously detected:
```python
if girando:
    if direccion_giro == 'R':
        if cx_rojo is not None and cx_rojo <= red_limit_x:
            girando = False
            direccion_giro = None
        elif right_black > 300:
            direction = 'C'
        else:
            direction = 'R'
    elif direccion_giro == 'L':
        if cx_green is not None and cx_green >= green_limit_x:
            girando = False
            direccion_giro = None
        elif left_black > 300:
            direction = 'C'
        else:
            direction = 'L'
```
If not currently turning:
```python
else:
    if area_rojo > umbral_bloque and cx_rojo is not None:
        direccion_giro = 'R'
        girando = True
        direction = 'R'
    elif area_green > umbral_bloque and cx_green is not None:
        direccion_giro = 'L'
        girando = True
        direction = 'L'
    else:
        min_black = min(left_black, center_black, right_black)
        if min_black == center_black:
            direction = 'C'
        elif min_black == left_black:
            direction = 'L'
        elif min_black == right_black:
            direction = 'R'
```
Controls the robot’s turning behavior based on detected blocks and black line regions.

- Orange line detection for stopping
```python
roi_linea = frame[400:480, :]
hsv = cv2.cvtColor(roi_linea, cv2.COLOR_BGR2HSV)
lower_orange = np.array([10, 150, 150])
upper_orange = np.array([25, 255, 255])
mask_linea = cv2.inRange(hsv, lower_orange, upper_orange)
area_linea = cv2.countNonZero(mask_linea)

if area_linea > umbral_linea and not linea_detectada and lineas_detectadas < 12:
    lineas_detectadas += 1
    linea_detectada = True
    time.sleep(1)

if area_linea <= umbral_linea:
    linea_detectada = False

if lineas_detectadas >= 12:
    direction = 'S'
    motor.stop()
else:
    motor.forward()
```
Counts the number of orange lines the robot crosses. Stops movement after 12 detections.

- Send servo angle to Arduino
```python
if direction == 'L':
    angulo = '58'
elif direction == 'C':
    angulo = '90'
elif direction == 'R':
    angulo = '120'
else:
    angulo = '90'

arduino.write((angulo + "\n").encode())
```
Maps the movement direction to a specific servo angle and sends the command to the Arduino for steering.

## 3. **Servo control code**
- Include libraries
```ino
#include <Servo.h>
#include <math.h>
```
The Servo.h library is included to control servo motors. math.h is included for compatibility but is not directly used in this code.

- Variable and object declarations
```ino
Servo miServo;
int pinServo = 2;
String comando = "";
unsigned long ultimoEnvio = 0;
const unsigned long intervalo = 100;
```
The miServo object is created to control the servo motor. pinServo sets the pin connected to the servo signal wire. The comando variable stores incoming serial commands as a string. ultimoEnvio and intervalo are reserved for timing tasks but are unused here.

- Setup function
```ino
void setup() {
  miServo.attach(pinServo);
  Serial1.begin(9600);
  Serial.begin(9600);
  delay(1000);

  Serial.println("Send angle (0 to 180) via Serial1 to move the servo.");
  miServo.write(90);
}
```
The servo is attached to the specified pin. UART serial communication (Serial1) is started at 9600 baud to talk with the Raspberry Pi, and USB serial (Serial) is started for debugging. The program waits one second for stability. A message with instructions is printed to the debug monitor. The servo is initialized at center position (90 degrees).

- Loop function: reading and processing commands
```ino
void loop() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();

    if (c == '\n' || c == '\r') {
      comando.trim();

      if (comando.length() > 0) {
        int angulo = comando.toInt();

        if (angulo >= 0 && angulo <= 180) {
          miServo.write(angulo);
          Serial.print("Servo moved to ");
          Serial.print(angulo);
          Serial.println(" degrees");
        } else {
          Serial.println("Angle out of range (0–180)");
        }
      }

      comando = "";
    } else {
      comando += c;
    }
  }
}
```
The code reads incoming characters from the UART serial buffer one by one. It accumulates them into a command string until it detects a newline or carriage return character, signaling the end of a command. Then it trims whitespace from the command. If the command is not empty, it converts the string to an integer representing the servo angle. It checks if the angle is between 0 and 180 degrees. If valid, it moves the servo to that angle and prints a confirmation message. If invalid, it prints an error message. After processing, it clears the command buffer to prepare for the next command and continues looping.

## 4. **Run code when the raspberry turns on**
- Import libraries and modules
```python
from gpiozero import Button
from signal import pause
import subprocess
```
The script imports the Button class from gpiozero to handle GPIO pin inputs on the Raspberry Pi. The pause function from the signal module is imported to keep the script running and listening for events. The subprocess module allows the script to run external programs or scripts.

- Initialize button on GPIO pin 16
```python
boton = Button(16)
```
A Button object is created and linked to GPIO pin 16, which is physically connected to a push button on the Raspberry Pi. This button will trigger running the robot’s main program.

- Define function to run the main program
```python
def ejecutar_programa():
    print("Button pressed! Running the main program...")
    subprocess.run(["python3", "/home/diego/WRO_Ingeniero/otracosaahi.py"])
```
This function runs when the button is pressed. It prints a message to the terminal indicating the button press and then uses subprocess.run to execute another Python script (otracosaahi.py), which is the robot’s main control program. The path must be correct and the script must be executable.

- Link button press event to the function
```python
boton.when_pressed = ejecutar_programa
```
The button’s press event is connected to the ejecutar_programa function. When the button is pressed, this function will be called automatically.

- Indicate readiness and keep script running
```python
print("Waiting for the button to be pressed...")
pause()
```
Prints a message to inform the user that the system is ready and listening for the button press. The pause() function keeps the script running indefinitely so it can detect the button press event; without it, the script would exit immediately.
