# Import necessary libraries
import cv2                      # OpenCV: Used for real-time computer vision (camera, image processing)
import numpy as np             # NumPy: Useful for numerical operations and working with arrays (like image data)
import serial                  # Serial: Allows communication with the Arduino
import time                    # Time: Used for delays and measuring time durations
from gpiozero import Motor, PWMOutputDevice  # Controls GPIO pins to operate motors and PWM speed

# ---------- SERIAL COMMUNICATION SETUP ----------
# Establish connection with the Arduino through the UART port of the Raspberry Pi
arduino = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)
time.sleep(2)  # Wait 2 seconds to ensure the connection is stable before continuing

# ---------- DRIVE MOTOR SETUP ----------
# Motor is connected to GPIO pins 20 (forward) and 21 (backward)
motor = Motor(forward=20, backward=21)

# PWM (Pulse Width Modulation) pin for speed control is connected to GPIO pin 12
# PWMOutputDevice allows us to adjust motor speed by changing signal duty cycle
velocidad = PWMOutputDevice(12)
velocidad.value = 0.8  # Set a default base speed (range is 0.0 to 1.0)

# ---------- CAMERA SETUP ----------
# Initialize the default camera (index 0)
cap = cv2.VideoCapture(0)

# Create a window to display what the camera sees, helpful for debugging and alignment
cv2.namedWindow("Vista Completa", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Vista Completa", 800, 600)

# ---------- FUNCTION: Check if a color is present in the mask ----------
def detectar_color_linea(mask, umbral=1500):
    # This function counts how many pixels are 'on' (non-zero) in the mask.
    # If enough pixels are detected, the color is considered "present".
    return cv2.countNonZero(mask) > umbral

# ---------- FUNCTION: Detect orange or blue color in the ROI ----------
def detectar_color_roi(roi_hsv):
    # Define HSV color range for blue
    lower_blue = np.array([100, 55, 20])
    upper_blue = np.array([140, 255, 255])
    mask_blue = cv2.inRange(roi_hsv, lower_blue, upper_blue)

    # Define HSV color range for orange
    lower_orange = np.array([0, 115, 139])
    upper_orange = np.array([25, 255, 255])
    mask_orange = cv2.inRange(roi_hsv, lower_orange, upper_orange)

    # Check which color is detected in sufficient amount
    if detectar_color_linea(mask_orange):
        return "naranja", mask_orange
    elif detectar_color_linea(mask_blue):
        return "azul", mask_blue
    else:
        return None, None  # No color detected

# ---------- MAIN CONTROL LOOP ----------
girando = False         # Indicates if the robot is currently turning
inicio_giro = 0         # Timestamp when the turn started
direccion = 'C'         # C = Centered, L = Left, R = Right

try:
    while True:
        # Read one frame from the camera
        ret, frame = cap.read()
        if not ret:
            continue  # Skip this loop iteration if no frame was captured

        # Define Region of Interest (ROI) near the bottom of the frame
        # This is where the line (color) is expected to be on the floor
        roi_inf = frame[355:405, :]

        # Convert ROI from BGR (default camera format) to HSV (used for color detection)
        hsv_inf = cv2.cvtColor(roi_inf, cv2.COLOR_BGR2HSV)

        # Detect color in the ROI
        color_inf, mask_inf = detectar_color_roi(hsv_inf)

        # Check if robot is currently turning
        if not girando:
            if color_inf == "naranja":
                # Orange detected → Turn right
                print("🟧 Orange detected → sending '120'")
                arduino.write(b'120\n')  # Tell Arduino to move servo right
                velocidad.value = 1.0    # Increase speed temporarily
                inicio_giro = time.time()  # Start turn timer
                girando = True
                direccion = 'R'
            elif color_inf == "azul":
                # Blue detected → Turn left
                print("🔵 Blue detected → sending '58'")
                arduino.write(b'58\n')   # Tell Arduino to move servo left
                velocidad.value = 1.0
                inicio_giro = time.time()
                girando = True
                direccion = 'L'
            else:
                # No color detected → Center servo
                print("⚪ No color detected → centering servo (90)")
                arduino.write(b'90\n')   # Tell Arduino to move servo to center
                direccion = 'C'
                velocidad.value = 0.8
        else:
            # If robot is currently turning, check how long it's been turning
            if time.time() - inicio_giro >= 1.3:
                # If more than 1.3 seconds have passed, stop turning
                print("✅ Turn duration complete → sending '90'")
                arduino.write(b'90\n')   # Center the servo
                velocidad.value = 0.8
                girando = False
                direccion = 'C'

        # Move the robot forward continuously
        motor.forward()

        # ---------- VISUAL FEEDBACK ----------
        # Draw a rectangle showing where the ROI is
        cv2.rectangle(frame, (0, 355), (640, 405), (255, 128, 0), 2)

        # Display the direction label on the screen
        cv2.putText(frame, f"Dir: {direccion}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 3)

        # Show the camera feed
        cv2.imshow("Vista Completa", frame)

        # Show the binary mask of the detected color (for debugging)
        if mask_inf is not None:
            cv2.imshow("ROI Inferior", mask_inf)

        # Allow exit from the program by pressing the 'q' key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# ---------- CLEANUP ON EXIT ----------
finally:
    cap.release()              # Stop camera
    cv2.destroyAllWindows()    # Close all OpenCV windows
    arduino.write(b'90\n')     # Ensure servo is centered before exiting
    arduino.close()            # Close serial connection with Arduino
    motor.stop()               # Stop the drive motor
