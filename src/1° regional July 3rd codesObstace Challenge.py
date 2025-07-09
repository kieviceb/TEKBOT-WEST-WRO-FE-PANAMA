# Import necessary libraries
import cv2                          # OpenCV for computer vision
import numpy as np                 # NumPy for array handling (image matrices)
import serial                      # Serial communication with Arduino
import time                        # Timing and delays
from gpiozero import Motor, PWMOutputDevice  # GPIO motor and PWM speed control on Raspberry Pi

# ---------- SERIAL CONNECTION ----------
# Setup UART serial connection to Arduino via Raspberry Pi's /dev/serial0
arduino = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)
time.sleep(2)  # Wait to ensure serial connection is ready

# ---------- MOTOR SETUP ----------
motor = Motor(forward=20, backward=21)         # Motor GPIO pins for forward and backward
velocidad = PWMOutputDevice(12)                # PWM pin for motor speed control
velocidad.value = 0.85                         # Set initial speed to 85%

# ---------- CAMERA SETUP ----------
cap = cv2.VideoCapture(0)                      # Open default camera (index 0)

cv2.namedWindow("Vista Completa", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Vista Completa", 800, 600)

# ---------- VARIABLES FOR LINE DETECTION ----------
lineas_detectadas = 0          # Counter for detected orange lines
umbral_linea = 1000            # Pixel threshold to detect orange line presence
umbral_bloque = 800            # Threshold for detecting colored blocks (red or green)
linea_detectada = False        # Debounce flag to avoid multiple counts of same line
girando = False                # Flag indicating if robot is currently turning
direccion_giro = None          # Current turn direction ('L' or 'R')

# Limits in X-axis to decide when block has passed a certain point (for stopping turns)
red_limit_x = 170              # X coordinate limit for red block (right turn)
green_limit_x = 470            # X coordinate limit for green block (left turn)

# ---------- HELPER FUNCTION ----------
def obtener_centro(mask):
    """
    Calculate the centroid X-coordinate of a binary mask.
    Returns None if no area is found.
    """
    M = cv2.moments(mask)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        return cx
    return None

# ---------- MAIN LOOP ----------
try:
    while True:
        ret, frame = cap.read()
        if not ret:
            continue  # Skip frame if capture failed

        # --- BLOCK DETECTION (Red and Green) ---
        roi_bloques = frame[200:320, :]                    # Region where blocks appear
        hsv_bloques = cv2.cvtColor(roi_bloques, cv2.COLOR_BGR2HSV)

        # Detect red blocks (two hue ranges for red)
        lower_red1 = np.array([0, 120, 120])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 120, 120])
        upper_red2 = np.array([179, 255, 255])
        mask_red = cv2.inRange(hsv_bloques, lower_red1, upper_red1) | cv2.inRange(hsv_bloques, lower_red2, upper_red2)

        # Detect green blocks 
        lower_green = np.array([85, 100, 100])
        upper_green = np.array([100, 255, 255])
        mask_green = cv2.inRange(hsv_bloques, lower_green, upper_green)

        # Calculate area and centroid for red and green masks
        area_rojo = cv2.countNonZero(mask_red)
        area_green = cv2.countNonZero(mask_green)
        cx_rojo = obtener_centro(mask_red)
        cx_green = obtener_centro(mask_green)

        # --- BLACK LINE FOLLOWING ---
        roi_nav = frame[160:280, :]                      # Region for line following
        gray = cv2.cvtColor(roi_nav, cv2.COLOR_BGR2GRAY)
        _, mask_black = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)  # Inverted binary mask for black line

        # Split mask into three vertical regions: left, center, right
        width = mask_black.shape[1]
        third = width // 3
        left_black = cv2.countNonZero(mask_black[:, 0:third])
        center_black = cv2.countNonZero(mask_black[:, third:2*third])
        right_black = cv2.countNonZero(mask_black[:, 2*third:3*third])

        # Default direction is center
        direction = 'C'

        # --- TURNING LOGIC BASED ON BLOCKS ---
        if girando:
            if direccion_giro == 'R':
                # Stop turning right when red block passes the limit
                if cx_rojo is not None and cx_rojo <= red_limit_x:
                    girando = False
                    direccion_giro = None
                elif right_black > 300:
                    direction = 'C'  # If too close to edge, center servo
                else:
                    direction = 'R'  # Keep turning right
            elif direccion_giro == 'L':
                # Stop turning left when green block passes the limit
                if cx_green is not None and cx_green >= green_limit_x:
                    girando = False
                    direccion_giro = None
                elif left_black > 300:
                    direction = 'C'
                else:
                    direction = 'L'
        else:
            # Not turning: Check if a block appears to start turning
            if area_rojo > umbral_bloque and cx_rojo is not None:
                direccion_giro = 'R'
                girando = True
                direction = 'R'
                print("🔴 Starting right turn")
            elif area_green > umbral_bloque and cx_green is not None:
                direccion_giro = 'L'
                girando = True
                direction = 'L'
                print("🟢 Starting left turn")
            else:
                # No turn: Follow the black line by checking which section has least black pixels
                min_black = min(left_black, center_black, right_black)
                if min_black == center_black:
                    direction = 'C'
                elif min_black == left_black:
                    direction = 'L'
                elif min_black == right_black:
                    direction = 'R'

        # --- ORANGE LINE DETECTION FOR STOPPING ---
        roi_linea = frame[400:480, :]
        hsv = cv2.cvtColor(roi_linea, cv2.COLOR_BGR2HSV)
        lower_orange = np.array([10, 150, 150])  # Adjusted for orange
        upper_orange = np.array([25, 255, 255])
        mask_linea = cv2.inRange(hsv, lower_orange, upper_orange)
        area_linea = cv2.countNonZero(mask_linea)

        # Count orange lines (max 12), with debounce to avoid double counting
        if area_linea > umbral_linea and not linea_detectada and lineas_detectadas < 12:
            lineas_detectadas += 1
            linea_detectada = True
            print(f"🟧 Orange line detected #{lineas_detectadas}")
            time.sleep(1)  # Small delay to avoid repeated counts

        if area_linea <= umbral_linea:
            linea_detectada = False

        # Stop robot if 12 orange lines detected
        if lineas_detectadas >= 12:
            direction = 'S'  # Stop command
            motor.stop()
            print("🚦 12 orange lines detected, stopping robot.")
        else:
            motor.forward()  # Continue moving forward

        # --- SEND ANGLE COMMAND TO ARDUINO ---
        # Map directions to servo angles:
        # 'L' = 58° (left), 'C' = 90° (center), 'R' = 120° (right), 'S' = stop (no movement)
        if direction == 'L':
            angulo = '58'
        elif direction == 'C':
            angulo = '90'
        elif direction == 'R':
            angulo = '120'
        else:
            angulo = '90'  # Default to center if stopping or unknown

        arduino.write((angulo + "\n").encode())  # Send angle as string over serial

        # --- VISUAL DEBUGGING ---
