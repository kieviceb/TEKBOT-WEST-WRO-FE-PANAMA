# Import necessary libraries
import cv2                          # OpenCV for computer vision
import numpy as np                 # NumPy for handling image arrays
import serial                      # Serial communication with Arduino
import time                        # For delays and timing
from gpiozero import Motor, PWMOutputDevice  # GPIO motor control on Raspberry Pi

# ---------- SERIAL CONNECTION ----------
# Set up UART connection to the Arduino
arduino = serial.Serial("/dev/serial0", baudrate=9600, timeout=1)
time.sleep(2)  # Wait for serial to initialize

# ---------- MOTOR SETUP ----------
motor = Motor(forward=20, backward=21)         # Drive motor GPIO pins
velocidad = PWMOutputDevice(12)                # PWM pin for motor speed
velocidad.value = 0.85                         # Set speed to 85%

# ---------- CAMERA SETUP ----------
cap = cv2.VideoCapture(0)                      # Open default camera
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

cv2.namedWindow("Vista Completa", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Vista Completa", 800, 600)

# ---------- INITIAL VARIABLES ----------
lineas_detectadas = 0          # Count of yellow lines detected
umbral_linea = 1000            # Threshold for detecting yellow line
umbral_bloque = 800            # Threshold for detecting colored blocks
linea_detectada = False
girando = False
direccion_giro = None

# X-coordinates to determine if blocks have passed limits
red_limit_x = 170              # Limit for red block
green_limit_x = 470            # Limit for green block

# ---------- HELPER FUNCTION ----------
# Calculate the center X of a mask (used to track block position)
def obtener_centro(mask):
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
            continue

        # --- BLOCK DETECTION ROI (Region of Interest) ---
        roi_bloques = frame[200:320, :]
        hsv_bloques = cv2.cvtColor(roi_bloques, cv2.COLOR_BGR2HSV)

        # RED color detection (two HSV ranges)
        lower_red1 = np.array([0, 120, 120])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 120, 120])
        upper_red2 = np.array([179, 255, 255])
        mask_red = cv2.inRange(hsv_bloques, lower_red1, upper_red1) | cv2.inRange(hsv_bloques, lower_red2, upper_red2)

        # GREEN color detection (originally cyan)
        lower_green = np.array([85, 100, 100])
        upper_green = np.array([100, 255, 255])
        mask_green = cv2.inRange(hsv_bloques, lower_green, upper_green)

        # Calculate pixel area and center for red and green blocks
        area_rojo = cv2.countNonZero(mask_red)
        area_green = cv2.countNonZero(mask_green)

        cx_rojo = obtener_centro(mask_red)
        cx_green = obtener_centro(mask_green)

        # --- LINE FOLLOWING (BLACK LINE) ROI ---
        roi_nav = frame[160:280, :]
        gray = cv2.cvtColor(roi_nav, cv2.COLOR_BGR2GRAY)
        _, mask_black = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

        # Divide mask into 3 vertical parts: left, center, right
        width = mask_black.shape[1]
        third = width // 3
        left_black = cv2.countNonZero(mask_black[:, 0:third])
        center_black = cv2.countNonZero(mask_black[:, third:2*third])
        right_black = cv2.countNonZero(mask_black[:, 2*third:3*third])

        direction = 'C'  # Default direction (centered)

        # --- BLOCK-TRIGGERED TURN LOGIC ---
        if girando:
            if direccion_giro == 'R':
                # Stop turning right if red block is past the limit
                if cx_rojo is not None and cx_rojo <= red_limit_x:
                    girando = False
                    direccion_giro = None
                elif right_black > 300:
                    direction = 'C'  # Correct if too close to edge
                else:
                    direction = 'R'  # Keep turning right
            elif direccion_giro == 'L':
                # Stop turning left if green block is past the limit
                if cx_green is not None and cx_green >= green_limit_x:
                    girando = False
                    direccion_giro = None
                elif left_black > 300:
                    direction = 'C'
                else:
                    direction = 'L'
        else:
            # --- Decide whether to start a turn ---
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
                # --- If no turn, follow the black line ---
                min_black = min(left_black, center_black, right_black)
                if min_black == center_black:
                    direction = 'C'
                elif min_black == left_black:
                    direction = 'L'
                elif min_black == right_black:
                    direction = 'R'

        # --- YELLOW LINE DETECTION ROI (End-of-Path Lines) ---
        roi_linea = frame[400:480, :]
        hsv = cv2.cvtColor(roi_linea, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 150, 150])
        upper_yellow = np.array([30, 255, 255])
        mask_linea = cv2.inRange(hsv, lower_yellow, upper_yellow)
        area_linea = cv2.countNonZero(mask_linea)

        # Count yellow lines (max 3), with debounce
        if area_linea > umbral_linea and not linea_detectada and lineas_detectadas < 3:
            lineas_detectadas += 1
            linea_detectada = True
            print(f"🟡 Line detected #{lineas_detectadas}")
            time.sleep(1)  # Delay to avoid counting the same line again

        if area_linea <= umbral_linea:
            linea_detectada = False

        # If 3 lines are detected, stop the robot
        if lineas_detectadas >= 3:
            direction = 'S'
            motor.stop()
        else:
            motor.forward()  # Keep moving

        # --- Send direction command to Arduino (for servo) ---
        arduino.write((direction + "\n").encode())

        # --- VISUAL FEEDBACK FOR DEBUGGING ---
        # Show ROIs and direction markers
        cv2.rectangle(frame, (0, 200), (640, 320), (255, 0, 255), 2)     # Block ROI
        cv2.rectangle(frame, (0, 160), (640, 280), (0, 255, 255), 2)     # Navigation ROI
        cv2.rectangle(frame, (0, 400), (640, 480), (0, 0, 255), 2)       # Yellow line ROI
        cv2.line(frame, (red_limit_x, 200), (red_limit_x, 320), (0, 0, 255), 2)
        cv2.line(frame, (green_limit_x, 200), (green_limit_x, 320), (0, 255, 0), 2)

        cv2.putText(frame, f"Dir: {direction}", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,255,0), 3)
        cv2.putText(frame, f"Lines: {lineas_detectadas}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,0,255), 3)

        # Show the camera output
        cv2.imshow("Vista Completa", frame)
        # Uncomment below for extra debug views:
        # cv2.imshow("Mask Line", mask_linea)
        # cv2.imshow("Mask Red", mask_red)
        # cv2.imshow("Mask Green", mask_green)

        # Exit if 'q' is pressed
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# ---------- CLEANUP ----------
finally:
    cap.release()
    cv2.destroyAllWindows()
    arduino.close()
    motor.stop()
