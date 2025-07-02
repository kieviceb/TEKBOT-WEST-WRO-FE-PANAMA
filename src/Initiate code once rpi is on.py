# Import the Button class from the gpiozero library.
# This library is used to control GPIO (General Purpose Input/Output) pins on a Raspberry Pi.
from gpiozero import Button

# Import 'pause' from the signal module.
# 'pause()' will keep the script running and listening for events, such as a button press.
from signal import pause

# Import the subprocess module.
# This allows us to run external programs or scripts from within this Python script.
import subprocess

# Create a Button object connected to GPIO pin 16 on the Raspberry Pi.
# This pin is physically connected to a push button, which will be used to start the robot's main program.
boton = Button(16)

# Define a function that will be called when the button is pressed.
def ejecutar_programa():
    # Print a message to the terminal to indicate that the button was pressed.
    print("Button pressed! Running the main program...")
    
    # Use subprocess.run to execute another Python script.
    # In this case, it runs 'otracosaahi.py', which is the main control program for the robot.
    # Make sure the path is correct and the script is executable.
    subprocess.run(["python3", "/home/diego/WRO_Ingeniero/otracosaahi.py"])

# Link the button press event to the function defined above.
# This means that when the button is pressed, the 'ejecutar_programa' function will run.
boton.when_pressed = ejecutar_programa

# Print a message indicating that the system is waiting for the button press.
# This helps the user know the system is ready and listening.
print("Waiting for the button to be pressed...")

# Keep the program running indefinitely, waiting for the button press event.
# Without this, the script would end immediately.
pause()

