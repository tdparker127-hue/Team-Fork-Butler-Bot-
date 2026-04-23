import pygame
import time
import serial 
#SERIAL_PORT = '/dev/Arm' # for the arm communication test
SERIAL_PORT = '/dev/Drive' # Symlink to identify ESP32 regardless of port
BAUD_RATE = 115200
def main():
    # Initialize Pygame and the joystick module
    pygame.init()
    pygame.joystick.init()
    # Initialize serial connection
    # timeout=1 ensures readline() doesn't block forever
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    
    # Wait for ESP32 to reboot after serial connection opens
    time.sleep(2) 
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")

    # Verify and initialize the joystick
    if pygame.joystick.get_count() == 0:
        print("No joystick detected.")
        return

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    #print(f"Controller: {joystick.get_name()}")

    try:
        while True:
            # Mandatory: Update joystick state via event pump
            pygame.event.pump()

            # Read Axes and Buttons
            joystickLx = round(joystick.get_axis(0), 2)  # Left stick X-axis
            joystickLy = round(joystick.get_axis(1), 2)  # Left stick Y-axis
            joystickRx = round(joystick.get_axis(2), 2)  # Right stick X-axis
            joystickRy = round(joystick.get_axis(3), 2)  # Right stick Y-axis
            
            #axes = [round(joystick.get_axis(i), 2) for i in range(joystick.get_numaxes())]
            #buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
            packet = f"{joystickLx},{joystickLy},{joystickRx},{joystickRy}\n"
            ser.write(packet.encode('utf-8'))
            print(f"Left Stick: ({joystickLx}, {joystickLy}) | Right Stick: ({joystickRx}, {joystickRy})", end="\r")
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nExiting...")
  

if __name__ == '__main__':
    main()
