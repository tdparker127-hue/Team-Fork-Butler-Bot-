import serial
import time

# Use the actual port (e.g., /dev/ttyUSB0) if the symlink is failing
PORT = '/dev/Drivetrain' 
BAUD = 115200

try:
    print(f"Attempting to open {PORT}...")
    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        print("Success! Port is open.")
        time.sleep(2)  # Wait for ESP32 reboot
        
        test_msg = "test_connection\n"
        ser.write(test_msg.encode('utf-8'))
        print(f"Sent: {test_msg.strip()}")
        
        # Check if the ESP32 sends anything back
        response = ser.readline().decode('utf-8').strip()
        if response:
            print(f"Received from ESP32: {response}")
        else:
            print("No response received (this is okay if your ESP32 isn't coded to reply).")

except serial.SerialException as e:
    print(f"Error: Could not open serial port: {e}")
except Exception as e:
    print(f"An unexpected error occurred: {e}")
