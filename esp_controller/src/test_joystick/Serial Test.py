import serial
import time
#checks serial communication for debugging purposes, sends "PING" to ESP32 and waits for response, prints it to console. Make sure to adjust SERIAL_PORT if needed.
#created a name using symlink to identify the esp board no matter which ports it's plugged into. 
#SERIAL_PORT = '/dev/Arm' # Adjust this if your ESP32 is on a different port
SERIAL_PORT = '/dev/Drive' # Symlink to identify ESP32 regardless of port
BAUD_RATE = 115200

try:
    # Initialize serial connection
    # timeout=1 ensures readline() doesn't block forever
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    
    # Wait for ESP32 to reboot after serial connection opens
    time.sleep(2) 
    ser.reset_input_buffer()
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")

    while True:
        # 1. Send data to ESP32
        message = "PING\n"
        ser.write(message.encode('utf-8'))
        print(f"Sent: {message.strip()}")

        # 2. Wait for and read response
        if ser.in_waiting > 0:
            response = ser.readline().decode('utf-8').rstrip()
            print(f"Received from ESP32: {response}")

        time.sleep(1) # Send every second

except serial.SerialException as e:
    print(f"Error: {e}")
except KeyboardInterrupt: #press Ctrl+C to exit
    print("\nClosing connection...")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()

