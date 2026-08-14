#!/usr/bin/env python3
import serial
import time
import sys

PORT = '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
BAUD = 115200

def test_hardware():
    print(f"--- Lift Hardware Test ---")
    print(f"Connecting to {PORT} @ {BAUD}...")
    
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1.0)
        print("✅ Serial Port Opened Successfully.")
    except Exception as e:
        print(f"❌ Failed to open serial port: {e}")
        print("Suggestions:")
        print("1. Check if USB cable is plugged in.")
        print("2. Check permissions on the by-id serial device.")
        print("3. Ensure no other ROS nodes are running (pkill -f ros2).")
        sys.exit(1)

    print("\nReading data for 5 seconds...")
    start_time = time.time()
    data_received = False
    
    while time.time() - start_time < 5.0:
        line = ser.readline().decode(errors='ignore').strip()
        if line:
            print(f"RX: {line}")
            data_received = True

    if not data_received:
        print("\n❌ No data received from Arduino.")
        print("Possible causes:")
        print("1. Arduino has no power.")
        print("2. Wrong baud rate / firmware mess up.")
        sys.exit(1)
    else:
        print("\n✅ Data received. Communication OK.")

    # Optional: Send HOME
    print("\nSending HOME command in 3 seconds... (Make sure area is clear)")
    time.sleep(3)
    ser.write(b'HOME\n')
    print("Sent: HOME")
    
    # Read response
    start_time = time.time()
    while time.time() - start_time < 5.0:
        line = ser.readline().decode(errors='ignore').strip()
        if line:
            print(f"RX: {line}")

    ser.close()
    print("\n--- Test Finished ---")

if __name__ == '__main__':
    test_hardware()
