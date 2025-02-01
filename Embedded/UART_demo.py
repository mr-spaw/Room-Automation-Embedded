import serial
import time

uart_port = '/dev/ttyUSB4'  # Update if necessary
baud_rate = 115200  # Check the ESP firmware

def setup_serial():
    try:
        ser = serial.Serial(uart_port, baud_rate, timeout=1)
        time.sleep(2)  # Wait for the serial connection to establish
        return ser
    except Exception as e:
        print(f"Error setting up serial: {e}")
        return None

def send_command(ser, command):
    """Send a command to the ESP and return the full response."""
    try:
        ser.write((command + "\n").encode('utf-8'))  # Send command with newline
        time.sleep(0.1)  # Short delay to allow response
        response = ""
        chunk = ""
        start_time = time.time()

        while True:
            if ser.in_waiting > 0:
                # Read a chunk of data from the serial buffer
                chunk = ser.read(ser.in_waiting).decode('utf-8')
                print(f"Received chunk: {chunk}")  # Debugging line
                response += chunk

                # Reset timeout as data is still being received
                start_time = time.time()
            elif time.time() - start_time > 6:  # No data for 1 second, assume complete
                break

        return response.strip() if response else "No data received."
    except Exception as e:
        print(f"Error sending command: {e}")
        return None

def main():
    ser = setup_serial()
    if not ser:
        return

    while True:
        command = input("Enter command (LIDAR, IR_DETECT, RESET, RECOGON, RECOGOFF, NORECOGON, NORECOGOFF, EXIT): ").strip().upper()
        if command == "LIDAR":
            print("LIDAR Data:")
            print(send_command(ser, "LIDAR"))
        elif command == "IR_DETECT":
            print("IR Detection Status:")
            print(send_command(ser, "IR_DETECT"))
        elif command == "RESET":
            print("Resetting ESP...")
            send_command(ser, "RESET")
        elif command == "RECOGON":
            print("Turning recognition ON...")
            send_command(ser, "RECOGON")
        elif command == "RECOGOFF":
            print("Turning recognition OFF...")
            send_command(ser, "RECOGOFF")
        elif command == "NORECOGON":
            print("Turning no recognition ON...")
            send_command(ser, "NORECOGON")
        elif command == "NORECOGOFF":
            print("Turning no recognition OFF...")
            send_command(ser, "NORECOGOFF")
        elif command == "EXIT":
            break
        else:
            print("Invalid command.")

    ser.close()

if __name__ == "__main__":
    main()

