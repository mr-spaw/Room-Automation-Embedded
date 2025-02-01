import serial
import time

uart_port = '/dev/ttyUSB0'  # Update if necessary
baud_rate = 115200  # Check the ESP firmware

class SerialController:
    def __init__(self):
        self.ser = self.setup_serial()

    def setup_serial(self):
        try:
            ser = serial.Serial(uart_port, baud_rate, timeout=1)
            time.sleep(2)  # Wait for the serial connection to establish
            return ser
        except Exception as e:
            print(f"Error setting up serial: {e}")
            return None
    
    def send_command(self, command):
        self.ser.write((command + "\n").encode('utf-8'))  # Send command with newline
        time.sleep(1)  # Wait for a brief moment

    def send_read_command(self, command, expect_response=True):
        try:
            self.ser.write((command + "\n").encode('utf-8'))  # Send command with newline
            time.sleep(1)  # Wait for a brief moment
            
            if not expect_response:
                return "Command sent successfully."

            response = ""
            start_time = time.time()

            while True:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').rstrip()
                    print(f"Raw Response: {line}")  # Debugging line
                    response += line + "\n"
                    
                    if line and line != "EE":  # Ignore undesirable response
                        break
                if time.time() - start_time > 10:  # Timeout after 10 seconds
                    print("Timeout waiting for response.")
                    break
            
            return response.strip() if response else "No valid data received."
        except Exception as e:
            print(f"Error sending command: {e}")
            return None

    def get_lidar_data(self):
        return self.send_read_command("GetLIDAR")

    def get_ir_data(self):
        return self.send_read_command("GetIR")

    def reset(self):
        return self.send_command("RESET", expect_response=False)

    def turn_recognition_on(self):
        return self.send_command("RECOGON", expect_response=False)

    def turn_recognition_off(self):
        return self.send_command("RECOGOFF", expect_response=False)

    def turn_no_recognition_on(self):
        return self.send_command("NORECOGON", expect_response=False)

    def turn_no_recognition_off(self):
        return self.send_command("NORECOGOFF", expect_response=False)

    def close(self):
        if self.ser:
            self.ser.close()

# Example usage (You can remove or modify this section as needed):
if __name__ == "__main__":
    controller = SerialController()
    lidar_data = controller.get_lidar_data()
    print(lidar_data)
    
    ir_data = controller.get_ir_data()
    print(ir_data)

    controller.reset()
    controller.turn_recognition_on()
    controller.turn_recognition_off()
    controller.turn_no_recognition_on()
    controller.turn_no_recognition_off()
    controller.close()

