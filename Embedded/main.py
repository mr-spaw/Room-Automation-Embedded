from gpio_control import drive_mosfet, cleanup_gpio, turn_off_gpio, setup_gpio
from i2s_message import play_audio
import time

def main():
    
    while True:
        try:
            user_input = input("Enter 1 to authenticate or 0 to fail: ")

            if user_input == "1":
                print("Authentication successful")
                time.sleep(0.1)
                play_audio(True)
                time.sleep(0.1)
                drive_mosfet()
                time.sleep(1)
                turn_off_gpio()
                time.sleep(1)
                cleanup_gpio()
                
            elif user_input == "0":
                print("Authentication failed!")
                time.sleep(0.1)
                play_audio(False)
                time.sleep(0.5)
                turn_off_gpio()
                cleanup_gpio()
            else:
                print("Invalid input! Please enter 1 or 0.")

            time.sleep(0.2)

        except KeyboardInterrupt:
            print("Program interrupted by user.")
            break

if __name__ == "__main__":
    try:
        setup_gpio()
        main()
    except KeyboardInterrupt:
        print("Interrupted")
    
    cleanup_gpio()  # Clean up GPIO pins
