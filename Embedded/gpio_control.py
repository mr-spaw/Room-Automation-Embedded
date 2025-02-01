import lgpio
import time

# Define the LED pin
LED = 17

def setup_gpio():
    global h
    h = lgpio.gpiochip_open(0)  # Open GPIO chip 0
    lgpio.gpio_claim_output(h, LED)  # Claim the LED pin as output

def drive_mosfet():
    lgpio.gpio_write(h, LED, 1)  # Turn the MOSFET on
    time.sleep(2)                # Keep it on for 2 seconds
    lgpio.gpio_write(h, LED, 0)  # Turn the MOSFET off

def turn_off_gpio():
    lgpio.gpio_write(h, LED, 0)  # Ensure the MOSFET is turned off

def cleanup_gpio():
    lgpio.gpio_write(h, LED, 0)  # Turn off the MOSFET
    lgpio.gpiochip_close(h)       # Close the GPIO chip


