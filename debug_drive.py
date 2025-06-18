from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import time

# Initialize pigpio factory
factory = PiGPIOFactory()

# Your drive pins
right_drive_pin = 12
left_drive_pin = 13

# Create servo objects for drive motors
right_drive = Servo(right_drive_pin, pin_factory=factory, min_pulse_width=0.001, max_pulse_width=0.002)
left_drive = Servo(left_drive_pin, pin_factory=factory, min_pulse_width=0.001, max_pulse_width=0.002)

# Function to set drive power
def set_drive_power(power):
    # Clamp power between -1 and 1
    power = max(-1.0, min(1.0, power))
    right_drive.value = -power  # right motor is inverted
    left_drive.value = power
    print(f"Drive power set to: {power:.2f}")

# Start test sequence
print("Testing drive motors...")

# Move forward slowly
set_drive_power(0.3)
time.sleep(2)

# Move forward faster
set_drive_power(0.6)
time.sleep(2)

# Full forward
set_drive_power(1.0)
time.sleep(2)

# Stop
set_drive_power(0)
time.sleep(1)

# Reverse slowly
set_drive_power(-0.3)
time.sleep(2)

# Full reverse
set_drive_power(-1.0)
time.sleep(2)

# Stop again
set_drive_power(0)
print("Done.")
