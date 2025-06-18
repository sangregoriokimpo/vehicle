from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory
import time

# Initialize pigpio factory
factory = PiGPIOFactory()

# Your steering pin
steering_pin = 17

# Create servo object with your pulse width range
steering = Servo(steering_pin, pin_factory=factory, min_pulse_width=0.001, max_pulse_width=0.002)

# Helper function to convert angle to servo value
def angle_to_servo_value(angle):
    # Clamp angle between 35 and 110 degrees
    angle = max(50, min(130, angle))
    servo_value = (angle - 90) / 55
    return max(-1.0, min(1.0, servo_value))

# Move to center first
center_value = angle_to_servo_value(90)
steering.value = center_value
print("Moving to center (90 degrees)")
time.sleep(2)

# Test full left (35 degrees)
left_value = angle_to_servo_value(50)
steering.value = left_value
print("Moving to full left (50 degrees)")
time.sleep(2)

# Test full right (110 degrees)
right_value = angle_to_servo_value(130)
steering.value = right_value
print("Moving to full right (130 degrees)")
time.sleep(2)

# Return to center
steering.value = center_value
print("Returning to center (90 degrees)")
time.sleep(2)

# Stop servo
steering.value = 0
print("Done.")
