from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

class Car:
    def __init__(self, steering_pin=17, drive_pin_right=12, drive_pin_left=13, min_pulse=0.001, max_pulse=0.002):
        factory = PiGPIOFactory()
        self.minAngle = 50
        self.maxAngle = 130

        self.steering = Servo(steering_pin, pin_factory=factory, min_pulse_width=min_pulse, max_pulse_width=max_pulse)
        self.right_drive = Servo(drive_pin_right, pin_factory=factory, min_pulse_width=min_pulse, max_pulse_width=max_pulse)
        self.left_drive = Servo(drive_pin_left, pin_factory=factory, min_pulse_width=min_pulse, max_pulse_width=max_pulse)

        self.current_angle = 90  # Start at center (match Arduino default)
        self.drive_power = 0.0

        self.move_steering(self.current_angle)

    def move_steering(self, angle):
        # Clamp angle between min and max range
        angle = max(self.minAngle, min(self.maxAngle, angle))
        self.current_angle = angle
        # Map to gpiozero Servo value (-1 to 1)
        servo_value = (self.current_angle - 90) / 55
        servo_value = max(-1.0, min(1.0, servo_value))
        self.steering.value = servo_value
        print(f"Steering moved to {self.current_angle}° ({servo_value:.2f})")

    def reset_steering(self):
        self.move_steering(75)  # Reset to center

    def drive(self, delta):
        self.drive_power += delta
        self.drive_power = max(-1.0, min(1.0, self.drive_power))
        self.right_drive.value = -self.drive_power
        self.left_drive.value = self.drive_power
        print(f"Drive power: {self.drive_power:.2f}")

    def stop(self):
        self.right_drive.value = 0
        self.left_drive.value = 0
        self.steering.value = 0
        print("Car stopped.")
