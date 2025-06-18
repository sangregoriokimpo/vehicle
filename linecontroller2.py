import cv2
import numpy as np
import time
from gpiozero import Servo
from gpiozero.pins.pigpio import PiGPIOFactory

# Use pigpio factory for accurate PWM
factory = PiGPIOFactory()

# Assign your GPIO pins
steeringPin = 17
drivePin0 = 12  # Right drive
drivePin1 = 13  # Left drive

minAngle = 35
maxAngle = 110

# Servo setup (adjust pulse widths if needed)
servoSteering = Servo(steeringPin, pin_factory=factory, min_pulse_width=0.001, max_pulse_width=0.002)
servoRightDrive = Servo(drivePin0, pin_factory=factory, min_pulse_width=0.001, max_pulse_width=0.002)
servoLeftDrive = Servo(drivePin1, pin_factory=factory, min_pulse_width=0.001, max_pulse_width=0.002)

# Open USB camera (likely /dev/video0)
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

def angle_to_servo_value(angle):
    normalized = (angle - 90) / 55
    return max(-1.0, min(1.0, normalized))

def move_servo(angle):
    if minAngle <= angle <= maxAngle:
        servo_value = angle_to_servo_value(angle)
        servoSteering.value = servo_value
        print(f"Moving servo to {angle} degrees (servo value: {servo_value:.2f})")
    else:
        print("Angle out of range.")

def normalized_to_angle(norm_value):
    angle_range = maxAngle - minAngle
    angle = (norm_value + 1) / 2 * angle_range + minAngle
    return angle

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        frame = cv2.resize(frame, (320, 240))
        roi = frame[140:240, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])
        mask = cv2.inRange(hsv, lower_black, upper_black)

        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            error = cx - 160
            steering_value = error / 160.0
            steering_value = max(-1.0, min(1.0, steering_value))

            steering_angle = normalized_to_angle(steering_value)
            move_servo(steering_angle)

            # Move forward when line is detected
            servoRightDrive.value = -0.4
            servoLeftDrive.value = 0.4

            print(f"Line: {cx}  Steering: {steering_value:.2f}")
            cv2.circle(roi, (cx, 50), 5, (255, 0, 0), -1)
        else:
            print("Line lost!")
            # Stop the robot if no line is found
            servoRightDrive.value = 0
            servoLeftDrive.value = 0

        cv2.imshow("Original", roi)
        cv2.imshow("Black Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

finally:
    print("Stopping...")
    servoSteering.value = 0
    servoRightDrive.value = 0
    servoLeftDrive.value = 0
    cap.release()
    cv2.destroyAllWindows()
