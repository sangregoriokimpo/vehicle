from hardware import Car
from vision import LineFollower
import cv2

car = Car()
vision = LineFollower()

try:
    while True:
        angle, mask, roi = vision.process_frame()

        if angle:
            # compute delta for move_steering
            delta = angle - car.current_angle
            car.move_steering(delta)
            car.drive(0.6)
            print(f"Steering to {angle:.1f}")
        else:
            print("Line lost, stopping...")
            car.stop()

        cv2.imshow("Original", roi)
        cv2.imshow("Mask", mask)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

except Exception as e:
    print(f"Error: {e}")

finally:
    vision.release()
    car.stop()
