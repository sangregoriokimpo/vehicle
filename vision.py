import cv2
import numpy as np

class LineFollower:
    def __init__(self):
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        self.minAngle = 50
        self.maxAngle = 130

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Failed to grab frame")
            return None, None, None

        frame = cv2.resize(frame, (320, 240))

        roi = frame[160:240, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, mask = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY_INV)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                error = cx - (roi.shape[1] // 2)
                steering_value = error / (roi.shape[1] // 2)
                steering_value = max(-1.0, min(1.0, steering_value))
                angle_range = self.maxAngle - self.minAngle
                angle = (steering_value + 1) / 2 * angle_range + self.minAngle

                cv2.drawContours(roi, [largest_contour], -1, (0, 0, 255), 2)
                cv2.circle(roi, (cx, roi.shape[0] // 2), 5, (0, 255, 0), -1)

                return angle, mask, roi
        return None, mask, roi

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    lf = LineFollower()

    # Create windows: not resizable, no resizeWindow() needed
    cv2.namedWindow("Mask", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("ROI", cv2.WINDOW_AUTOSIZE)

    while True:
        angle, mask, roi = lf.process_frame()
        if angle is not None:
            print(f"Steering Angle: {angle:.2f}°")
        else:
            print("Line lost!")

        cv2.imshow("Mask", mask)
        cv2.imshow("ROI", roi)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    lf.release()






# import cv2
# import numpy as np

# class LineFollower:
#     def __init__(self):
#         self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
#         self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
#         self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
#         self.minAngle = 35
#         self.maxAngle = 110

#     def process_frame(self):
#         ret, frame = self.cap.read()
#         if not ret:
#             print("Failed to grab frame")
#             return None, None, None

#         frame = cv2.resize(frame, (320, 240))
#         roi = frame[140:240, :]
#         hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

#         lower_black = np.array([0, 0, 0])
#         upper_black = np.array([180, 255, 50])
#         mask = cv2.inRange(hsv, lower_black, upper_black)

#         M = cv2.moments(mask)
#         if M['m00'] > 0:
#             cx = int(M['m10'] / M['m00'])
#             error = cx - 160
#             steering_value = error / 160.0
#             steering_value = max(-1.0, min(1.0, steering_value))
#             angle_range = self.maxAngle - self.minAngle
#             angle = (steering_value + 1) / 2 * angle_range + self.minAngle

#             return angle, mask, roi
#         else:
#             return None, mask, roi

#     def release(self):
#         self.cap.release()
#         cv2.destroyAllWindows()
