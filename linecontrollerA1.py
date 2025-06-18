import cv2
import numpy as np

class LineFollower:
    def __init__(self):
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        
        self.minAngle = 35
        self.maxAngle = 110

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Failed to grab frame")
            return None, None, None

        frame = cv2.resize(frame, (320, 240))
        roi = frame[160:240, :]  # Bottom third ROI

        # 1️⃣ Detect black lane
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, black_mask = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY_INV)

        kernel = np.ones((3, 3), np.uint8)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)

        # 2️⃣ Detect white (full image)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)

        # 3️⃣ Limit white detection only near the black lane
        dilated_black = cv2.dilate(black_mask, np.ones((15, 15), np.uint8))
        white_mask_limited = cv2.bitwise_and(white_mask, dilated_black)

        # 4️⃣ Draw white boundary (blue outlines)
        white_contours, _ = cv2.findContours(white_mask_limited, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in white_contours:
            if cv2.contourArea(cnt) > 100:
                cv2.drawContours(roi, [cnt], -1, (255, 0, 0), 2)

        # 5️⃣ Black lane steering logic
        contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(roi, [largest_contour], -1, (0, 0, 255), 2)

            M = cv2.moments(largest_contour)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                error = cx - (roi.shape[1] // 2)
                steering_value = error / (roi.shape[1] // 2)
                steering_value = max(-1.0, min(1.0, steering_value))
                angle_range = self.maxAngle - self.minAngle
                angle = (steering_value + 1) / 2 * angle_range + self.minAngle

                cv2.circle(roi, (cx, roi.shape[0] // 2), 5, (0, 255, 0), -1)
                return angle, black_mask, roi

        return None, black_mask, roi

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    lf = LineFollower()

    cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
    cv2.namedWindow("ROI", cv2.WINDOW_NORMAL)

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

