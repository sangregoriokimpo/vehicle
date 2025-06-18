import cv2
import numpy as np
import math

class AckermannLaneFollower:
    def __init__(self):
        # Camera capture
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        # Steering configuration
        self.minAngle = 35     # degrees
        self.maxAngle = 110    # degrees

        # Ackermann geometry (fully tuned)
        self.wheelbase = 0.15      # meters (your car's wheelbase)
        self.pixels_per_meter = 2353  # calibrated based on your lane (170mm wide -> 400 px wide)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            print("Failed to grab frame")
            return None, None, None

        frame = cv2.resize(frame, (320, 240))
        roi = frame  # full frame processing

        # 1️⃣ Black lane detection
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, black_mask = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY_INV)

        kernel = np.ones((3, 3), np.uint8)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_CLOSE, kernel)
        black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, kernel)

        # 2️⃣ White boundary detection
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)

        # 3️⃣ Limit white detection near black lane
        dilated_black = cv2.dilate(black_mask, np.ones((15, 15), np.uint8))
        white_mask_limited = cv2.bitwise_and(white_mask, dilated_black)

        # 4️⃣ Draw white boundaries (blue)
        white_contours, _ = cv2.findContours(white_mask_limited, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in white_contours:
            if cv2.contourArea(cnt) > 100:
                cv2.drawContours(roi, [cnt], -1, (255, 0, 0), 2)

        # 5️⃣ Black lane detection + Ackermann path
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

                # Draw centroid
                cv2.circle(roi, (cx, roi.shape[0] - 40), 5, (0, 255, 0), -1)

                # Draw Ackermann path prediction
                self.draw_ackermann_path(roi, steering_value)

                return angle, black_mask, roi

        return None, black_mask, roi

    def draw_ackermann_path(self, roi, steering_value):
        roi_height = roi.shape[0]
        steering_deg = (steering_value + 1) / 2 * (self.maxAngle - self.minAngle) + self.minAngle
        steering_rad = np.radians(steering_deg - 90)

        if abs(steering_rad) < 1e-3:
            for d in range(10, roi_height, 10):
                x = roi.shape[1] // 2
                y = roi.shape[0] - d
                cv2.circle(roi, (x, y), 2, (0, 255, 255), -1)
            return

        radius = self.wheelbase / np.tan(steering_rad)

        for d in range(10, roi_height, 5):
            lookahead_m = d / self.pixels_per_meter
            dx = radius * np.sin(lookahead_m / radius)
            dy = radius * (1 - np.cos(lookahead_m / radius))

            x_pixel = int(roi.shape[1] // 2 + dx * self.pixels_per_meter)
            y_pixel = int(roi.shape[0] - dy * self.pixels_per_meter)

            if 0 <= x_pixel < roi.shape[1] and 0 <= y_pixel < roi.shape[0]:
                cv2.circle(roi, (x_pixel, y_pixel), 2, (0, 255, 255), -1)

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    lf = AckermannLaneFollower()

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
