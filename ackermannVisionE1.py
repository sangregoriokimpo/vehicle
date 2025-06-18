import cv2
import numpy as np

class AckermannVision:
    def __init__(self):
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        if not self.cap.isOpened():
            raise IOError("Cannot open camera /dev/video0")

        self.minAngle = 35
        self.maxAngle = 110
        self.wheelbase = 0.1  # meters
        self.pixels_per_meter = 300  # must be calibrated

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            print("Failed to grab frame")
            return None, None, None, None

        frame = cv2.resize(frame, (320, 240))
        roi = frame[160:240, :]

        # --- Black lane detection (your original) ---
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, mask = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY_INV)

        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # --- White boundary detection ---
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, kernel)
        white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)

        dilated_black = cv2.dilate(mask, np.ones((15, 15), np.uint8))
        white_mask_limited = cv2.bitwise_and(white_mask, dilated_black)

        white_contours, _ = cv2.findContours(white_mask_limited, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in white_contours:
            if cv2.contourArea(cnt) > 50:
                cv2.drawContours(roi, [cnt], -1, (255, 0, 0), 2)  # blue boundary lines

        # --- Black lane center detection ---
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        lane_width_px = None

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

                x, y, w, h = cv2.boundingRect(largest_contour)
                lane_width_px = w

                return angle, mask, roi, lane_width_px

        return None, mask, roi, lane_width_px

    def draw_trajectory(self, roi, steering_angle_deg):
        center_x = roi.shape[1] // 2
        bottom_y = roi.shape[0]

        steering_angle_rad = np.radians(steering_angle_deg - 75)  # Center at 0 deg

        if abs(steering_angle_rad) < 1e-2:
            radius = 1000  # simulate straight line
        else:
            radius = self.wheelbase / np.tan(steering_angle_rad)

        radius_px = radius * self.pixels_per_meter
        track_width_m = 0.15  # meters
        track_width_px = track_width_m * self.pixels_per_meter

        for y in range(0, roi.shape[0], 5):
            dy = y
            if abs(radius_px) > 1e-2:
                dx_center = radius_px - np.sign(radius_px) * np.sqrt(max(radius_px**2 - dy**2, 0))
            else:
                dx_center = 0

            pt_y = bottom_y - dy

            pt_x_center = int(center_x + dx_center)
            cv2.circle(roi, (pt_x_center, pt_y), 1, (0, 255, 255), -1)  # center yellow

            pt_x_left = int(pt_x_center - (track_width_px / 2))
            cv2.circle(roi, (pt_x_left, pt_y), 1, (0, 255, 0), -1)  # left green

            pt_x_right = int(pt_x_center + (track_width_px / 2))
            cv2.circle(roi, (pt_x_right, pt_y), 1, (0, 0, 255), -1)  # right red

        return roi

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    av = AckermannVision()

    cv2.namedWindow("Trajectory", cv2.WINDOW_AUTOSIZE)

    while True:
        angle, mask, roi, lane_width_px = av.process_frame()

        if roi is None:
            print("No ROI, skipping frame...")
            continue

        if angle is not None:
            print(f"Steering Angle: {angle:.2f}°, Lane width: {lane_width_px}px")
            roi = av.draw_trajectory(roi, angle)
        else:
            print("Line lost!")

        cv2.imshow("Trajectory", roi)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    av.release()
