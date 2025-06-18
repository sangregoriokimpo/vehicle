import cv2
import numpy as np
import math

class AckermannPathPredictor:
    def __init__(self):
        # Camera capture
        self.cap = cv2.VideoCapture('/dev/video0', cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        # Parameters
        self.wheelbase = 0.15  # meters (actual car wheelbase)
        self.pixels_per_meter = 2353  # your calibration

        self.lane_width_m = 0.17  # meters (lane width 170mm)
        self.lane_width_px = 400  # pixels (based on your example)

    def get_ipm_transform(self, roi):
        src = np.float32([
            [60, 140],  
            [260, 140], 
            [0, 240],   
            [320, 240]  
        ])
        dst = np.float32([
            [80, 0],
            [240, 0],
            [80, 240],
            [240, 240]
        ])
        M = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(roi, M, (320, 240))
        return warped

    def detect_lane_center(self, ipm_img):
        hsv = cv2.cvtColor(ipm_img, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 50, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        centers = []
        for y in range(220, 0, -10):
            row = white_mask[y, :]
            nonzero = np.where(row > 0)[0]
            if len(nonzero) > 0:
                cx = int(np.mean(nonzero))
                centers.append((cx, y))

        return centers, white_mask

    def fit_polynomial(self, centers):
        if len(centers) < 5:
            return None
        centers = np.array(centers)
        X = centers[:, 0]
        Y = centers[:, 1]
        poly_coeff = np.polyfit(Y, X, 2)
        return poly_coeff

    def calculate_curvature(self, poly_coeff, y_eval):
        xm_per_pix = self.lane_width_m / self.lane_width_px
        ym_per_pix = 1.0 / self.pixels_per_meter

        A = poly_coeff[0] * (xm_per_pix / (ym_per_pix**2))
        B = poly_coeff[1] * (xm_per_pix / ym_per_pix)

        R_curve = ((1 + (2*A*y_eval*ym_per_pix + B)**2)**1.5) / abs(2*A)
        return R_curve

    def draw_polynomial(self, ipm_img, poly_coeff):
        plot_y = np.linspace(0, 240, 240)
        plot_x = poly_coeff[0]*plot_y**2 + poly_coeff[1]*plot_y + poly_coeff[2]
        for x, y in zip(plot_x, plot_y):
            cv2.circle(ipm_img, (int(x), int(y)), 2, (0, 255, 0), -1)

    def draw_ackermann_path(self, roi, turning_radius):
        roi_height, roi_width = roi.shape[:2]
        x, y, theta = 0, 0, 0  # meters

        dt = 0.05
        v = 1.0  
        total_steps = 100

        for _ in range(total_steps):
            x += v * np.cos(theta) * dt
            y += v * np.sin(theta) * dt
            theta += (v / turning_radius) * dt

            x_pixel = int(roi_width // 2 + x * self.pixels_per_meter)
            y_pixel = int(roi_height - y * self.pixels_per_meter)

            if 0 <= x_pixel < roi_width and 0 <= y_pixel < roi_height:
                cv2.circle(roi, (x_pixel, y_pixel), 2, (0, 255, 255), -1)
            else:
                break

    def process_frame(self, steering_angle_deg):
        ret, frame = self.cap.read()
        if not ret:
            print("Failed to capture frame.")
            return None

        frame = cv2.resize(frame, (320, 240))
        roi = frame.copy()

        # Apply IPM
        warped = self.get_ipm_transform(roi)
        centers, white_mask = self.detect_lane_center(warped)

        # Lane polynomial (optional for debugging)
        if len(centers) >= 5:
            poly_coeff = self.fit_polynomial(centers)
            self.draw_polynomial(warped, poly_coeff)

            y_eval = 240
            R_curve = self.calculate_curvature(poly_coeff, y_eval)
            print(f"Estimated curvature: {R_curve:.2f} meters")

        # Ackermann path based on actual wheel angle
        turning_radius = self.compute_turning_radius_from_angle(steering_angle_deg)
        self.draw_ackermann_path(roi, turning_radius)

        return frame, warped, white_mask, roi

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def compute_turning_radius_from_angle(self, steering_angle_deg):
        steering_rad = math.radians(steering_angle_deg - 90)
        if abs(math.tan(steering_rad)) < 1e-3:
            return 1e6  
        turning_radius = self.wheelbase / math.tan(steering_rad)
        return turning_radius


if __name__ == "__main__":
    predictor = AckermannPathPredictor()

    # Test mode: pass dummy steering angle
    test_steering_angle = 75  # center angle
    while True:
        frame, warped, mask, roi = predictor.process_frame(test_steering_angle)

        if frame is None:
            break

        cv2.imshow("Original", frame)
        cv2.imshow("IPM", warped)
        cv2.imshow("White Mask", mask)
        cv2.imshow("Ackermann Path", roi)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    predictor.release()
