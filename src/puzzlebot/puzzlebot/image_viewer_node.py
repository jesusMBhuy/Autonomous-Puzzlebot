#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
from collections import deque

class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')

        self.bridge = CvBridge()
        self.frame_count = 0
        self.last_cx = None
        self.error = 0.0
        self.master_has_control = False

        self.prev_error = 0.0
        self.integral_angle = 0.0
        self.last_control_time = time.time()

        self.vel_left = 0.0
        self.vel_right = 0.0
        self.prev_err_l = 0.0
        self.int_l = 0.0
        self.prev_err_r = 0.0
        self.int_r = 0.0

        self.dt = 0.05
        self.tolerance = 2

        self.kp_ang = 0.00107
        self.ki_ang = 0.0

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.line_status_pub = self.create_publisher(Bool, '/line_lost', qos_profile_system_default)

        self.sub_wl = self.create_subscription(Float32, '/VelocityEncL', self.wl_callback, qos_profile_system_default)
        self.sub_wr = self.create_subscription(Float32, '/VelocityEncR', self.wr_callback, qos_profile_system_default)
        self.sub_img = self.create_subscription(CompressedImage, '/video_source/compressed', self.image_callback, qos_profile_sensor_data)
        self.sub_priority = self.create_subscription(Bool, '/control_priority', self.priority_callback, qos_profile_system_default)

        cv2.namedWindow("Ajustes")
        for name, init, maxval in [
            ("Threshold bin", 230, 255), ("Canny min", 140, 255), ("Canny max", 200, 255),
            ("Área mínima", 250, 2000), ("Morph op", 1, 2),
            ("Kp_L", 20, 100), ("Ki_L", 0, 100), ("Kd_L", 0, 100),
            ("Kp_R", 20, 100), ("Ki_R", 0, 100), ("Kd_R", 0, 100),
            ("Velocidad cm/s", 20, 100)
        ]:
            cv2.createTrackbar(name, "Ajustes", init, maxval, lambda x: None)

        for win in ["Imagen Original", "Combinado final", "Vertical Only"]:
            cv2.namedWindow(win, cv2.WINDOW_NORMAL)

        self.timer = self.create_timer(self.dt, self.control_loop)

    def wl_callback(self, msg: Float32):
        self.vel_left = msg.data

    def wr_callback(self, msg: Float32):
        self.vel_right = msg.data

    def priority_callback(self, msg: Bool):
        self.master_has_control = msg.data

    def image_callback(self, msg: CompressedImage):
        if self.frame_count % 2 != 0:
            self.frame_count += 1
            return
        self.frame_count += 1

        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        sharpened = cv2.filter2D(blurred, -1, np.array([[-1, -1, -1], [-1, 10, -1], [-1, -1, -1]]))

        h, w = sharpened.shape
        roi_y = int(h * 0.7)
        roi = sharpened[roi_y:, :]

        rect_width = int(w * 0.75)
        x_start = (w - rect_width) // 2
        mask = np.zeros_like(roi)
        mask[:, x_start:x_start + rect_width] = 255
        roi = cv2.bitwise_and(roi, mask)

        thresh = cv2.getTrackbarPos("Threshold bin", "Ajustes")
        cmin = cv2.getTrackbarPos("Canny min", "Ajustes")
        cmax = cv2.getTrackbarPos("Canny max", "Ajustes")
        min_area = cv2.getTrackbarPos("Área mínima", "Ajustes")
        morph_op = cv2.getTrackbarPos("Morph op", "Ajustes")

        edges = cv2.Canny(roi, cmin, cmax)
        _, binary = cv2.threshold(roi, thresh, 255, cv2.THRESH_BINARY_INV)
        combined_pre = cv2.bitwise_or(edges, binary)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        if morph_op == 0:
            combined_post = cv2.dilate(combined_pre, kernel, iterations=2)
        elif morph_op == 1:
            combined_post = cv2.erode(combined_pre, kernel, iterations=2)
        else:
            combined_post = cv2.morphologyEx(combined_pre, cv2.MORPH_CLOSE, kernel)

        vert_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 50))
        vertical_only = cv2.morphologyEx(combined_post, cv2.MORPH_OPEN, vert_kernel)

        contours, _ = cv2.findContours(vertical_only, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cx_list = []
        for c in contours:
            x, y, cw, ch = cv2.boundingRect(c)
            if ch < cw * 2:
                continue
            M = cv2.moments(c)
            if M['m00'] > min_area:
                cx_list.append(int(M['m10'] / M['m00']))

        if cx_list:
            self.last_cx = sorted(cx_list)[len(cx_list)//2]
            self.line_status_pub.publish(Bool(data=False))
        else:
            self.line_status_pub.publish(Bool(data=True))

        if self.last_cx is not None:
            center_x = w // 2
            self.error = center_x - self.last_cx
            if abs(self.error) < self.tolerance:
                self.error = 0.0

        disp = frame.copy()
        if self.last_cx is not None:
            cy = roi_y + int((h - roi_y) / 2)
            cv2.circle(disp, (self.last_cx, cy), 5, (0, 255, 0), -1)
            cv2.line(disp, (w // 2, cy), (self.last_cx, cy), (0, 255, 255), 2)
        cv2.imshow("Imagen Original", cv2.resize(disp, (640, 480)))
        cv2.imshow("Combinado final", cv2.resize(combined_post, (640, 240)))
        cv2.imshow("Vertical Only", cv2.resize(vertical_only, (640, 240)))
        cv2.waitKey(1)

    def control_loop(self):
        '''
        if self.vel_left == 0.0 and self.vel_right == 0.0:
            self.get_logger().warn("⏳ Esperando feedback de encoders...")
            return
        '''
        if self.master_has_control:
            return

        now = time.time()
        dt = now - self.last_control_time or self.dt
        self.last_control_time = now

        setpoint = cv2.getTrackbarPos("Velocidad cm/s", "Ajustes")

        self.integral_angle += self.error * dt
        deriv = (self.error - self.prev_error) / dt
        self.prev_error = self.error
        u_ang = self.kp_ang * self.error + self.ki_ang * self.integral_angle + 0.0 * deriv
        ang_z = np.clip(u_ang, -0.35, 0.35)

        err_l = setpoint - self.vel_left
        self.int_l += err_l * dt
        der_l = (err_l - self.prev_err_l) / dt
        self.prev_err_l = err_l
        u_l = np.clip((cv2.getTrackbarPos("Kp_L", "Ajustes")/100 * err_l + cv2.getTrackbarPos("Ki_L", "Ajustes")/100 * self.int_l + cv2.getTrackbarPos("Kd_L", "Ajustes")/100 * der_l)/100, -0.25, 0.25)

        err_r = setpoint - self.vel_right
        self.int_r += err_r * dt
        der_r = (err_r - self.prev_err_r) / dt
        self.prev_err_r = err_r
        u_r = np.clip((cv2.getTrackbarPos("Kp_R", "Ajustes")/100 * err_r + cv2.getTrackbarPos("Ki_R", "Ajustes")/100 * self.int_r + cv2.getTrackbarPos("Kd_R", "Ajustes")/100 * der_r)/100, -0.25, 0.25)

        if abs(u_l) > 0.2 or abs(u_r) > 0.2:
            self.get_logger().warn("🛑 Comando PID fuera de límites seguros. Cancelando envío.")
            return

        twist = Twist()
        twist.linear.x = (u_l + u_r) / 2.0
        twist.angular.z = ang_z
        self.cmd_pub.publish(twist)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()