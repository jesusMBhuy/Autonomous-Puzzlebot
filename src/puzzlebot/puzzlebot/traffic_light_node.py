from collections import deque
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rclpy.qos import qos_profile_sensor_data


def show_hsv_debug(event, x, y, flags, param):
    if event == cv2.EVENT_MOUSEMOVE:
        hsv = param
        if y < hsv.shape[0] and x < hsv.shape[1]:
            pixel = hsv[y, x]
            print(f"🧪 HSV at ({x},{y}) = {pixel}")


class TrafficLightDetector(Node):
    def __init__(self):
        super().__init__('traffic_light_detector')

        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.last_color = "unknown"
        self.pub_str = self.create_publisher(String, '/light_state', qos_profile)
        self.pub_int = self.create_publisher(Int32, '/light_state_int', 10)

        self.subscription = self.create_subscription(
            CompressedImage,
            '/video_source/compressed',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.bridge = CvBridge()
        self.detections = {
            'red': deque(maxlen=5),
            'yellow': deque(maxlen=5),
            'green': deque(maxlen=5)
        }

        self.color_to_int = {'red': 1, 'yellow': 2, 'green': 3, 'unknown': 0}

    def image_callback(self, msg: CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        frame_blur = cv2.GaussianBlur(frame, (7, 7), sigmaX=1.5, sigmaY=1.5)
        hsv = cv2.cvtColor(frame_blur, cv2.COLOR_BGR2HSV)

        h, s, v = cv2.split(hsv)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        v_eq = clahe.apply(v)
        hsv = cv2.merge([h, s, v_eq])

        cv2.namedWindow("Camera View")
        cv2.setMouseCallback("Camera View", show_hsv_debug, hsv)

        rangos = {
            'red': [
                cv2.inRange(hsv, (0, 50, 70), (10, 255, 255)),
                cv2.inRange(hsv, (0,  70,  50), (10, 255, 255)),
                cv2.inRange(hsv, (0,  50, 200), (10, 255, 255)),
                cv2.inRange(hsv, (160, 50, 70), (179, 255, 255)),
                cv2.inRange(hsv, (170, 30, 70), (179, 255, 200)),
            ],
            'yellow': [
                cv2.inRange(hsv, (35, 70, 90), (75, 255, 255)),    # amarillo real
                cv2.inRange(hsv, (35, 50, 150), (75, 255, 255)),   # poco saturado
                cv2.inRange(hsv, (45, 40, 200), (75, 255, 255)),    # foco brillante
                cv2.inRange(hsv, (20,  40, 200), (35, 255, 255)),
            ],
            'green': [
                cv2.inRange(hsv, (45, 70, 90), (75, 255, 255)),    # verde real
                cv2.inRange(hsv, (45, 50, 150), (75, 255, 255)),   # poco saturado
                cv2.inRange(hsv, (45, 40, 200), (75, 255, 255)),    # foco brillante
                cv2.inRange(hsv, (40, 60, 100), (85, 255, 255)),  # más amplio y confiable
            ],
        } 

        best_circle = None
        best_score = -1
        min_area, max_area = 30, 300
        circularity_th = 0.8

        for color in ['red', 'yellow', 'green']:
            mask = rangos[color][0]
            for extra_mask in rangos[color][1:]:
                mask = cv2.bitwise_or(mask, extra_mask)

            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
            mask = cv2.medianBlur(mask, 5)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if not (min_area < area < max_area):
                    continue

                perim = cv2.arcLength(cnt, True)
                if perim == 0:
                    continue
                circularity = (4 * np.pi * area) / (perim * perim)
                if circularity < circularity_th:
                    continue

                score = circularity * area
                if score > best_score:
                    best_score = score
                    best_circle = (color, cnt)

        detected_state = "unk"
        if best_circle is not None:
            color, cnt = best_circle
            detected_state = color

            if detected_state == self.last_color:
                return

            self.detections[detected_state].append(1)
            if sum(self.detections[detected_state]) >= 3:
                self.last_color = detected_state
                self.pub_str.publish(String(data=detected_state))
                self.pub_int.publish(Int32(data=self.color_to_int[detected_state]))
                self.get_logger().info(f"✅ Published: {detected_state} ({self.color_to_int[detected_state]})")

        # Mostrar en ventana
        cv2.putText(frame, f"Color: {detected_state.upper()}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0) if detected_state == 'green' else
                    (0, 255, 255) if detected_state == 'yellow' else
                    (0, 0, 255) if detected_state == 'red' else
                    (200, 200, 200), 2)
        cv2.imshow("Camera View", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()