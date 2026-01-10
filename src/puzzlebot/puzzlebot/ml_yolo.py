#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import torch
from collections import deque
import time
from ultralytics import YOLO

class ML_Node(Node):
    def __init__(self):
        super().__init__('ml_yolo')

        self.bridge = CvBridge()
        self.last_msg = None
        self.last_detection_time = None
        self.last_detection = None
        self.past_sign = 0

        # Ventana de detección y consistencia
        self.detection_window = deque(maxlen=15)
        self.required_consistency = 5
        self.no_detection_limit = 8  # Máximo de frames sin señal válida antes de limpiar

        # Mapeo de clases
        self.class_map = {'stop': 1, 'straight': 2, 'left': 3, 'right': 4}

        # Área mínima requerida por clase (ajustable)
        self.min_area_by_class = {
            'stop': 100,
            'straight': 400,
            'left': 300,
            'right': 300,
        }

        # Modelo YOLO
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Using device: {self.device}")
        self.model = YOLO("best.pt")
        self.model.to(self.device)
        self.model.amp = True

        # Publicador
        self.sign_pub = self.create_publisher(Int32, '/signal_detected', 10)
        self.subscription = self.create_subscription(
            CompressedImage,
            '/video_source/compressed',
            self.image_callback,
            qos_profile_sensor_data
        )

        # Ventana
        cv2.namedWindow("Traffic Signals", cv2.WINDOW_NORMAL)

        # Timer
        self.timer = self.create_timer(0.3, self.process_latest_image)
        self.get_logger().info("YOLO model initialized")

    def image_callback(self, msg):
        self.last_msg = msg

    def process_latest_image(self):
        if self.last_msg is None:
            return

        msg = self.last_msg
        self.last_msg = None
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)
        if frame is None:
            self.get_logger().warn("⚠️ Imagen inválida.")
            return

        detected = self.detect_sign(frame)
        msg_out = Int32()
        msg_out.data = detected
        self.sign_pub.publish(msg_out)

    def detect_sign(self, frame):
        sign_detected = 0
        disp_frame = frame.copy()
        most_conf = 0
        selected_box = None
        candidate_cls = None

        results = self.model(frame, verbose=False)
        boxes = results[0].boxes
        img_center_x = frame.shape[1] // 2

        for box in boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cls_id = int(box.cls[0])
            cls_name = self.model.names[cls_id]

            bbox_center_x = (x1 + x2) // 2
            bbox_area = (x2 - x1) * (y2 - y1)

            # Filtro por clase válida
            if cls_name not in self.class_map:
                continue

            # Filtro por área mínima
            min_area = self.min_area_by_class.get(cls_name, 9000)
            if bbox_area < min_area:
                self.get_logger().info(f"❌ '{cls_name}' descartada por área: {bbox_area} < {min_area}")
                continue

            # Filtro espacial
            if cls_name == 'left' and bbox_center_x > img_center_x:
                continue
            if cls_name == 'right' and bbox_center_x < img_center_x:
                continue

            # Filtro por confianza
            if conf > 0.6 and conf > most_conf:
                most_conf = conf
                selected_box = (x1, y1, x2, y2)
                candidate_cls = cls_name

        # Si se encontró una clase válida
        if candidate_cls:
            self.detection_window.append(candidate_cls)
        else:
            self.detection_window.append('none')  # Para permitir limpieza

        # Conteo de detecciones en la ventana
        counts = {s: self.detection_window.count(s) for s in set(self.detection_window) if s != 'none'}
        if counts:
            most_common, count = max(counts.items(), key=lambda x: x[1])
            if count >= self.required_consistency:
                self.last_detection = most_common
                self.last_detection_time = time.time()
                sign_detected = self.class_to_id(most_common)
                self.past_sign = sign_detected
                self.get_logger().info(f"✅ Señal CONSISTENTE detectada: {most_common} (x{count})")
            else:
                sign_detected = self.past_sign
        else:
            # Si hay demasiados 'none', limpiar estado
            if self.detection_window.count('none') >= self.no_detection_limit:
                self.get_logger().info("⚠️ Limpiando señal por detecciones inconsistentes.")
                self.past_sign = 0
                self.last_detection = None
            sign_detected = self.past_sign

        # Dibujar en pantalla
        status = f"Señal: {self.last_detection or 'Ninguna'} | ID: {sign_detected}"
        cv2.putText(disp_frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        if selected_box:
            x1, y1, x2, y2 = selected_box
            cv2.rectangle(disp_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(disp_frame, f"{candidate_cls} {most_conf:.2f}", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("Traffic Signals", cv2.resize(disp_frame, (320, 240)))
        cv2.waitKey(1)

        return sign_detected

    def class_to_id(self, cls_name):
        return self.class_map.get(cls_name, 0)

    def destroy_node(self):
        self.get_logger().info("Cerrando nodo...")
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ML_Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()

if __name__ == '__main__':
    main()
