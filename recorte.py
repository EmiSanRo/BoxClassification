# recorte_publicador_camera.py
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RecortePublisherCam(Node):
    def __init__(self):
        super().__init__('recorte_publisher_camera')

        self.publisher = self.create_publisher(Image, 'imagen_recortada', 10)
        self.bridge = CvBridge()

        # Captura de cámara local (0 = webcam por defecto)
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error('No se pudo abrir la cámara.')
            exit()

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('No se pudo leer un frame de la cámara.')
            return

        frame = cv2.resize(frame, (640, 480))
        new_w, new_h = 320, 80
        x0 = (640 - new_w) // 2
        cropped = frame[0:200, x0:x0 + new_w]

        # Convertir BGR (OpenCV) a RGB (para 'rgb8')
        cropped_rgb = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)

        msg = self.bridge.cv2_to_imgmsg(cropped_rgb, encoding='rgb8')
        self.publisher.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RecortePublisherCam()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
