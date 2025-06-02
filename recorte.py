# recorte_publicador.py
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RecortePublisher(Node):
    def __init__(self):
        super().__init__('recorte_publisher')

        self.publisher = self.create_publisher(Image, 'imagen_recortada', 10)
        self.bridge = CvBridge()
        self.img = np.zeros((480, 640, 3), dtype=np.uint8)

        # Suscripción a la cámara original
        self.create_subscription(Image, '/camera1/image', self.callback_image, 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def callback_image(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def timer_callback(self):
        frame = cv2.resize(self.img, (640, 480))
        new_w, new_h = 320, 80
        x0 = (640 - new_w) // 2
        cropped = frame[0:200, x0:x0 + new_w]

        cropped_msg = self.bridge.cv2_to_imgmsg(cropped, 'rgb8')
        self.publisher.publish(cropped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = RecortePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()