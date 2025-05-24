import sys
import threading
import random

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QListWidget,
    QHBoxLayout, QVBoxLayout, QListWidgetItem
)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')

        self.bridge = CvBridge() #COnversion cv2.msg imagen y viseversa
        self.latest_frame = None #Iniicializamos el ultimo frame que ha llegado

        #Suscrpción al nodo de la imagén camara 1
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )

        self.app = QApplication(sys.argv)
        self.gui = DashboardGUI(self)
        self.gui.show()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        except Exception as e:
            self.get_logger().error(f"Error al convertir la imagen: {e}")


#Dashboard
class DashboardGUI(QWidget):
    def __init__(self, node: DashboardNode):
        super().__init__()
        self.node = node
        self.setWindowTitle("Conveyor Belt Dashboard")
        self.setGeometry(100, 100, 1200, 600)

        self.box_types = [
            "A caja", "A caja hoyo", "A caja abierta", "B caja", "B caja abierta",
            "B caja hoyo", "C caja", "C caja hoyo", "C caja abierta"
        ]
        self.displayed_count = 0

        # Layout principal horizontal
        main_layout = QHBoxLayout()

        # Layout de cámaras
        video_layout = QVBoxLayout()
        self.video_label_1 = QLabel()
        self.video_label_2 = QLabel()

        for label in [self.video_label_1, self.video_label_2]:
            label.setFixedSize(580, 280)
            label.setAlignment(Qt.AlignCenter)

        # Layouts individuales para cada cámara con su título
        cam1_layout = QVBoxLayout()
        cam1_title = QLabel("Identificación de errores en cajas")
        cam1_title.setAlignment(Qt.AlignCenter)
        cam1_layout.addWidget(cam1_title)
        cam1_layout.addWidget(self.video_label_1)

        cam2_layout = QVBoxLayout()
        cam2_title = QLabel("Lectura de etiquetas")
        cam2_title.setAlignment(Qt.AlignCenter)
        cam2_layout.addWidget(cam2_title)
        cam2_layout.addWidget(self.video_label_2)

        video_layout.addLayout(cam1_layout)
        video_layout.addLayout(cam2_layout)

        main_layout.addLayout(video_layout)

        # Layout de lista y contador
        side_layout = QVBoxLayout()
        self.counter_label = QLabel("Nombres mostrados: 0")
        self.counter_label.setAlignment(Qt.AlignCenter)
        self.object_list = QListWidget()

        side_layout.addWidget(self.counter_label)
        side_layout.addWidget(self.object_list)
        main_layout.addLayout(side_layout)

        self.setLayout(main_layout)

        # Timer de imagen
        self.video_timer = QTimer()
        self.video_timer.timeout.connect(self.update_frames)
        self.video_timer.start(30)

        # Timer de nombres (cada 2 segundos)
        self.name_timer = QTimer()
        self.name_timer.timeout.connect(self.add_random_box)
        self.name_timer.start(2000)

    def update_frames(self):
        if self.node.latest_frame is not None:
            frame = self.node.latest_frame
            h, w, ch = frame.shape
            bytes_per_line = ch * w
            qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pix = QPixmap.fromImage(qt_image)

            scaled_pix_1 = pix.scaled(self.video_label_1.width(), self.video_label_1.height(), Qt.KeepAspectRatio)
            scaled_pix_2 = pix.scaled(self.video_label_2.width(), self.video_label_2.height(), Qt.KeepAspectRatio)

            self.video_label_1.setPixmap(scaled_pix_1)
            self.video_label_2.setPixmap(scaled_pix_2)

    def add_random_box(self):
        name = random.choice(self.box_types)
        self.object_list.addItem(QListWidgetItem(name))
        self.displayed_count += 1
        self.counter_label.setText(f"Nombres mostrados: {self.displayed_count}")


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    try:
        sys.exit(node.app.exec_())
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
