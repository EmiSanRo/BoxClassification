import sys #System-specific parameters and functions
import threading #Creation and management of thread
import random #Generate random numbers

import rclpy #ROS2 library
from rclpy.node import Node #ROS2 nodes

from sensor_msgs.msg import Image #Image message type
from cv_bridge import CvBridge #Cv2 to ROS2 msg convertion

import cv2 #Computer vision

#PyQT modules for the GUI
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QListWidget,
    QHBoxLayout, QVBoxLayout, QListWidgetItem
)
from PyQt5.QtGui import QImage, QPixmap #Images handling
from PyQt5.QtCore import QTimer, Qt 


class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')

        self.bridge = CvBridge() #CV2 and ROS2 images conversion
        self.latest_frame = None #Latest captured frame stored initialization

        #Subscription to /Image topic published by Cam2Img
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )

        self.app = QApplication(sys.argv) #GUI Initialize
        self.gui = DashboardGUI(self) #DashboardGUI object
        self.gui.show() #Show interface
    
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') #ROS2 image converted to a format compatible with cv2
            self.latest_frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB) #Last recorded frame is stored in latest_frame
        except Exception as e:
            self.get_logger().error(f"Error al convertir video") #Error msg


#Dashboard
class DashboardGUI(QWidget):
    def __init__(self, node: DashboardNode):
        super().__init__() 
        self.node = node 
        self.setWindowTitle("Conveyor Belt Dashboard") #Window title
        self.setGeometry(100, 100, 1200, 600) #Position and size of the window
        
        #Box types to be selected
        self.box_types = [
            "A caja", "A caja hoyo", "A caja abierta", "B caja", "B caja abierta",
            "B caja hoyo", "C caja", "C caja hoyo", "C caja abierta"
        ]
        self.displayed_count = 0 #Initialize the count of objects

        # Main layout, horizontal(cameras left, displayed objects right)
        main_layout = QHBoxLayout()

        # Camera layout,vertical
        video_layout = QVBoxLayout()
        self.video_label_1 = QLabel()
        self.video_label_2 = QLabel()

        for label in [self.video_label_1, self.video_label_2]:
            label.setFixedSize(580, 280) 
            label.setAlignment(Qt.AlignCenter)

        # Box error identification via yolo camera
        cam1_layout = QVBoxLayout()
        cam1_title = QLabel("Identificación de errores en cajas") 
        cam1_title.setAlignment(Qt.AlignCenter)
        cam1_layout.addWidget(cam1_title)
        cam1_layout.addWidget(self.video_label_1)

        #arUco camera
        cam2_layout = QVBoxLayout()
        cam2_title = QLabel("Lectura de etiquetas")
        cam2_title.setAlignment(Qt.AlignCenter)
        cam2_layout.addWidget(cam2_title)
        cam2_layout.addWidget(self.video_label_2)
        
        #Add camera layouts to the main layout
        video_layout.addLayout(cam1_layout)
        video_layout.addLayout(cam2_layout)

        main_layout.addLayout(video_layout)

        # Layout object list and counter (vertical)
        side_layout = QVBoxLayout()
        self.counter_label = QLabel("Cajas contadas: 0") #Initialize the visual counter label
        self.counter_label.setAlignment(Qt.AlignCenter) 
        self.object_list = QListWidget()

        side_layout.addWidget(self.counter_label)
        side_layout.addWidget(self.object_list)
        main_layout.addLayout(side_layout)

        self.setLayout(main_layout)

        # Image timer to update video frames and connct it to the method update_frames
        self.video_timer = QTimer()
        self.video_timer.timeout.connect(self.update_frames)
        self.video_timer.start(30) #30ms interval

        # Add box timer
        self.name_timer = QTimer()
        self.name_timer.timeout.connect(self.add_box)
        self.name_timer.start(2000) #2 seconds interval

    #Method to update the images shown in the camera labels
    def update_frames(self):
        if self.node.latest_frame is not None: #Check if the latest frame exist
            frame = self.node.latest_frame  #Save the latest frame from ROS2 node
            h, w, ch = frame.shape #Save the image info
            bytes_per_line = ch * w
            qt_image = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888) #Convert the image to a QImage
            pix = QPixmap.fromImage(qt_image) #Convert the QImage to a QPixMap

            scaled_pix_1 = pix.scaled(self.video_label_1.width(), self.video_label_1.height(), Qt.KeepAspectRatio)
            scaled_pix_2 = pix.scaled(self.video_label_2.width(), self.video_label_2.height(), Qt.KeepAspectRatio)

            self.video_label_1.setPixmap(scaled_pix_1)
            self.video_label_2.setPixmap(scaled_pix_2)

    #Function to insert the detected boxes to the list
    def add_box(self):
        name = random.choice(self.box_types) #Receive object
        self.object_list.addItem(QListWidgetItem(name)) #Add box to the list
        self.displayed_count += 1 #Sum 1
        self.counter_label.setText(f"Cajas contadas: {self.displayed_count}")


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
