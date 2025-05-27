import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class CameraProcessingNode(Node):
    def __init__(self):
        super().__init__('camera_processing_node')
        #Subscribe to the image topic
        self.camera1_subscription = self.create_subscription(
            Image,
            '/image',
            self.cameras_callback,
            10
        )
        # Create a publisher for processed images
        self.camera1_publisher = self.create_publisher(Image, '/processed_image', 10)
        self.bridge = CvBridge() #Initialize CvBridge for image conversion

    def cameras_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        croped_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
        croped_image = croped_image[0:1080, 640:1280]  # Crop the image coordinates (y1:y2, x1:x2)(1080 x 1920)
        self.camera1_publisher.publish(self.bridge.cv2_to_imgmsg(croped_image, encoding='rgb8'))
        
        
        
def main(args=None):
    rclpy.init(args=args)
    node = CameraProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()