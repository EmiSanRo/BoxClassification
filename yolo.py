#PARA EL PROCESO DE IMAGENES
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
from matplotlib import colors
import cv2
#PARA LOS TOPICOS
import math 
import numpy as np 
import rclpy 
from rclpy import qos
from rclpy.node import Node
from std_msgs.msg import Float32, Int16
from sensor_msgs.msg import Image
from skimage.io import imread, imshow
from skimage.color import rgb2hsv
from cv_bridge import CvBridge
from ultralytics import YOLO



class My_Subscriber(Node):
   def __init__(self):
       super().__init__('lab9')
       #Mensajes que se publicaran
       self.msg1 = Image()
       self.counter = 100
       self.img =  np.zeros((1000, 1000, 3), dtype=np.uint8)
       self.img_cv2 = np.zeros((1000, 1000, 3), dtype=np.uint8)
       self.bridge = CvBridge()
       self.detected_objects = []
       self.A_counter = 0
       self.detections = []
       self.msg2= Int16()
       self.img_detected = 9




       #Subscripciones al topico de la imagen 
       self.video_source = self.create_subscription(Image, 'esquinas', self.Image_hz, qos.qos_profile_sensor_data)

       self.publisher_detected = self.create_publisher(Int16, 'detected_object_id', 10)



       self.timer_period= .1
       self.create_timer(self.timer_period,self.timer_callback) 

       # ### Sharpening


   def Image_hz(self, msg):
       self.img = self.bridge.imgmsg_to_cv2(msg, "rgb8") 
       #self.img_cv2 = self.bridge.imgmsg_to_cv2(msg, "rgb8")
       


   def timer_callback(self):
       image =self.img_cv2
       self.img_cv2= cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
       #self.rotate = cv2.rotate(self.img_cv2, cv2.ROTATE_180)
       model=YOLO('src/basic_comms/basic_comms/best.pt')
       print(self.img_detected)



     #self.publisher_video.publish(self.bridge.cv2_to_imgmsg(np.array(results)))
      # self.publisher_MC.publish(edges(x=float(self.cX), y=float(self.cY), z=0.0))
       results = model (source=self.img_cv2, show= False, conf=0.3, save=False)
       self.detected_objs = []
       for result in results:
        for obj in result.boxes:
            class_id = int(obj.cls.item())  # Convert tensor to int
            conf = float(obj.conf.item())   # Convert tensor to float
            self.detected_objs.append((class_id))


        if len(self.detected_objs) != 0:
           self.detections.append(self.detected_objs[0])
           print(self.detections)
           self.A_counter += 1
           if self.A_counter == 10:
             #flattened_detections = [class_id for detection in self.detected_objects for class_id, _ in detection]
             counts = np.bincount(self.detections)
             temp =np.argmax(counts)
             
             self.img_detected = int(temp)
             self.msg2.data = self.img_detected
             self.publisher_detected.publish(self.msg2)
             self.A_counter = 0
             self.detections=[]
        elif len(self.detected_objs) == 0:
              self.img_detected = 9
              self.msg2.data= self.img_detected
              self.publisher_detected.publish(self.msg2)

            
           

       



def main (args = None):
    rclpy.init(args=args)
    m_s = My_Subscriber()
    rclpy.spin(m_s)
    m_s.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()