import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

class My_Subscriber(Node):
    def __init__(self):
        super().__init__('Yolo_node')

        # Publisher para la etiqueta detectada (Int16) 
        self.publisher_detected = self.create_publisher(Int16, 'detected_object_id', 10)
        self.msg2 = Int16()

        # Publisher para la imagen procesada (sensor_msgs/Image)
        self.publisher_img = self.create_publisher(Image, 'processed_image', 10)
        self.msg_img = Image()
        self.bridge = CvBridge()

        # Cargamos el modelo YOLO
        self.model = YOLO('best_12v_0_74.pt')

        # Captura de video desde cámara física
        self.cap = cv2.VideoCapture(4)
        if not self.cap.isOpened():
            self.get_logger().error('No se pudo abrir la cámara en /dev/video4')
            rclpy.shutdown()

        # Parámetros de la lógica GlaxoPruebas
        self.END_DELAY_FRAMES = 40
        self.collecting = False
        self.no_detection_count = 0
        self.box_detections = []

        # Temporizador a 0.1 s (10 Hz)
        self.timer_period = 0.1
        self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("No se pudo leer frame de la cámara")
            return

        # 1) Redimensionar a 640×480
        frame = cv2.resize(frame, (640, 480))

        # 2) Recorte central (320x200)
        h, w = frame.shape[:2]
        new_w, new_h = 320, 200
        x0 = (w - new_w) // 2
        cropped = frame[0:200, x0:x0 + new_w]

        # 3) Convertimos a RGB
        #rgb_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)

        # 4) YOLO
        results = self.model(source=cropped, conf=0.40 , verbose=False)[0]

        # 5) Filtro top-bottom
        filtered_ids = []
        for coords, cls in zip(results.boxes.xyxy, results.boxes.cls):
            x1, y1, x2, y2 = coords.cpu().numpy().astype(int)
            if y1 == 0 or y2 >= 180:
                continue
            filtered_ids.append(int(cls.item()))

        # 6) Lógica de conteo
        if filtered_ids:
            if not self.collecting:
                self.collecting = True
                self.no_detection_count = 0
                self.box_detections = []
            self.box_detections.extend(filtered_ids)
            self.no_detection_count = 0
        else:
            if self.collecting:
                self.no_detection_count += 1
                if self.no_detection_count >= self.END_DELAY_FRAMES:
                    if self.box_detections:
                        most_common = int(np.bincount(self.box_detections).argmax())
                        self.msg2.data = most_common
                        self.publisher_detected.publish(self.msg2)
                    self.collecting = False
                    self.no_detection_count = 0
                    self.box_detections.clear()

        # 7) Imagen anotada
        annotated = results.plot()
        annotated_bgr = cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR)

        # 8) Mostrar
        #cv2.imshow("YOLO Adaptado (center crop)", annotated)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()
            cv2.destroyAllWindows()

        # 9) Publicar imagen en tópico
        self.msg_img = self.bridge.cv2_to_imgmsg(annotated_bgr,'rgb8')
        self.publisher_img.publish(self.msg_img)

    def destroy(self):
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = My_Subscriber()
    rclpy.spin(node)
    node.destroy()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
