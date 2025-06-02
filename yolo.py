# yolo_adaptado.py
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
        super().__init__('lab9_adaptado')

        # Publisher para la etiqueta detectada (Int16)
        self.publisher_detected = self.create_publisher(Int16, 'detected_object_id', 10)
        self.msg2 = Int16()

        # Publisher para la imagen procesada (sensor_msgs/Image)
        self.publisher_img = self.create_publisher(Image, 'processed_image', 10)
        self.msg_img = Image()

        # Suscripción al tópico 'esquinas' (sensor_msgs/Image entrante)
        self.bridge = CvBridge()
        self.img = np.zeros((480, 640, 3), dtype=np.uint8)  # placeholder inicial
        self.create_subscription(Image, 'esquinas', self.Image_hz, 10
        )

        # Cargamos el modelo YOLO (misma ruta a best.pt)
        self.model = YOLO('src/basic_comms/basic_comms/best.pt')

        # Parámetros de la lógica “GlaxoPruebas”
        self.END_DELAY_FRAMES = 40      # Frames sin detección tras empezar colección para “cerrar caja”
        self.collecting = False
        self.no_detection_count = 0
        self.box_detections = []

        # Temporizador a 0.1 s (10 Hz)
        self.timer_period = 0.1
        self.create_timer(self.timer_period, self.timer_callback)

    def Image_hz(self, msg: Image):
        # Convertimos mensaje ROS Image a OpenCV BGR
        self.img = self.bridge.imgmsg_to_cv2(msg, "rgb8")

    def timer_callback(self):
        # 1) Redimensionar la imagen a 640×480
        frame = cv2.resize(self.img, (640, 480))

        # 2) Recorte central (320×200): 
        #    ancho recortado = 320, alto recortado = 200, centrado horizontal
        h, w = frame.shape[:2]           # 480, 640
        new_w, new_h = 320, 80
        x0 = (w - new_w) // 2            # 160
        cropped = frame[0:200, x0:x0 + new_w]  # [y0=0 : y1=200, x0=160 : x0+320=480]

        # 3) Convertimos a RGB para YOLO
        rgb_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)

        # 4) Ejecutamos YOLO sobre el recorte
        results = self.model(source=rgb_cropped, conf=0.3, imgsz=512, verbose=False)[0]

        # 5) Filtrar detecciones que toquen top (y1==0) o bottom (y2>=180)
        filtered_ids = []
        for coords, cls in zip(results.boxes.xyxy, results.boxes.cls):
            x1, y1, x2, y2 = coords.cpu().numpy().astype(int)
            # Condición de descartar si y1==0 o y2>=180
            if y1 == 0 or y2 >= 180:
                continue
            filtered_ids.append(int(cls.item()))

        # 6) Lógica de acumulación / “cerrar caja” cada 40 frames sin detección
        if filtered_ids:
            # Hay detecciones válidas en este frame
            if not self.collecting:
                self.collecting = True
                self.no_detection_count = 0
                self.box_detections = []
            # Acumulamos todas las IDs válidas de este frame
            self.box_detections.extend(filtered_ids)
            self.no_detection_count = 0
        else:
            # No hay detecciones en este frame
            if self.collecting:
                self.no_detection_count += 1
                if self.no_detection_count >= self.END_DELAY_FRAMES:
                    # “Cerrar caja”: calculamos etiqueta más frecuente
                    if self.box_detections:
                        most_common = int(np.bincount(self.box_detections).argmax())
                    # Publicamos la etiqueta en detected_object_id
                    self.msg2.data = most_common
                    self.publisher_detected.publish(self.msg2)

                    # (Opcional) Si quieres dibujar el texto de la etiqueta más frecuente
                    # sobre la última imagen anotada, podrías hacerlo aquí con:
                    # cv2.putText(annotated_bgr, f"ID_acumulado: {most_common}", (10, 30),
                    #             cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

                    # Reiniciamos para la próxima “caja”
                    self.collecting = False
                    self.no_detection_count = 0
                    self.box_detections.clear()
        # 7) Obtener la imagen anotada por YOLO (bounding boxes + clases)
        #    `results.plot()` regresa una imagen RGB con las anotaciones
        annotated = results.plot()
        # Convertimos a BGR para imshow y para publicar por ROS
        annotated_bgr = cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR)

        # 8) Mostrar en pantalla con OpenCV
        cv2.imshow("YOLO Adaptado (center crop)", annotated_bgr)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # Si presionas 'q', cerramos nodo y ventanas
            rclpy.shutdown()
            cv2.destroyAllWindows()

        # 9) Publicar la imagen procesada en el tópico 'processed_image'
        self.msg_img = self.bridge.cv2_to_imgmsg(annotated_bgr, 'bgr8')
        self.publisher_img.publish(self.msg_img)

    def destroy(self):
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
