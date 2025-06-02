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

        # Publicador para la etiqueta detectada
        self.publisher_detected = self.create_publisher(Int16, 'detected_object_id', 10)
        self.msg2 = Int16()

        # Suscripción al tópico 'esquinas' (llega mensaje sensor_msgs/Image)
        self.bridge = CvBridge()
        self.img = np.zeros((480, 640, 3), dtype=np.uint8)  # placeholder inicial
        self.create_subscription(
            Image,
            'esquinas',
            self.Image_hz,
            10
        )

        # Parámetros para la lógica de “glaxo”:
        self.model = YOLO('src/basic_comms/basic_comms/best.pt')  # Mismo best.pt
        self.END_DELAY_FRAMES = 40  # frames sin detección para “cerrar caja”
        self.FRAME_PERIOD = 1.0 / 60.0  # no se usa explícito, pero lo dejamos como referencia
        self.collecting = False
        self.no_detection_count = 0
        self.box_detections = []

        # Temporizador a 0.1 s (10 Hz)
        self.timer_period = 0.1
        self.create_timer(self.timer_period, self.timer_callback)

    def Image_hz(self, msg: Image):
        # Recibimos imagen desde ROS, la convertimos a CV BGR
        self.img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def timer_callback(self):
        # 1) Resize a 640×480
        frame = cv2.resize(self.img, (640, 480))

        # 2) Recorte central: ancho 320, alto 200 (misma lógica de GlaxoPruebas)
        h, w = frame.shape[:2]           # h=480, w=640
        new_w, new_h = 320, 80
        x0 = (w - new_w) // 2            # (640-320)//2 = 160
        # y0 = new_h (80), pero recortamos de 0:200
        cropped = frame[0:200, x0:x0 + new_w]

        # 3) Convertimos BGR→RGB para YOLO
        rgb_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)

        # 4) Ejecutamos YOLO sobre el recorte
        results = self.model(source=rgb_cropped, conf=0.3, imgsz=512, verbose=False)[0]

        # 5) Filtrar detecciones que toquen top/bottom
        #    frame_height para bottom: 180 (igual al código Glaxo)
        filtered_ids = []
        for coords, cls in zip(results.boxes.xyxy, results.boxes.cls):
            x1, y1, x2, y2 = coords.cpu().numpy().astype(int)
            # Si y1==0 (top) o y2>=180 (bottom), ignorar
            if y1 == 0 or y2 >= 180:
                continue
            filtered_ids.append(int(cls.item()))
        frame_ids = filtered_ids

        # 6) Lógica de acumulación / cierre de caja:
        if frame_ids:
            # Sí hay detecciones válidas en este frame
            if not self.collecting:
                self.collecting = True
                self.no_detection_count = 0
                self.box_detections = []
            # Agregar todas las IDs detectadas (en Glaxo solo extendían varios IDs, aquí agregamos el primero)
            # Si prefieres todos, usa box_detections.extend(frame_ids). 
            # Mantendré 'extend' para acercarme más a Glaxo.
            self.box_detections.extend(frame_ids)
            self.no_detection_count = 0
        else:
            # No hubo detecciones en este frame
            if self.collecting:
                self.no_detection_count += 1
                if self.no_detection_count >= self.END_DELAY_FRAMES:
                    # “Cerrar caja”: calculamos la etiqueta más frecuente
                    if self.box_detections:
                        most_common = int(np.bincount(self.box_detections).argmax())
                    else:
                        most_common = 9  # si por alguna razón no hay detecciones acumuladas
                    # Publicamos la etiqueta
                    self.msg2.data = most_common
                    self.publisher_detected.publish(self.msg2)
                    # Reiniciamos estado para próxima caja
                    self.collecting = False
                    self.no_detection_count = 0
                    self.box_detections.clear()
            else:
                # Si ni siquiera está en modo collecting, publicamos 9 inmediatamente
                self.msg2.data = 9
                self.publisher_detected.publish(self.msg2)

        # 7) Mostrar por pantalla la imagen recortada con detecciones (plot de YOLO)
        #    `results.plot()` ya está sobre la copia interna, así que:
        annotated = results.plot()  # devuelve una copia RGB o BGR?
        # Ultralitycs normalmente regresa RGB, pero OpenCV espera BGR para imshow:
        annotated_bgr = cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR)
        cv2.imshow("YOLO Adaptado (center crop)", annotated_bgr)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # Si presionan 'q', cerramos todo
            rclpy.shutdown()
            cv2.destroyAllWindows()

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
