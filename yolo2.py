# yolo_adaptado_con_inventario.py
import os
import csv
from datetime import datetime

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
        super().__init__('lab9_adaptado_inventario')

        # -------------------------------------------------------------
        # 1) Publishers: etiqueta detectada (Int16) y imagen procesada
        # -------------------------------------------------------------
        self.publisher_detected = self.create_publisher(Int16, 'detected_object_id', 10)
        self.msg2 = Int16()

        self.publisher_img = self.create_publisher(Image, 'processed_image', 10)
        self.msg_img = Image()

        # -------------------------------------------------------------
        # 2) Suscripción al tópico 'esquinas' (llega sensor_msgs/Image)
        # -------------------------------------------------------------
        self.bridge = CvBridge()
        # Inicialmente, un placeholder para la imagen
        self.img = np.zeros((480, 640, 3), dtype=np.uint8)
        self.create_subscription(
            Image,
            'esquinas',
            self.Image_hz,
            10
        )

        # -------------------------------------------------------------
        # 3) Modelo YOLO (misma ruta a tu best.pt original)
        # -------------------------------------------------------------
        self.model = YOLO('src/basic_comms/basic_comms/best.pt')

        # -------------------------------------------------------------
        # 4) Parámetros de la lógica “GlaxoPruebas” modificada
        # -------------------------------------------------------------
        self.END_DELAY_FRAMES = 40      # Frames sin detección tras empezar colección para “cerrar caja”
        self.collecting = False
        self.no_detection_count = 0
        self.box_detections = []

        # -------------------------------------------------------------
        # 5) Inventario general: acumular conteo total por etiqueta
        # -------------------------------------------------------------
        #   key = etiqueta (int), value = número total de cajas detectadas (sumadas en cada evento)
        self.total_counts = {}

        # -------------------------------------------------------------
        # 6) Archivo CSV para inventario de eventos
        # -------------------------------------------------------------
        # Nombre del archivo CSV
        self.csv_filename = 'detections_inventory.csv'
        # Si el archivo no existe, lo creamos y escribimos la cabecera
        if not os.path.isfile(self.csv_filename):
            with open(self.csv_filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['timestamp', 'label_most_common', 'count_in_event'])
                # Formato:
                #   timestamp         : YYYY-MM-DD HH:MM:SS
                #   label_most_common : entero (ID)
                #   count_in_event    : cuántas cajas se acumularon en este “cierre de caja”

        # -------------------------------------------------------------
        # 7) Temporizador a 0.1 s (10 Hz)
        # -------------------------------------------------------------
        self.timer_period = 0.1
        self.create_timer(self.timer_period, self.timer_callback)

    def Image_hz(self, msg: Image):
        # Convertir mensaje ROS Image a OpenCV BGR
        # Nota: asumir que la imagen entrante viene en “rgb8” (según tu última versión)
        self.img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

    def timer_callback(self):
        # -------------------------------------------------------------
        # 1) Redimensionar imagen a 640×480
        # -------------------------------------------------------------
        frame = cv2.resize(self.img, (640, 480))

        # -------------------------------------------------------------
        # 2) Recorte central (320×200)
        # -------------------------------------------------------------
        h, w = frame.shape[:2]           # 480, 640
        new_w, new_h = 320, 80
        x0 = (w - new_w) // 2            # 160
        cropped = frame[0:200, x0:x0 + new_w]  # [y=0:200, x=160:480]

        # -------------------------------------------------------------
        # 3) Convertir a RGB para YOLO
        # -------------------------------------------------------------
        rgb_cropped = cv2.cvtColor(cropped, cv2.COLOR_BGR2RGB)

        # -------------------------------------------------------------
        # 4) Ejecutar YOLO sobre el recorte
        # -------------------------------------------------------------
        results = self.model(source=rgb_cropped, conf=0.3, imgsz=512, verbose=False)[0]

        # -------------------------------------------------------------
        # 5) Filtrar detecciones que toquen top (y1==0) o bottom (y2>=180)
        # -------------------------------------------------------------
        filtered_ids = []
        for coords, cls in zip(results.boxes.xyxy, results.boxes.cls):
            x1, y1, x2, y2 = coords.cpu().numpy().astype(int)
            # Condición: descartar si y1 == 0 (toca top) o y2 >= 180 (toca bottom)
            if y1 == 0 or y2 >= 180:
                continue
            filtered_ids.append(int(cls.item()))

        # -------------------------------------------------------------
        # 6) Lógica de acumulación / “cerrar caja” cada 40 frames sin detección
        # -------------------------------------------------------------
        if filtered_ids:
            # Hay detecciones válidas en este frame
            if not self.collecting:
                # Empezamos nueva “colección”
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
                    # ---------- “Cerrar caja”: calculamos etiqueta más frecuente ----------
                    if self.box_detections:
                        most_common = int(np.bincount(self.box_detections).argmax())
                        count_in_event = len(self.box_detections)
                    else:
                        # Por seguridad, aunque box_detections nunca esté vacío cuando collecting=True
                        most_common = None
                        count_in_event = 0

                    # ------ Publicar la etiqueta más frecuente en detected_object_id ------
                    if most_common is not None:
                        self.msg2.data = most_common
                        self.publisher_detected.publish(self.msg2)

                        # ------ Actualizar inventario total ------
                        prev_total = self.total_counts.get(most_common, 0)
                        self.total_counts[most_common] = prev_total + count_in_event

                        # ------ Registrar en el CSV el evento “cierre de caja” ------
                        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                        with open(self.csv_filename, 'a', newline='') as csvfile:
                            writer = csv.writer(csvfile)
                            writer.writerow([timestamp, most_common, count_in_event])

                    # Limpiar estado para la siguiente caja
                    self.collecting = False
                    self.no_detection_count = 0
                    self.box_detections.clear()

        # -------------------------------------------------------------
        # 7) Obtener la imagen anotada por YOLO (bounding boxes + clases)
        # -------------------------------------------------------------
        annotated = results.plot()  # Regresa una imagen RGB con las anotaciones
        # Convertir a BGR para imshow y para publicar por ROS
        annotated_bgr = cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR)

        # -------------------------------------------------------------
        # 8) Mostrar en pantalla con OpenCV
        # -------------------------------------------------------------
        cv2.imshow("YOLO Adaptado (center crop)", annotated_bgr)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()
            cv2.destroyAllWindows()

        # -------------------------------------------------------------
        # 9) Publicar la imagen procesada en 'processed_image'
        # -------------------------------------------------------------
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
