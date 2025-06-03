import time
import cv2
import numpy as np
import csv
import sys
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
from collections import Counter


class My_Subscriber(Node):
    def __init__(self):
        super().__init__('Yolo_node')

        # ——— 1) Publicadores ROS ———
        self.publisher_detected = self.create_publisher(Int16, 'detected_object_id', 10)
        self.msg2 = Int16()
        self.publisher_img = self.create_publisher(Image, 'processed_image', 10)
        self.msg_img = Image()
        self.bridge = CvBridge()

        # ——— 2) Modelo YOLO ———
        self.model = YOLO('best_12v_0_74.pt')

        # ——— 3) Apertura de cámara ———
        self.cap = cv2.VideoCapture(4)
        if not self.cap.isOpened():
            self.get_logger().error('No se pudo abrir la cámara en /dev/video4')
            rclpy.shutdown()
            return

        # ——— 4) Parámetros de conteo por ventana ———
        self.END_DELAY_FRAMES = 40
        self.collecting = False
        self.no_detection_count = 0
        self.box_detections = []

        # ——— 5) Counter para conteo de ventanas ———
        # Cada vez que cierre una ventana, incrementamos:
        #   window_counts[most_common] += 1
        self.window_counts = Counter()

        # ——— 6) Temporizador a 0.1 s (10 Hz) ———
        self.timer_period = 0.1
        self.create_timer(self.timer_period, self.timer_callback)

        # ——— 7) Hilo para apagar con “q” en la terminal ———
        self._stop_thread = False
        thread = threading.Thread(target=self.listen_keyboard, daemon=True)
        thread.start()

        # ——— 8) Primer guardado del CSV (vacío, con solo cabecera) ———
        self.write_csv()

    def listen_keyboard(self):
        """
        Lee líneas de la entrada estándar. Si recibe “q”, apaga el nodo.
        """
        while rclpy.ok() and not self._stop_thread:
            try:
                line = sys.stdin.readline().strip()
            except Exception:
                break
            if line.lower() == 'q':
                self.get_logger().info("Se recibió 'q' en terminal: apagando nodo.")
                rclpy.shutdown()
                break

    def write_csv(self):
        """
        Reescribe el archivo counts.csv con:
            etiqueta, conteo_total
        para cada etiqueta presente en self.window_counts.
        """
        try:
            with open('counts.csv', 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['etiqueta', 'conteo_total'])
                for label_id, cnt in self.window_counts.items():
                    writer.writerow([label_id, cnt])
        except Exception as e:
            self.get_logger().error(f"Error al escribir counts.csv: {e}")

    def timer_callback(self):
        # 1) Leer un frame de la cámara
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("No se pudo leer frame de la cámara")
            return

        # 2) Redimensionar a 640×480
        frame = cv2.resize(frame, (640, 480))

        # 3) Recorte central (320×200)
        h, w = frame.shape[:2]
        new_w, new_h = 320, 200
        x0 = (w - new_w) // 2
        cropped = frame[0:200, x0:x0 + new_w]

        # 4) Inferencia YOLO sobre el recorte
        results = self.model(source=cropped, conf=0.40, verbose=False)[0]

        # 5) Filtrado top/bottom
        filtered_ids = []
        for coords, cls in zip(results.boxes.xyxy, results.boxes.cls):
            x1, y1, x2, y2 = coords.cpu().numpy().astype(int)
            # Descarta si toca el top (y1 == 0) o sale por abajo (y2 >= 180) o lateral (x2 >= 300)
            if y1 == 0 or y2 >= 180 or x2 >= 300:
                continue
            filtered_ids.append(int(cls.item()))

        # 6) Lógica de detección por “ventana”
        if filtered_ids:
            if not self.collecting:
                # Comienza nueva ventana
                self.collecting = True
                self.no_detection_count = 0
                self.box_detections = []
            # Acumula todas las detecciones (IDs) dentro de esta ventana
            self.box_detections.extend(filtered_ids)
            self.no_detection_count = 0

        else:
            # No hay detecciones en este frame
            if self.collecting:
                self.no_detection_count += 1
                if self.no_detection_count >= self.END_DELAY_FRAMES:
                    # Se supera el umbral: cerramos la ventana
                    if self.box_detections:
                        # 6.1) Calculamos el most_common en esta ventana
                        most_common = int(np.bincount(self.box_detections).argmax())

                        # 6.2) Publicamos en ROS la etiqueta most_common
                        print(f"CAJA_DETECTADA_CON_ETIQUETA_{most_common}  inventario actualizado")
                        self.msg2.data = most_common
                        self.publisher_detected.publish(self.msg2)

                        # 6.3) Incrementamos el contador de ventanas para esa etiqueta
                        self.window_counts[most_common] += 1

                        # 6.4) Reescribimos counts.csv con los totales actualizados
                        self.write_csv()

                    # 6.5) Reiniciamos variables para la siguiente ventana
                    self.collecting = False
                    self.no_detection_count = 0
                    self.box_detections.clear()

        # 7) Construir imagen anotada para visualización
        annotated = results.plot()
        annotated_bgr = cv2.cvtColor(annotated, cv2.COLOR_RGB2BGR)

        # 8) Mostrar ventana (si se presiona 'q' aquí, también apagamos)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Se recibió 'q' en ventana OpenCV: apagando nodo.")
            rclpy.shutdown()

        # 9) Publicar imagen anotada en el tópico
        self.msg_img = self.bridge.cv2_to_imgmsg(annotated_bgr, 'rgb8')
        self.publisher_img.publish(self.msg_img)

    def destroy(self):
        # Cuando el nodo se destruye, detenemos el hilo y cerramos recursos
        self._stop_thread = True
        if self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

        # Por si acaso quedó algo pendiente, reescribimos el CSV final
        self.write_csv()

    def __del__(self):
        # Asegurar cierre de CSV si no se llamó a destroy()
        try:
            self.write_csv()
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = My_Subscriber()
    rclpy.spin(node)
    node.destroy()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
