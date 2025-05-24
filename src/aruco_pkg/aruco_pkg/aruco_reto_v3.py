#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String

import cv2
from cv2 import aruco
import numpy as np
import json

# Parámetros de tu calibración y ArUco
DICT_TYPE          = aruco.DICT_4X4_50
MARKER_SIZE_M      = 0.05
CALIB_FILE         = "aruco_detector/aruco_detector/config.json"

def load_calibration(path: str):
    with open(path, 'r') as f:
        d = json.load(f)
    return np.asarray(d["camera_matrix"]), np.asarray(d["dist_coeff"])

def create_detector(dict_type=DICT_TYPE) -> aruco.ArucoDetector:
    aruco_dict = aruco.getPredefinedDictionary(dict_type)
    params     = aruco.DetectorParameters()
    return aruco.ArucoDetector(aruco_dict, params)

def detect_markers(gray_img: np.ndarray, detector: aruco.ArucoDetector):
    return detector.detectMarkers(gray_img)


class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # Publisher único para el estado de la caja
        self.state_pub = self.create_publisher(String, 'box_state', 10)

        # Subscriber al tópico yolo_cajas
        self.current_yolo = None
        self.create_subscription(Int32, 'yolo_cajas', self.yolo_callback, 10)

        # Contadores
        self.adequate_count = 0
        self.damaged_count  = 0

        # Carga calibración y detector
        self.camera_mtx, self.dist_coeff = load_calibration(CALIB_FILE)
        self.detector = create_detector()

        # Inicializa cámara
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara.")
            rclpy.shutdown()
            return

        self.get_logger().info("Nodo listo: detectando ArUcos y escuchando yolo_cajas...")

    def yolo_callback(self, msg: Int32):
        self.current_yolo = msg.data

    def detect_and_publish(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detect_markers(gray, self.detector)

        estado = None

        if ids is not None:
            # Prepara puntos 3D del marcador
            half_s = MARKER_SIZE_M / 2
            obj_pts = np.array([
                [-half_s,  half_s, 0],
                [ half_s,  half_s, 0],
                [ half_s, -half_s, 0],
                [-half_s, -half_s, 0],
            ], dtype=np.float32)

            # Solo procesamos el primer marcador detectado
            m_id = int(ids.flatten()[0])
            c = corners[0].reshape(4, 2).astype(np.int32)

            # Dibujar contorno e ID
            cv2.polylines(frame, [c], True, (0, 255, 0), 2)
            center = tuple(np.mean(c, axis=0).astype(int))
            cv2.circle(frame, center, 4, (0, 0, 255), -1)
            cv2.putText(frame, f"ID: {m_id}",
                        (center[0] - 20, center[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

            # Estimar distancia
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, c.astype(np.float32),
                self.camera_mtx, self.dist_coeff,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if ok:
                dist_cm = float(np.linalg.norm(tvec) * 100.0)
                cv2.putText(frame, f"{dist_cm:.1f} cm",
                            (center[0] - 40, center[1] + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

                # Lógica de estado solo si tenemos un valor de YOLO y estamos cerca
                if self.current_yolo is not None and dist_cm < 120.0:
                    y = self.current_yolo
                    # Detectar daño
                    if y in (1,2,4,5,7,8):
                        estado = "caja dañada"
                        self.damaged_count += 1
                    else:
                        # 0-2 → tipo 0, 3-5 → tipo 1, 6-8 → tipo 2
                        tipo = y // 3
                        if tipo == m_id:
                            estado = "caja adecuada"
                            self.adequate_count += 1
                        else:
                            estado = "caja incorrecta"

        # Publicar e imprimir un solo mensaje de estado
        if estado is not None:
            msg = String(data=estado)
            self.state_pub.publish(msg)
            self.get_logger().info(
                f"[Estado: {estado}] Adecuadas={self.adequate_count}, Dañadas={self.damaged_count}"
            )
            # Mostrar estado en la ventana
            cv2.putText(frame, f"Estado: {estado}",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Mostrar siempre la ventana con detección y distancia
        cv2.imshow("ArUco Multi-detector", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        while rclpy.ok():
            node.detect_and_publish()
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap and node.cap.isOpened():
            node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#Con el Aruco mas pequeño hay un error de aproximadamente un metro 
# y lo detecta correctamente a hasta 1.5m aprox