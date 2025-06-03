#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
from cv2 import aruco
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import json

DICT_TYPE     = aruco.DICT_4X4_50
MARKER_SIZE_M = 0.05

# Obtiene dinámicamente la ruta al recurso config.json
pkg_share = get_package_share_directory('aruco_detector')
CALIB_FILE = os.path.join(pkg_share, 'config.json')

def load_calibration(path: str):
    with open(path, 'r') as f:
        d = json.load(f)
    camera_mtx = np.asarray(d["camera_matrix"], dtype=np.float32)
    dist_coeff = np.asarray(d["dist_coeff"], dtype=np.float32)
    return camera_mtx, dist_coeff

def create_detector(dict_type=DICT_TYPE) -> aruco.ArucoDetector:
    aruco_dict = aruco.getPredefinedDictionary(dict_type)
    params     = aruco.DetectorParameters()
    return aruco.ArucoDetector(aruco_dict, params)

def detect_markers(gray_img: np.ndarray, detector: aruco.ArucoDetector):
    return detector.detectMarkers(gray_img)

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # Publisher para el estado de la caja
        self.state_pub = self.create_publisher(String, 'box_state', 10)
        self.publisher = self.create_publisher(Image, 'imagen_aruco', 10)
        self.bridge = CvBridge()

        # Subscriber al tópico de YOLO
        self.current_yolo = None
        self.create_subscription(Int32, 'yolo_cajas', self.yolo_callback, 10)

        # Contadores
        self.adequate_count = 0
        self.damaged_count  = 0

        # Intentar cargar calibración
        try:
            self.camera_mtx, self.dist_coeff = load_calibration(CALIB_FILE)
        except Exception as e:
            self.get_logger().error(f"No se pudo cargar la calibración: {e}")
            rclpy.shutdown()
            return
        
        self.cap = cv2.VideoCapture(1)
        if not self.cap.isOpened():
            self.get_logger().error("No se pudo abrir la cámara.")
            rclpy.shutdown()
            return

        self.detector = create_detector()
        self.get_logger().info("Nodo listo: escuchando /camera1/video_raw y /yolo_cajas")

    def yolo_callback(self, msg: Int32):
        self.current_yolo = msg.data
        self.get_logger().debug(f"yolo_cajas recibió: {self.current_yolo}")

    def image_callback(self, img_msg: Image):
        # Convertir ROS Image a OpenCV
        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detect_markers(gray, self.detector)

        estado = None

        if ids is not None:
            half_s = MARKER_SIZE_M / 2
            obj_pts = np.array([
                [-half_s,  half_s, 0],
                [ half_s,  half_s, 0],
                [ half_s, -half_s, 0],
                [-half_s, -half_s, 0],
            ], dtype=np.float32)

            m_id = int(ids.flatten()[0])
            c = corners[0].reshape(4, 2).astype(np.int32)

            cv2.polylines(frame, [c], True, (0, 255, 0), 2)
            center = tuple(np.mean(c, axis=0).astype(int))
            cv2.circle(frame, center, 4, (0, 0, 255), -1)
            cv2.putText(frame, f"ID: {m_id}",
                        (center[0] - 20, center[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

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

                if self.current_yolo is not None and dist_cm < 120.0:
                    y = self.current_yolo
                    if y in (0, 1, 3, 4, 6, 7):
                        estado = "caja dañada"
                        self.damaged_count += 1
                    elif (y == 2 and m_id in (0, 1, 2)) or \
                         (y == 5 and m_id in (3, 4, 5)) or \
                         (y == 8 and m_id in (6, 7, 8)):
                        estado = "caja adecuada"
                        self.adequate_count += 1
                    else:
                        estado = "caja incorrecta"

        if estado is not None:
            msg = String(data=estado)
            self.state_pub.publish(msg)
            self.get_logger().info(
                f"[Estado: {estado}] Adecuadas={self.adequate_count}, Dañadas={self.damaged_count}"
            )
            cv2.putText(frame, f"Estado: {estado}",
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow("ArUco Multi-detector Orin", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rclpy.shutdown()

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
