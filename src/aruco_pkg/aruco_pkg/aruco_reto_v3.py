#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
from cv2 import aruco
import numpy as np
import os
import json
from ament_index_python.packages import get_package_share_directory

DICT_TYPE     = aruco.DICT_4X4_50
MARKER_SIZE_M = 0.05
PKG_NAME      = 'aruco_pkg'

class ArucoDetectorNode(Node):
    # Inicializa el nodo y configura publishers, suscriptores, calibración, cámara y timer
    def __init__(self):
        super().__init__('aruco_detector_node')
        self.state_pub = self.create_publisher(Int32, 'box_state', 10)
        self.img_pub   = self.create_publisher(Image, 'imagen_aruco', 10)
        self.bridge    = CvBridge()
        self.current_yolo = None
        self.create_subscription(
            Int32,
            'yolo_cajas',
            self.yolo_callback,
            10
        )
        self.adequate_count = 0
        self.damaged_count  = 0
        try:
            share_dir = get_package_share_directory(PKG_NAME)
            file_path = os.path.join(share_dir, 'config.json')
            self.camera_mtx, self.dist_coeff = self.load_calibration(file_path)
            self.get_logger().info(f"Calibración cargada desde: {file_path}")
        except Exception as e:
            self.get_logger().error(f"No se pudo cargar calibración: {e}")
            rclpy.shutdown()
            return
        self.detector = self.create_aruco_detector(DICT_TYPE)
        cam_index = 0
        self.cap = cv2.VideoCapture(cam_index)
        if not self.cap.isOpened():
            self.get_logger().error(f"No se pudo abrir la cámara en índice {cam_index}.")
            rclpy.shutdown()
            return
        self.get_logger().info("Cámara inicializada con VideoCapture.")
        timer_period = 0.05
        self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Nodo listo: detectando ArUco a ~20 Hz y escuchando 'yolo_cajas'.")

    # Carga los parámetros de calibración de la cámara desde un archivo JSON
    def load_calibration(self, path: str):
        with open(path, 'r') as f:
            data = json.load(f)
        cam_mtx = np.asarray(data["camera_matrix"], dtype=np.float32)
        dist = np.asarray(data["dist_coeff"][0], dtype=np.float32)
        return cam_mtx, dist

    # Crea y retorna un detector de marcas ArUco basado en el diccionario especificado
    def create_aruco_detector(self, dict_type: int):
        aruco_dict = aruco.getPredefinedDictionary(dict_type)
        params     = aruco.DetectorParameters()
        detector   = aruco.ArucoDetector(aruco_dict, params)
        return detector

    # Callback que actualiza el valor recibido del tópico 'yolo_cajas'
    def yolo_callback(self, msg: Int32):
        self.current_yolo = msg.data
        self.get_logger().debug(f"yolo_cajas recibió: {self.current_yolo}")

    # Procesa frames periódicamente: detecta marcadores, calcula estado y publica resultados
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            estado = Int32(data=0)
            self.state_pub.publish(estado)
            self.get_logger().warn("No se pudo leer frame de cámara. Publicando estado=0.")
            return
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        estado_valor = 0
        if ids is not None and len(ids) > 0:
            m_id = int(ids.flatten()[0])
            c   = corners[0].reshape(4, 2).astype(np.int32)
            cv2.polylines(frame, [c], True, (0, 255, 0), 2)
            center = tuple(np.mean(c, axis=0).astype(int))
            cv2.circle(frame, center, 4, (0, 0, 255), -1)
            cv2.putText(frame, f"ID: {m_id}",
                        (center[0] - 20, center[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            half_s = MARKER_SIZE_M / 2.0
            obj_pts = np.array([
                [-half_s,  half_s, 0],
                [ half_s,  half_s, 0],
                [ half_s, -half_s, 0],
                [-half_s, -half_s, 0],
            ], dtype=np.float32)
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts,
                c.astype(np.float32),
                self.camera_mtx,
                self.dist_coeff,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )
            if ok:
                dist_m   = np.linalg.norm(tvec)
                dist_cm  = dist_m * 100.0
                cv2.putText(frame, f"{dist_cm:.1f} cm",
                            (center[0] - 40, center[1] + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
                if (self.current_yolo is not None) and (dist_cm < 120.0):
                    y = self.current_yolo
                    if y in (0, 1, 3, 4, 6, 7):
                        estado_valor = 1
                        self.damaged_count += 1
                        descripcion = "caja dañada"
                    elif (y == 2 and m_id in (0, 1, 2)) or \
                         (y == 5 and m_id in (3, 4, 5)) or \
                         (y == 8 and m_id in (6, 7, 8)):
                        estado_valor = 0
                        self.adequate_count += 1
                        descripcion = "caja adecuada"
                    else:
                        estado_valor = 1
                        descripcion = "caja incorrecta"
                    cv2.putText(frame, f"Estado: {descripcion}",
                                (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    self.get_logger().info(
                        f"[{descripcion.upper()}] YOLO={y}, ArUco={m_id}  "
                        f"Adecuadas={self.adequate_count}, Dañadas={self.damaged_count}"
                    )
                else:
                    estado_valor = 0
            else:
                estado_valor = 0
        else:
            estado_valor = 0
        msg_estado = Int32(data=int(estado_valor))
        self.state_pub.publish(msg_estado)
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.img_pub.publish(img_msg)
        except CvBridgeError as e:
            self.get_logger().error(f"Error en CvBridge: {e}")
        cv2.imshow("ArUco Multi-detector Orin", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rclpy.shutdown()

    # Libera recursos de OpenCV y destruye el nodo ROS
    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

# Función principal que inicializa ROS, crea el nodo y gestiona su spin y cierre
def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
