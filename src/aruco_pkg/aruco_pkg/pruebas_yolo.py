#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import random

class YoloCajasSimulator(Node):
    def __init__(self):
        super().__init__('yolo_cajas_simulator')
        self.pub = self.create_publisher(Int32, 'yolo_cajas', 10)
        # Timer cada 5 segundos
        self.create_timer(5.0, self.timer_callback)
        self.get_logger().info('Simulador yolo_cajas iniciado, publicando cada 5s...')

    def timer_callback(self):
        value = random.randint(1, 8)  # aleatorio entre 1 y 8
        msg = Int32(data=value)
        self.pub.publish(msg)
        self.get_logger().info(f'Publicado en yolo_cajas: {value}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloCajasSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
