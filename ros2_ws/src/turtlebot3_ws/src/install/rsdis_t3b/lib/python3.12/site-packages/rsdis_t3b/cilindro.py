import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan


class TurtleBot3detetarcilindro(Node):

    def __init__(self):
        super().__init__('turtlebot3_cylinder_detection')
        print('TurtleBot3 Detetar Cilindro')
        print('----------------------------------------------')

        self.scan_ranges = []
        self.has_scan_received = False

        self. scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        self.timer = self.create_timer(0.5, self.timer_callback) # Corre a cada 0.5s para conseguires ler no terminal

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.has_scan_received = True

    def timer_callback(self):
        if self.has_scan_received:
            self.detetar_cilindro()

    def detetar_cilindro(self):
        threshold = 0.30
        flag_cilindro = 0
        Pi = None
        Pf = None
            
        for i in range(1, len(self.scan_ranges)):
            cur = i
            prev = i - 1

            if math.isinf(self.scan_ranges[cur]) or math.isinf(self.scan_ranges[prev]):
                continue
            if self.scan_ranges[cur] <= 0.0 or self.scan_ranges[prev] <= 0.0:
                continue

            diff = self.scan_ranges[cur] - self.scan_ranges[prev]

            if abs(diff) > threshold and diff < 0:
                flag_cilindro = 1
                Pi = [cur, round(self.scan_ranges[cur], 2)]

            elif abs(diff) > threshold and flag_cilindro == 1 and diff > 0:

                Pf = [prev, round(self.scan_ranges[prev], 2)]
                
                print(f"Cilindro Detetado! -> Pi (Início): {Pi} | Pf (Fim): {Pf}")
                
                flag_cilindro = 0
                break


def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3detetarcilindro()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()