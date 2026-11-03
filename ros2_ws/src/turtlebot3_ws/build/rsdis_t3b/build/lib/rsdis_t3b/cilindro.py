import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class TurtleBot3DetetarCilindro(Node):

    def __init__(self):
        super().__init__('turtlebot3_cylinder_detection')
        self.get_logger().info('TurtleBot3 Detetar Cilindro Iniciado')
        self.get_logger().info('----------------------------------------------')

        self.latest_scan = None

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        self.timer = self.create_timer(0.5, self.timer_callback)

    def scan_callback(self, msg):
        self.latest_scan = msg

    def timer_callback(self):
        if self.latest_scan is not None:
            self.detetar_cilindro(self.latest_scan)

    def detetar_cilindro(self, scan_msg):
        threshold = 0.35  
        flag_cilindro = 0
        i_inicio = None
        
        ranges = scan_msg.ranges
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment

        for i in range(1, len(ranges)):
            cur = i
            prev = i - 1

            if math.isinf(ranges[cur]) or math.isinf(ranges[prev]) or math.isnan(ranges[cur]) or math.isnan(ranges[prev]):
                continue
            if ranges[cur] <= 0.0 or ranges[prev] <= 0.0:
                continue

            diff = ranges[cur] - ranges[prev]

            if diff < -threshold and flag_cilindro == 0:
                flag_cilindro = 1
                i_inicio = prev


            elif diff > threshold and flag_cilindro == 1:
                i_fim = cur
                flag_cilindro = 0

                i_centro = (i_inicio + i_fim) // 2


                angulo_rad = angle_min + (i_centro * angle_increment)
                angulo_deg = math.degrees(angulo_rad)


                while angulo_deg > 180.0:
                    angulo_deg -= 360.0
                while angulo_deg < -180.0:
                    angulo_deg += 360.0

                distancia = ranges[i_centro]

                self.get_logger().info(
                    f"Cilindro Detetado! | Distância: {distancia:.2f}m | Ângulo: {angulo_deg:.1f}°"
                )
                break  

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot3DetetarCilindro()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()