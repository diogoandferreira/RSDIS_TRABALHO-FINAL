import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class desvio_cilindros(Node):

    def __init__(self):
        super().__init__('desvio_cilindros_node')

        self.tele_twist = Twist()
        self.has_scan_received = False
        self.tele_twist.linear.x = 0.5
        self.tele_twist.angular.z = 0.0
        self.estado_atual = "GO"
        self.scan_ranges = []
        self.stop_distance = 0.5

        self.subscription = self.create_subscription(String, 'ugv_status', self.status_callback, 10)
        self.subscription = self.create_subscription(LaserScan, '/scan',self.scan_callback, 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel_raw',self.cmd_vel_raw_callback, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel',10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Nó de desvio LiDAR iniciado com sucesso!")

    def timer_callback(self):

        if self.estado_atual == "STOP":
            self.desvio_obstaculo()
        else: 
 
            self.publisher.publish(self.tele_twist)
        
    def scan_callback(self, msg): # Guarda continuamente as leituras do LiDAR
        
        self.scan_ranges = msg.ranges
        self.has_scan_received = True

    def cmd_vel_raw_callback(self, msg):
        self.tele_twist = msg

    def status_callback(self, msg):  # Se recebemos STOP e temos leituras do LiDAR, executamos a lógica de desvio

        self.get_logger().info(f"Recebi o estado: '{msg.data}'")
        self.estado_atual = msg.data

    def sector_min(self, vals):  
            
        v = [x for x in vals if math.isfinite(x) and x > 0.05]
        return min(v) if v else 3.5

    def desvio_obstaculo(self):

        self.get_logger().info(f'entrei na funcao desviar', throttle_duration_sec=2)

        r = list(self.scan_ranges) #pontos do lidar
        n = len(r)                 #distancia dos pontos
        a30 = int(n / 12)         # 30
        a90 = int(n / 4)          # 90
        a270 = int(n * 3 / 4)     # 270
        a330 = int(n * 11 / 12)   # 330

        obstacle_distance = self.sector_min(r[0:a30] + r[a330:n])
        left_side = self.sector_min(r[a30:a90])
        right_side = self.sector_min(r[a270:a330])

        twist = Twist()

        if obstacle_distance < self.stop_distance:
            twist.linear.x = 0.0

            if left_side > right_side:
                twist.angular.z = 1.0
                self.get_logger().info('Virar à esquerda!', throttle_duration_sec=2)
                self.estado_atual = "GO"
            else:
                twist.angular.z = -1.0
                self.get_logger().info('Virar à direita!', throttle_duration_sec=2)
                self.estado_atual = "GO"
        else:
             twist = self.tele_twist

        self.publisher.publish(twist)
        
def main(args=None):

    rclpy.init(args=args)
    node = desvio_cilindros()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()



