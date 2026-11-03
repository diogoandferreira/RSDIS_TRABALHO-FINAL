from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan


class Turtlebot3ObstacleDetection(Node):

    def __init__(self):
        super().__init__('turtlebot3_obstacle_detection')
        print('TurtleBot3 Obstacle Detection - Auto Move Enabled')
        print('----------------------------------------------')
        print('stop distance: 0.2 m')
        print('----------------------------------------------')

        self.scan_ranges = []
        self.has_scan_received = False

        self.stop_distance = 0.5    
        self.linear_speed = 0.15
        self.angular_z = 0.0
        
        qos = QoSProfile(depth=10)

        # MUDANÇA: O Gazebo DQN stage 4 exige TwistStamped para o cmd_vel
        self.cmd_vel_pub = self.create_publisher(TwistStamped, 'cmd_vel', qos)

        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def scan_callback(self, msg):
        self.scan_ranges = msg.ranges
        self.has_scan_received = True

    def timer_callback(self):
        if self.has_scan_received:
            self.detect_obstacle()

    def detect_distance_side(self):
        l_left_range = int(len(self.scan_ranges) / 4) #90

        obstacle_distance_left = min(self.scan_ranges[30:l_left_range])
        
        r_right_range = int(len(self.scan_ranges) * 11 / 12) #270
        
        obstacle_distance_right = min(self.scan_ranges[270:r_right_range])

        return obstacle_distance_left, obstacle_distance_right

    def detect_obstacle(self):
        left_range = int(len(self.scan_ranges)/ 12) #30
        right_range = int(len(self.scan_ranges) * 11 / 12) #330

        obstacle_distance = min(
            min(self.scan_ranges[0:left_range]),
            min(self.scan_ranges[right_range:360])
        )

        dist_left, dist_right = self.detect_distance_side()

        # Valores predefinidos: anda para a frente a 0.15 m/s
        linear_x = self.linear_speed
        angular_z = 0.0
        
        if obstacle_distance < self.stop_distance:
            linear_x = 0.0  
            self.get_logger().info('Obstacle close! Comparing sides...', throttle_duration_sec=2)

            # Compara qual o lado com maior espaço livre
            if dist_left > dist_right:
                angular_z = 0.25  # Vira para a esquerda
                self.get_logger().info(f'Left is safer ({dist_left:.2f}m > {dist_right:.2f}m). Turning Left.', throttle_duration_sec=2)
            else:
                angular_z = -0.25 # Vira para a direita
                self.get_logger().info(f'Right is safer ({dist_right:.2f}m > {dist_left:.2f}m). Turning Right.', throttle_duration_sec=2)
                
        # Construir a mensagem TwistStamped exigida pela simulação
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = 'base_link'
        twist_stamped.twist.linear.x = linear_x
        twist_stamped.twist.angular.z = angular_z

        self.cmd_vel_pub.publish(twist_stamped) 


def main(args=None):
    rclpy.init(args=args)
    turtlebot3_obstacle_detection = Turtlebot3ObstacleDetection()
    rclpy.spin(turtlebot3_obstacle_detection)

    turtlebot3_obstacle_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()