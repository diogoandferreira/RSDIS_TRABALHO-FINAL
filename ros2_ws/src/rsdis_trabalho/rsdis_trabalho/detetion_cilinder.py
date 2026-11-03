import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np

class detecao_cilindros(Node):

    def __init__(self):
        super().__init__('detecao_cilindros_node')

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, 'camera/image_raw',self.convert_cv,10)
        self.publisher = self.create_publisher(String, 'ugv_status', 10)

    def convert_cv(self, msg):
        
        imagem_cv = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        imagem_hsv = cv2.cvtColor(imagem_cv, cv2.COLOR_BGR2HSV)

        lower_verde = np.array([45, 80, 40])
        upper_verde = np.array([75, 255, 255])

        mascara = cv2.inRange(imagem_hsv, lower_verde, upper_verde) 

        if(cv2.countNonZero(mascara) > 500):

            self.get_logger().info("Cilindro verde detetado!")

            status_msg = String()
            status_msg.data = "STOP"
            self.publisher.publish(status_msg)

            cv2.imshow("Visão camara do drone", mascara)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = detecao_cilindros()

    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()



