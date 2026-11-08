import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math

# Estados do UGV
PARADO = 'PARADO'
SEGUIR = 'SEGUIR'
VOLTAR_CASA = 'VOLTAR_CASA'
CONCLUIDO = 'CONCLUIDO'


class seguir_drone(Node):

    def __init__(self):
        super().__init__('seguir_drone_node')

        # Subscrições e Publicadores
        self.odom_ugv_sub = self.create_subscription(Odometry, '/odom', self.odom_ugv_cb, 10)                # posicao UGV
        self.odom_uav_sub = self.create_subscription(Odometry, '/X3/gazebo/odometry', self.odom_uav_cb, 10)  # posicao UAV
        self.arvore_sub = self.create_subscription(Point, '/arvores_detetadas', self.arvore_cb, 10)          # arvores registadas pelo UAV
        self.status_sub = self.create_subscription(String, '/ugv_status', self.status_uav_cb, 10)            # estado do UAV (SAFE, STOP, BACKHOME, DONE)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)              # velocidade do UGV
        self.tether_pub = self.create_publisher(String, '/tether_status', 10)    # estado do cabo (OK, AVISO, LIMITE)

        # Posição do UGV e do UAV
        self.offset_x = 0.0             # onde o TurtleBot nasce no mundo (a odometria comeca em 0)
        self.offset_y = 0.0
        self.ugv_x = 0.0
        self.ugv_y = 0.0
        self.ugv_yaw = 0.0
        self.uav_x = 0.0
        self.uav_y = 0.0
        self.tem_ugv = False
        self.tem_uav = False

        # Posição de casa (onde o UGV arrancou)
        self.home_x = 0.0
        self.home_y = 0.0

        # Variáveis de Estado
        self.estado = PARADO
        self.estado_cabo = 'OK'
        self.arvores = []               # posicoes das arvores [(x, y), ...]

        # --- Cabo (tether) ---
        self.dist_max = 10.0            # comprimento do cabo: distancia maxima UGV-UAV (m)
        self.dist_seguir = 4.0          # comeca a seguir acima disto
        self.dist_parar = 2.0           # pára abaixo disto

        # --- Árvores ---
        self.raio_arvore = 0.9          # zona onde o UGV nao entra (m): arvore 0.6 + robo 0.25 + folga
        self.raio_desvio = 1.8          # a partir desta distancia comeca a desviar-se (m)

        # --- Movimento ---
        self.v_max = 0.5                # m/s (o waffle real faz 0.26)
        self.w_max = 1.0                # rad/s
        self.k_w = 1.5                  # ganho de rotacao

        # Temporizador a 10 Hz
        self.create_timer(0.1, self.ugv_timer)
        self.get_logger().info("Nó do UGV a seguir o drone iniciado!")

    def odom_ugv_cb(self, msg):  # posicao e orientacao do UGV
        self.ugv_x = msg.pose.pose.position.x + self.offset_x
        self.ugv_y = msg.pose.pose.position.y + self.offset_y
        q = msg.pose.pose.orientation
        self.ugv_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if not self.tem_ugv:  # a primeira posicao recebida e a casa
            self.home_x = self.ugv_x
            self.home_y = self.ugv_y
            self.tem_ugv = True
            self.get_logger().info(f"Casa do UGV guardada: X={self.home_x:.2f}, Y={self.home_y:.2f}")

    def odom_uav_cb(self, msg):  # posicao do UAV
        self.uav_x = msg.pose.pose.position.x
        self.uav_y = msg.pose.pose.position.y
        self.tem_uav = True

    def arvore_cb(self, msg):  # guarda a posicao de cada arvore registada pelo UAV
        for ax, ay in self.arvores:
            if math.hypot(msg.x - ax, msg.y - ay) < 0.3:
                return  # arvore repetida
        self.arvores.append((msg.x, msg.y))
        self.get_logger().info(f"Árvore {len(self.arvores)} registada no UGV: X={msg.x:.2f}, Y={msg.y:.2f}")

    def status_uav_cb(self, msg):  # quando o drone volta para casa, o UGV tambem volta
        if msg.data in ("BACKHOME", "DONE") and self.estado not in (VOLTAR_CASA, CONCLUIDO):
            self.estado = VOLTAR_CASA
            self.get_logger().info("O drone vai para casa! O UGV também vai regressar à posição inicial.")

    def ugv_timer(self):
        if not (self.tem_ugv and self.tem_uav):
            self.get_logger().warn("À espera das odometrias do UGV e do UAV...", throttle_duration_sec=3.0)
            return

        cmd = Twist()
        dist = math.hypot(self.uav_x - self.ugv_x, self.uav_y - self.ugv_y)

        # --- Estado do cabo ---
        if dist >= self.dist_max:
            cabo = 'LIMITE'
        elif dist > 0.8 * self.dist_max:
            cabo = 'AVISO'
        else:
            cabo = 'OK'
        if cabo != self.estado_cabo:
            self.get_logger().warn(f"Cabo: {cabo} (distância {dist:.1f} m, limite {self.dist_max} m)")
            self.estado_cabo = cabo
        cabo_msg = String()
        cabo_msg.data = cabo
        self.tether_pub.publish(cabo_msg)

        self.get_logger().info(f"Distância UGV-UAV: {dist:.1f} m | estado: {self.estado} | árvores: {len(self.arvores)}",
                               throttle_duration_sec=5.0)

        # --- Estados ---
        if self.estado == PARADO and dist > self.dist_seguir:
            self.estado = SEGUIR
            self.get_logger().info(f"A seguir o drone (distância {dist:.1f} m)")
        elif self.estado == SEGUIR and dist < self.dist_parar:
            self.estado = PARADO
            self.get_logger().info(f"Perto do drone ({dist:.1f} m): UGV parado")

        # --- Para onde ir ---
        if self.estado == SEGUIR:
            alvo_x, alvo_y = self.uav_x, self.uav_y
        elif self.estado == VOLTAR_CASA:
            alvo_x, alvo_y = self.home_x, self.home_y
            if math.hypot(alvo_x - self.ugv_x, alvo_y - self.ugv_y) < 0.3:
                self.get_logger().info("UGV chegou à posição inicial!")
                self.estado = CONCLUIDO
        else:
            alvo_x, alvo_y = None, None

        if alvo_x is not None:
            # direção para o alvo (vetor de tamanho 1)
            dx, dy = alvo_x - self.ugv_x, alvo_y - self.ugv_y
            d_alvo = max(math.hypot(dx, dy), 0.01)
            vx, vy = dx / d_alvo, dy / d_alvo

            # desvio das árvores: afasta-se das que estão perto
            dentro = False
            for ax, ay in self.arvores:
                rx, ry = self.ugv_x - ax, self.ugv_y - ay
                d = math.hypot(rx, ry)
                if d < self.raio_desvio and d > 0.01:
                    forca = 2.0 * (self.raio_desvio - d) / (self.raio_desvio - self.raio_arvore)
                    ux, uy = rx / d, ry / d              # sentido contrário à árvore
                    tx, ty = -uy, ux                     # sentido de contornar a árvore
                    if tx * dx + ty * dy < 0:            # contorna do lado do alvo
                        tx, ty = -tx, -ty
                    vx += forca * (ux + 0.5 * tx)
                    vy += forca * (uy + 0.5 * ty)
                    if d < self.raio_arvore:
                        dentro = True

            # velocidade: roda para o sentido pretendido e avança se estiver virado para lá
            erro_yaw = math.atan2(vy, vx) - self.ugv_yaw
            erro_yaw = math.atan2(math.sin(erro_yaw), math.cos(erro_yaw))   # entre -pi e pi
            cmd.angular.z = max(-self.w_max, min(self.w_max, self.k_w * erro_yaw))
            if abs(erro_yaw) < 1.0:
                cmd.linear.x = self.v_max * math.cos(erro_yaw)
                if dentro:
                    cmd.linear.x *= 0.3   # dentro da zona de uma árvore: sai devagar

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = seguir_drone()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()