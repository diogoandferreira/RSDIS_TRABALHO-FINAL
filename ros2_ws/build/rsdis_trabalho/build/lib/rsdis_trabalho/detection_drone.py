import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

# Estados da máquina de estados do drone
SUBIR = 'SUBIR'
PROCURAR = 'PROCURAR'
CENTRAR = 'CENTRAR'
PASSAR = 'PASSAR'
CONCLUIDO = 'CONCLUIDO'


class detecao_cilindros(Node):

    def __init__(self):
        super().__init__('detecao_cilindros_node')

        self.bridge = CvBridge()

        # Subscrições e Publicadores
        self.subscription = self.create_subscription(Image, '/uav/camera/image_raw', self.convert_cv, 10)  # camara do UAV
        self.odom_sub = self.create_subscription(Odometry, '/X3/gazebo/odometry', self.odom_cb, 10)         # posicao UAV

        self.status_pub = self.create_publisher(String, '/ugv_status', 10)           # estados do drone para o UGV (SAFE, STOP, DONE)
        self.uav_pub = self.create_publisher(Twist, '/X3/gazebo/command/twist', 10)  # twist de comando para o UAV
        self.enable_pub = self.create_publisher(Bool, '/X3/enable', 10)              # enable do UAV (para manter motores ligados)

        self.arvore_det_pub = self.create_publisher(Point, '/arvores_detetadas', 10)  # posição das árvores detetadas (X, Y, Z) para o UGV

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        # Variáveis de Estado
        self.estado = SUBIR
        self.altura_alvo = 4.0   # altura de voo (m)

        self.candidatos_atuais = []     # blobs verdes da imagem atual
        self.alvo_time = self.get_clock().now()
        self.t_sem_alvo = self.get_clock().now()
        self.t_passar = self.get_clock().now()

        self.contador_centrado = 0
        self.cilindros_visitados = 0
        self.max_cilindros = 14         # número total de cilindros a visitar

        # --- Geometria da câmara (virada para baixo) ---
        self.fov_h = 1.047              # horizontal_fov do model.sdf (rad)
        self.altura_topo_arvore = 1.5   # topo dos cilindros (z = 0.5 + 2.0/2)

        # --- Memória de árvores já registadas ---
        self.arvores_registadas = []
        self.dist_nova_arvore = 1.0     # m: uma árvore a menos disto de uma registada é a mesma
        self.tempo_passar = 1.0         # s parado depois de registar

        # Temporizador a 10 Hz
        self.create_timer(0.1, self.uav_timer)
        self.get_logger().info("Nó de centramento multialvo iniciado!")

    def odom_cb(self, msg):  # atualiza a posição atual do UAV a partir da odometria
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z

    def convert_cv(self, msg):
        imagem_cv = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(imagem_cv, cv2.COLOR_BGR2HSV)

        # Intervalo de cor verde para os cilindros
        lower_verde = np.array([45, 80, 40])
        upper_verde = np.array([75, 255, 255])
        mascara = cv2.inRange(hsv, lower_verde, upper_verde)
        mascara = cv2.morphologyEx(mascara, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        h, w = mascara.shape

        # Recolher todos os blobs verdes válidos (ignorar ruído pequeno)
        candidatos = []
        for c in contornos:
            area = cv2.contourArea(c)
            if area > 80.0:
                m = cv2.moments(c)
                if m['m00'] > 0:
                    cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
                    ex = (cx - w / 2) / w  # Erro normalizado X [-0.5, 0.5]
                    ey = (cy - h / 2) / h  # Erro normalizado Y [-0.5, 0.5]
                    candidatos.append({'ex': ex, 'ey': ey, 'area': area, 'w': w, 'h': h})

        self.candidatos_atuais = candidatos
        if candidatos:
            self.alvo_time = self.get_clock().now()

    def estimar_posicao_arvore(self, ex, ey, w, h):
        """Estima a posição (x, y) no mundo de um blob, a partir do erro na imagem.
        Assume câmara a apontar para baixo e drone sem rodar (yaw ~ 0):
        topo da imagem = frente do drone (+x), direita da imagem = -y."""
        dist = max(self.current_z - self.altura_topo_arvore, 0.5)    # câmara -> topo do cilindro
        largura_m = 2.0 * dist * math.tan(self.fov_h / 2.0)          # largura vista no plano do topo
        arvore_x = self.current_x - ey * (h / w) * largura_m
        arvore_y = self.current_y - ex * largura_m
        return arvore_x, arvore_y

    def selecionar_melhor_alvo(self):
        """Escolhe, entre os blobs visíveis, o mais próximo do centro que ainda NÃO foi registado."""
        idade = (self.get_clock().now() - self.alvo_time).nanoseconds / 1e9
        if not self.candidatos_atuais or idade > 0.4:
            return None

        novos = []
        for c in self.candidatos_atuais:
            # posição estimada DESTE blob (e não a do drone)
            ax, ay = self.estimar_posicao_arvore(c['ex'], c['ey'], c['w'], c['h'])
            ja_registada = any(math.hypot(ax - px, ay - py) < self.dist_nova_arvore
                               for px, py in self.arvores_registadas)
            if not ja_registada:
                novos.append((c['ex'] ** 2 + c['ey'] ** 2, c, ax, ay))

        if not novos:
            return None

        _, melhor, ax, ay = min(novos, key=lambda n: n[0])
        return (melhor['ex'], melhor['ey'], ax, ay)

    def uav_timer(self):
        # Habilitar motores do X3
        enable_msg = Bool()
        enable_msg.data = True
        self.enable_pub.publish(enable_msg)

        agora = self.get_clock().now()
        cmd = Twist()
        status = "SAFE"
        resultado_alvo = self.selecionar_melhor_alvo()

        # Manutenção constante da altura (Z)
        if self.estado != SUBIR:
            erro_z = self.altura_alvo - self.current_z
            cmd.linear.z = float(np.clip(1.0 * erro_z, -0.4, 0.4))

        if self.estado == SUBIR:
            if self.current_z < self.altura_alvo - 0.2:
                cmd.linear.z = 0.8
                self.get_logger().info(f"A subir para {self.altura_alvo}m... Altura atual: {self.current_z:.2f}m",
                                       throttle_duration_sec=1.0)
            else:
                self.get_logger().info(f"Altura de {self.altura_alvo}m atingida. A iniciar prospeção do pomar!")
                self.estado = PROCURAR

        elif self.estado == PROCURAR:
            status = "SAFE"
            if resultado_alvo is not None:
                # há uma árvore nova à vista (à frente, ao lado ou atrás): vai centrar nela
                self.get_logger().info("Nova árvore detetada! A iniciar centramento...")
                self.contador_centrado = 0
                self.t_sem_alvo = agora
                self.estado = CENTRAR
            else:
                cmd.linear.x = 0.35  # sem árvores novas à vista: avança pelo corredor
                cmd.linear.y = 0.0

        elif self.estado == CENTRAR:
            status = "STOP"
            if resultado_alvo is not None:
                self.t_sem_alvo = agora
                ex, ey = resultado_alvo[0], resultado_alvo[1]

                k = 1.2
                vmax = 0.35
                cmd.linear.x = float(np.clip(-k * ey, -vmax, vmax))
                cmd.linear.y = float(np.clip(-k * ex, -vmax, vmax))

                if abs(ex) < 0.05 and abs(ey) < 0.05:
                    self.contador_centrado += 1
                else:
                    self.contador_centrado = 0

                # Quando está centrado durante 1.5 segundos
                if self.contador_centrado >= 15:
                    self.cilindros_visitados += 1

                    # 1. Registar a posição atual do drone como sendo a árvore
                    pos_arvore_x = self.current_x
                    pos_arvore_y = self.current_y
                    self.arvores_registadas.append((pos_arvore_x, pos_arvore_y))

                    # 2. Publicar a posição no tópico ROS 2
                    p_msg = Point()
                    p_msg.x = float(pos_arvore_x)
                    p_msg.y = float(pos_arvore_y)
                    p_msg.z = float(self.current_z)
                    self.arvore_det_pub.publish(p_msg)
                    self.get_logger().info(
                        f"Árvore {self.cilindros_visitados}/{self.max_cilindros} registada em "
                        f"X={pos_arvore_x:.2f}, Y={pos_arvore_y:.2f}")

                    self.t_passar = agora
                    self.estado = PASSAR
            else:
                # perdeu o alvo: se passar 1 s sem ver nada novo, volta a procurar
                if (agora - self.t_sem_alvo).nanoseconds / 1e9 > 1.0:
                    self.estado = PROCURAR

        elif self.estado == PASSAR:
            status = "STOP"   # fica parado um instante e volta a procurar (a árvore pode estar ao lado)
            if (agora - self.t_passar).nanoseconds / 1e9 >= self.tempo_passar:
                if self.cilindros_visitados >= self.max_cilindros:
                    self.get_logger().info("Missão cumprida! Todas as árvores visitadas.")
                    self.estado = CONCLUIDO
                else:
                    self.get_logger().info("A procurar a próxima árvore nova...")
                    self.estado = PROCURAR

        elif self.estado == CONCLUIDO:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            status = "DONE"

        self.uav_pub.publish(cmd)

        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = detecao_cilindros()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()