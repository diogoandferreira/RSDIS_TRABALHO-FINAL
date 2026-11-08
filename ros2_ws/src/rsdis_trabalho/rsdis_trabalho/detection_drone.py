import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
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
VOLTAR_CASA = 'VOLTAR_CASA'
CONCLUIDO = 'CONCLUIDO'


class detecao_cilindros(Node):

    def __init__(self):
        super().__init__('detecao_cilindros_node')

        self.bridge = CvBridge()

        # Subscrições e Publicadores
        self.subscription = self.create_subscription(Image, '/uav/camera/image_raw', self.convert_cv, 10)  # camara do UAV
        self.odom_sub = self.create_subscription(Odometry, '/X3/gazebo/odometry', self.odom_cb, 10)         # posicao UAV

        self.status_pub = self.create_publisher(String, '/ugv_status', 10)           # estados do drone para o UGV (SAFE, STOP, BACKHOME, DONE)
        self.uav_pub = self.create_publisher(Twist, '/X3/gazebo/command/twist', 10)  # twist de comando para o UAV
        self.enable_pub = self.create_publisher(Bool, '/X3/enable', 10)              # enable do UAV (para manter motores ligados)

        # transient_local: quem se ligar depois (o UGV) recebe também as árvores já registadas
        qos_arvores = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.arvore_det_pub = self.create_publisher(Point, '/arvores_detetadas', qos_arvores)  # posição das árvores detetadas (X, Y, Z) para o UGV

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        self.spawn_x = 0.0
        self.spawn_y = 0.0
        self.spawn_z = 0.0
        self.spawn_registado = False

        # Variáveis de Estado
        self.estado = SUBIR
        self.altura_alvo = 4.0   # altura de voo (m)
        self.motores_ligados = True     # passa a False quando aterra

        self.candidatos_atuais = []     # blobs verdes da imagem atual
        self.alvo_time = self.get_clock().now()
        self.t_sem_alvo = self.get_clock().now()
        self.t_passar = self.get_clock().now()

        self.contador_centrado = 0
        self.cilindros_visitados = 0
        self.max_cilindros = 4        # número total de cilindros a visitar

        # --- Geometria da câmara (virada para baixo) ---
        self.fov_h = 1.047              # horizontal_fov do model.sdf (rad)
        self.altura_topo_arvore = 1.5   # topo dos cilindros (z = 0.5 + 2.0/2)

        # --- Memória de árvores já registadas ---
        self.arvores_registadas = []
        self.dist_nova_arvore = 1.0     # m: uma árvore a menos disto de uma registada é a mesma
        self.tempo_passar = 1.0         # s parado depois de registar

        # --- Centramento: o drone VIRA para a árvore e avança em frente (sem andar de lado) ---
        self.k_yaw = 1.2                # ganho de rotação (rad/s por rad de erro)
        self.w_max = 0.8                # rotação máxima (rad/s)
        self.k_v = 1.0                  # ganho de avanço (m/s por m de distância)
        self.v_max_centrar = 0.35       # avanço máximo ao centrar (m/s)
        self.d_fino = 0.12              # m: abaixo disto deixa de rodar
        self.d_centrado = 0.15          # m: considera-se centrado abaixo disto
        self.cmd_em_corpo = True        # False se o X3 interpretar o Twist no referencial do mundo

        # --- Voltar a casa e aterrar ---
        self.v_max_casa = 0.5           # velocidade maxima a voltar (m/s)
        self.dist_casa = 0.5            # m: abaixo disto comeca a descer
        self.v_descida = 0.5            # m/s
        self.a_aterrar = False          # fica True quando chega por cima de casa

        # Temporizador a 10 Hz
        self.create_timer(0.1, self.uav_timer)
        self.get_logger().info("Nó de centramento multialvo iniciado!")

    def odom_cb(self, msg):  # atualiza a posição atual do UAV a partir da odometria
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

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

        dist = max(self.current_z - self.altura_topo_arvore, 0.5)    # câmara -> topo do cilindro
        largura_m = 2.0 * dist * math.tan(self.fov_h / 2.0)
        frente = -ey * (h / w) * largura_m
        esquerda = -ex * largura_m
        c, sn = math.cos(self.current_yaw), math.sin(self.current_yaw)
        arvore_x = self.current_x + c * frente - sn * esquerda
        arvore_y = self.current_y + sn * frente + c * esquerda
        return arvore_x, arvore_y

    def comando_centrar(self, ex, ey, w, h):

        dist_cam = max(self.current_z - self.altura_topo_arvore, 0.5)
        largura_m = 2.0 * dist_cam * math.tan(self.fov_h / 2.0)
        frente = -ey * (h / w) * largura_m        # m à frente do drone (negativo = atrás)
        direita = ex * largura_m                  # m à direita do drone
        d = math.hypot(frente, direita)
        if d > self.d_fino:
            erro_ang = math.atan2(direita, frente)    # >0: árvore à direita
            wz = float(np.clip(-self.k_yaw * erro_ang, -self.w_max, self.w_max))
            wz *= min(1.0, d / 0.6)          # perto da árvore roda mais devagar (evita exagerar a viragem)
            # só avança quando está virado para a árvore (cos^3 quase anula o avanço com erro grande)
            vx = float(np.clip(self.k_v * d * max(0.0, math.cos(erro_ang)) ** 3, 0.0, self.v_max_centrar))
        else:
            wz = 0.0
            vx = float(np.clip(self.k_v * frente, -0.1, 0.1))   # acerto fino à frente/atrás
        return vx, wz, d

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
        return (melhor['ex'], melhor['ey'], ax, ay, melhor['w'], melhor['h'])

    def uav_timer(self):
        # Habilitar motores do X3 (passa a False quando aterra)
        enable_msg = Bool()
        enable_msg.data = self.motores_ligados
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

            if not self.spawn_registado and self.current_z > 0.01:
                self.spawn_x = self.current_x
                self.spawn_y = self.current_y
                self.spawn_z = self.current_z
                self.spawn_registado = True
                self.get_logger().info(f"Posição inicial guardada: X={self.spawn_x:.2f}, Y={self.spawn_y:.2f}, Z={self.spawn_z:.2f}")

        elif self.estado == PROCURAR:
            status = "SAFE"
            if resultado_alvo is not None:
                # há uma árvore nova à vista (à frente, ao lado ou atrás): vai centrar nela
                self.get_logger().info("Nova árvore detetada!")
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
                ex, ey, _, _, w_img, h_img = resultado_alvo

                # vira para a árvore e avança em frente (sem andar de lado)
                cmd.linear.x, cmd.angular.z, d = self.comando_centrar(ex, ey, w_img, h_img)

                if d < self.d_centrado:
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
            status = "STOP"   # fica parado, alinha o rumo com o pomar e volta a procurar
            # as filas de árvores seguem os eixos: roda para o múltiplo de 90° mais próximo
            yaw_alvo = round(self.current_yaw / (math.pi / 2)) * (math.pi / 2)
            erro_yaw = math.atan2(math.sin(yaw_alvo - self.current_yaw),
                                  math.cos(yaw_alvo - self.current_yaw))
            cmd.angular.z = float(np.clip(1.5 * erro_yaw, -self.w_max, self.w_max))
            t_passar = (agora - self.t_passar).nanoseconds / 1e9
            if t_passar >= self.tempo_passar and (abs(erro_yaw) < 0.08 or t_passar > 4.0):
                if self.cilindros_visitados >= self.max_cilindros:
                    self.get_logger().info("Missão cumprida! Todas as árvores visitadas. A voltar para casa...")
                    self.estado = VOLTAR_CASA
                else:
                    self.get_logger().info("A procurar a próxima árvore nova...")
                    self.estado = PROCURAR

        elif self.estado == VOLTAR_CASA:
            status = "BACKHOME"
            dist_origem = math.hypot(self.current_x - self.spawn_x, self.current_y - self.spawn_y)

            # 1. Voltar à origem: vira para casa e avança em frente (sem andar de lado)
            if not self.a_aterrar and dist_origem > self.dist_casa:
                angulo_origem = math.atan2(self.spawn_y - self.current_y, self.spawn_x - self.current_x)
                erro_yaw = math.atan2(math.sin(angulo_origem - self.current_yaw),
                                      math.cos(angulo_origem - self.current_yaw))
                cmd.angular.z = float(np.clip(self.k_yaw * erro_yaw, -self.w_max, self.w_max))
                cmd.linear.x = float(np.clip(self.k_v * dist_origem * max(0.0, math.cos(erro_yaw)) ** 3,
                                             0.0, self.v_max_casa))
                # (cmd.linear.z fica com a manutenção de altura de cima)
                self.get_logger().info(f"BACK HOME: a regressar à origem... Distância: {dist_origem:.2f}m",
                                       throttle_duration_sec=2.0)

            # 2. Por cima de casa: desce de forma controlada
            elif self.current_z > self.spawn_z + 0.3:
                self.a_aterrar = True        # a partir daqui já não volta a andar na horizontal
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                cmd.linear.z = -self.v_descida
                self.get_logger().info(f"A descer para a aterragem... Altura atual: {self.current_z:.2f}m",
                                       throttle_duration_sec=1.0)

            # 3. Já está em baixo
            else:
                self.get_logger().info("UAV aterrou na posição inicial.")
                self.estado = CONCLUIDO

        elif self.estado == CONCLUIDO:
            status = "DONE"
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0
            cmd.linear.z = 0.0
            if self.motores_ligados:
                self.motores_ligados = False   # desliga os motores
                self.get_logger().info("Motores desligados.")

        if not self.cmd_em_corpo:   # converter velocidade do corpo para o mundo
            c, sn = math.cos(self.current_yaw), math.sin(self.current_yaw)
            vx_b, vy_b = cmd.linear.x, cmd.linear.y
            cmd.linear.x = c * vx_b - sn * vy_b
            cmd.linear.y = sn * vx_b + c * vy_b

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