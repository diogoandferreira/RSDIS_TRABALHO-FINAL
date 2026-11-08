import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import numpy as np
import heapq
import math

# Estados do UGV
PARADO = 'PARADO'
SEGUIR = 'SEGUIR'


class seguir_drone(Node):

    def __init__(self):
        super().__init__('seguir_drone_node')

        # Subscrições e Publicadores
        qos_arvores = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)  # igual ao publicador do drone

        self.odom_ugv_sub = self.create_subscription(Odometry, '/odom', self.odom_ugv_cb, 10)                 # posicao UGV
        self.odom_uav_sub = self.create_subscription(Odometry, '/X3/gazebo/odometry', self.odom_uav_cb, 10)   # posicao UAV
        self.arvore_sub = self.create_subscription(Point, '/arvores_detetadas', self.arvore_cb, qos_arvores)  # arvores registadas pelo UAV

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)              # velocidade do UGV
        self.tether_pub = self.create_publisher(String, '/tether_status', 10)    # estado do tether (OK, AVISO, LIMITE)

        # Posição do UGV e do UAV
        self.spawn_x = 0.0              
        self.spawn_y = 0.0
        self.ugv_x = 0.0
        self.ugv_y = 0.0
        self.ugv_yaw = 0.0
        self.uav_x = 0.0
        self.uav_y = 0.0
        self.tem_ugv = False
        self.tem_uav = False

        # Variáveis de Estado
        self.estado = PARADO
        self.estado_tether = 'OK'
        self.arvores = []               # posicoes das arvores [(x, y), ...] (pode-se pre-carregar aqui)
        self.caminho = []               # caminho planeado ate ao UAV
        self.t_plano = self.get_clock().now()

        # --- Tether ---
        self.dist_max = 20.0            # distancia maxima UGV-UAV (m)
        self.dist_seguir = 10.0         # comeca a seguir acima disto
        self.dist_parar = 5.0           # pára abaixo disto

        # --- Árvores: zona proibida à volta de cada uma ---
        self.raio_arvore = 1.0          # raio da zona proibida (m)
        self.margem_plano = 0.2         # o caminho passa a raio_arvore + margem_plano
        self.margem_parar = 0.3         # ao parar ao lado de uma arvore fica a raio_arvore + margem_parar
        self.res_grelha = 0.2           # resolucao da grelha do planeador (m)

        # --- Movimento ---
        self.v_max = 0.5                # m/s (o waffle real faz 0.26)
        self.w_max = 1.0                # rad/s
        self.k_w = 1.5                  # ganho de rotacao

        # Temporizador a 10 Hz
        self.create_timer(0.1, self.ugv_timer)
        self.get_logger().info("Nó do UGV a seguir o drone iniciado!")

    def odom_ugv_cb(self, msg):  # posicao e orientacao do UGV
        self.ugv_x = msg.pose.pose.position.x + self.spawn_x
        self.ugv_y = msg.pose.pose.position.y + self.spawn_y
        q = msg.pose.pose.orientation
        self.ugv_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.tem_ugv = True

    def odom_uav_cb(self, msg):  # posicao do UAV
        self.uav_x = msg.pose.pose.position.x
        self.uav_y = msg.pose.pose.position.y
        self.tem_uav = True

    def arvore_cb(self, msg):  # guarda a posicao de cada arvore registada pelo UAV
        for ax, ay in self.arvores:
            if math.hypot(msg.x - ax, msg.y - ay) < 0.3:
                return  # arvore repetida
        self.arvores.append((msg.x, msg.y))
        self.caminho = []
        self.get_logger().info(f"Árvore {len(self.arvores)} registada no UGV: X={msg.x:.2f}, Y={msg.y:.2f}")

    def objetivo_perto_uav(self):
        # objetivo = posicao do UAV, mas fora da zona das arvores (o UGV fica ao lado)
        gx, gy = self.uav_x, self.uav_y
        r_parar = self.raio_arvore + self.margem_parar
        for _ in range(2):
            for ax, ay in self.arvores:
                dx, dy = gx - ax, gy - ay
                d = math.hypot(dx, dy)
                if d < r_parar:
                    if d < 0.001:  # UAV mesmo em cima da arvore: usa o lado onde esta o UGV
                        dx, dy = self.ugv_x - ax, self.ugv_y - ay
                        d = math.hypot(dx, dy)
                        if d < 0.001:
                            dx, dy, d = 1.0, 0.0, 1.0
                    gx = ax + dx / d * r_parar
                    gy = ay + dy / d * r_parar
        return gx, gy

    def planear_caminho(self, objetivo):
        # A* numa grelha onde a zona de cada arvore fica bloqueada
        res = self.res_grelha
        r_plano = self.raio_arvore + self.margem_plano

        xs = [self.ugv_x, objetivo[0]] + [a[0] for a in self.arvores]
        ys = [self.ugv_y, objetivo[1]] + [a[1] for a in self.arvores]
        x0, y0 = min(xs) - 4.0, min(ys) - 4.0
        nx = int((max(xs) + 4.0 - x0) / res) + 1
        ny = int((max(ys) + 4.0 - y0) / res) + 1
        X, Y = np.meshgrid(x0 + res * np.arange(nx), y0 + res * np.arange(ny), indexing='ij')

        zona = np.zeros((nx, ny), dtype=bool)        # dentro do raio da arvore (proibido)
        zona_plano = np.zeros((nx, ny), dtype=bool)  # raio + margem (por onde o caminho nao passa)
        for ax, ay in self.arvores:
            d2 = (X - ax) ** 2 + (Y - ay) ** 2
            zona |= d2 < self.raio_arvore ** 2
            zona_plano |= d2 < r_plano ** 2

        # o UGV pode sair da margem se estiver perto de uma arvore (mas nunca entra na zona)
        dist_ugv = np.hypot(X - self.ugv_x, Y - self.ugv_y)
        livre = ~zona_plano | ((dist_ugv < 1.0) & ~zona)

        def celula(x, y):
            return (min(max(int(round((x - x0) / res)), 0), nx - 1),
                    min(max(int(round((y - y0) / res)), 0), ny - 1))

        s = celula(self.ugv_x, self.ugv_y)
        g = celula(objetivo[0], objetivo[1])
        if zona[s]:  # se estiver dentro de uma zona, deixa-o sair
            livre |= dist_ugv < 0.8
        if not livre[g]:  # objetivo bloqueado: usa a celula livre mais proxima
            livres = np.argwhere(livre)
            k = np.argmin(np.hypot(livres[:, 0] - g[0], livres[:, 1] - g[1]))
            g = (int(livres[k][0]), int(livres[k][1]))

        vizinhos = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        abertos = [(0.0, 0.0, s)]
        custo = {s: 0.0}
        veio = {}
        while abertos:
            _, c, atual = heapq.heappop(abertos)
            if atual == g:
                break
            if c > custo[atual]:
                continue
            for di, dj in vizinhos:
                n = (atual[0] + di, atual[1] + dj)
                if 0 <= n[0] < nx and 0 <= n[1] < ny and livre[n]:
                    novo = c + math.hypot(di, dj) * (1.0 + 3.0 * zona_plano[n])  # custa mais andar na margem
                    if novo < custo.get(n, 1e9):
                        custo[n] = novo
                        veio[n] = atual
                        heapq.heappush(abertos, (novo + math.hypot(g[0] - n[0], g[1] - n[1]), novo, n))

        if g != s and g not in veio:
            return []  # sem caminho

        caminho = [g]
        while caminho[-1] != s:
            caminho.append(veio[caminho[-1]])
        caminho.reverse()
        return [(x0 + i * res, y0 + j * res) for i, j in caminho]

    def travao(self, v):
        # nao avanca se nos proximos 0.8 s entrar na zona de uma arvore
        if v <= 0.0:
            return v
        for ax, ay in self.arvores:
            d_agora = math.hypot(self.ugv_x - ax, self.ugv_y - ay)
            for t in (0.2, 0.4, 0.6, 0.8):
                px = self.ugv_x + v * t * math.cos(self.ugv_yaw)
                py = self.ugv_y + v * t * math.sin(self.ugv_yaw)
                d = math.hypot(px - ax, py - ay)
                if d < self.raio_arvore - 0.02 and d < d_agora:
                    return 0.0
        return v

    def ugv_timer(self):
        if not (self.tem_ugv and self.tem_uav):
            self.get_logger().warn("À espera das odometrias do UGV e do UAV...", throttle_duration_sec=3.0)
            return

        agora = self.get_clock().now()
        dist = math.hypot(self.uav_x - self.ugv_x, self.uav_y - self.ugv_y)
        d_arvore = min((math.hypot(self.ugv_x - ax, self.ugv_y - ay) for ax, ay in self.arvores),
                       default=999.0)

        self.get_logger().info(
            f"Distância UGV-UAV: {dist:.1f} m | estado: {self.estado} | árvores: {len(self.arvores)} "
            f"| árvore mais próxima: {d_arvore:.2f} m", throttle_duration_sec=5.0)
        if d_arvore < self.raio_arvore - 0.05:
            self.get_logger().warn(f"UGV dentro da zona proibida ({d_arvore:.2f} m da árvore)!",
                                   throttle_duration_sec=1.0)

        # histerese: segue acima de dist_seguir, pára abaixo de dist_parar
        if self.estado == PARADO and dist > self.dist_seguir:
            self.estado = SEGUIR
            self.get_logger().info(f"A seguir o drone (distância {dist:.1f} m)")
        elif self.estado == SEGUIR and dist < self.dist_parar:
            self.estado = PARADO
            self.get_logger().info(f"Perto do drone ({dist:.1f} m): UGV parado")

        # estado do tether
        if dist >= self.dist_max:
            tether = 'LIMITE'
        elif dist > 0.8 * self.dist_max:
            tether = 'AVISO'
        else:
            tether = 'OK'
        if tether != self.estado_tether:
            self.get_logger().warn(f"Tether: {tether} (distância {dist:.1f} m)")
            self.estado_tether = tether
        tether_msg = String()
        tether_msg.data = tether
        self.tether_pub.publish(tether_msg)

        cmd = Twist()
        if self.estado == SEGUIR:
            objetivo = self.objetivo_perto_uav()
            alvo = objetivo

            # caminho que contorna as arvores (replaneado de 0.3 em 0.3 s)
            if len(self.arvores) > 0:
                if len(self.caminho) == 0 or (agora - self.t_plano).nanoseconds / 1e9 > 0.3:
                    self.caminho = self.planear_caminho(objetivo)
                    self.t_plano = agora
                    if len(self.caminho) == 0:
                        self.get_logger().warn("Sem caminho livre até ao drone!", throttle_duration_sec=2.0)
                # segue o primeiro ponto do caminho que esteja a mais de 1 m
                for px, py in self.caminho:
                    if math.hypot(px - self.ugv_x, py - self.ugv_y) > 1.0:
                        alvo = (px, py)
                        break
                else:
                    if len(self.caminho) > 0:
                        alvo = self.caminho[-1]

            dx, dy = alvo[0] - self.ugv_x, alvo[1] - self.ugv_y
            dist_alvo = math.hypot(dx, dy)
            erro_yaw = math.atan2(math.sin(math.atan2(dy, dx) - self.ugv_yaw),
                                  math.cos(math.atan2(dy, dx) - self.ugv_yaw))

            cmd.angular.z = float(np.clip(self.k_w * erro_yaw, -self.w_max, self.w_max))
            if dist_alvo > 0.1:
                cmd.linear.x = float(min(self.v_max, dist_alvo) * max(0.0, math.cos(erro_yaw)) ** 2)
            cmd.linear.x = self.travao(cmd.linear.x)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = seguir_drone()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()