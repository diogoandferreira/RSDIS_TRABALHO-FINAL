import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def normalizar_angulo(a):
    return math.atan2(math.sin(a), math.cos(a))


def ajustar_objetivo(objetivo, robo, arvores, r_seg):
    """Se o objetivo (posição do drone) estiver em cima/perto de uma árvore, desloca-o
    para a circunferência de segurança, do lado do robô. Assim o UGV pára AO LADO da árvore."""
    gx, gy = objetivo
    for _ in range(2):
        for ax, ay in arvores:
            vx, vy = gx - ax, gy - ay
            d = math.hypot(vx, vy)
            if d < r_seg:
                if d < 1e-3:                       # drone mesmo em cima do centro da árvore
                    vx, vy = robo[0] - ax, robo[1] - ay
                    d = math.hypot(vx, vy)
                    if d < 1e-3:
                        vx, vy, d = 1.0, 0.0, 1.0
                gx = ax + vx / d * r_seg
                gy = ay + vy / d * r_seg
    return gx, gy


def calcular_velocidade(robo, objetivo, arvores, p, seguir=True):
    """Campo de potenciais: atração para o objetivo + repulsão (e desvio tangencial) das árvores.
    Devolve (vx, vy) desejados no mundo e a folga mínima (m) às árvores."""
    rx, ry = robo
    att_dir = (0.0, 0.0)
    ax_, ay_ = 0.0, 0.0
    if seguir:
        dx, dy = objetivo[0] - rx, objetivo[1] - ry
        dist = math.hypot(dx, dy)
        if dist > 0.05:
            att_dir = (dx / dist, dy / dist)
            v_att = min(p['v_max'], p['k_att'] * dist)
            ax_, ay_ = att_dir[0] * v_att, att_dir[1] * v_att

    rep_x = rep_y = 0.0
    c_min = float('inf')
    for tx, ty in arvores:
        vx, vy = rx - tx, ry - ty
        d = math.hypot(vx, vy)
        if d < 1e-3:
            continue
        c = d - p['r_arvore'] - p['r_robo']          # folga entre a superfície da árvore e o robô
        c_min = min(c_min, c)
        if c < p['c0']:
            c_eff = max(c, 0.03)
            mag = p['k_rep'] * (1.0 / c_eff - 1.0 / p['c0'])
            ux, uy = vx / d, vy / d                   # afastar da árvore
            rep_x += mag * ux
            rep_y += mag * uy
            # componente tangencial: contornar a árvore do lado do objetivo
            tgx, tgy = -uy, ux
            if att_dir[0] * tgx + att_dir[1] * tgy < 0:
                tgx, tgy = -tgx, -tgy
            rep_x += p['k_tan'] * mag * tgx
            rep_y += p['k_tan'] * mag * tgy

    vx_t, vy_t = ax_ + rep_x, ay_ + rep_y
    n = math.hypot(vx_t, vy_t)
    if n > p['v_max']:
        vx_t, vy_t = vx_t / n * p['v_max'], vy_t / n * p['v_max']
    return vx_t, vy_t, c_min


def vel_para_comando(vx, vy, yaw, p, c_min):
    """Converte a velocidade desejada (mundo) em (v, w) para um robô diferencial."""
    n = math.hypot(vx, vy)
    if n < 0.02:
        return 0.0, 0.0
    err = normalizar_angulo(math.atan2(vy, vx) - yaw)
    w = float(np.clip(p['k_w'] * err, -p['w_max'], p['w_max']))
    v = n * max(0.0, math.cos(err))                   # se estiver virado para o lado errado, roda primeiro
    if c_min < p['c_lento']:                           # perto de uma árvore: abrandar
        v *= float(np.clip(c_min / p['c_lento'], 0.3, 1.0))
    return v, w


def limitar_colisao(robo, yaw, v, arvores, p):
    """Travão de segurança: se avançar ao longo do rumo atual levar o robô a menos de c_stop
    da zona da árvore (raio r_arvore + r_robo) E mais perto do que está agora, não avança
    (o robô só roda). Nunca impede de se afastar."""
    if v <= 0.0 or not arvores:
        return v
    raio = p['r_arvore'] + p['r_robo']
    passo = 0.1
    n = max(1, int(p['t_previsao'] / passo))
    for tx, ty in arvores:
        d_agora = math.hypot(robo[0] - tx, robo[1] - ty) - raio
        for i in range(1, n + 1):
            s = v * passo * i
            px = robo[0] + s * math.cos(yaw)
            py = robo[1] + s * math.sin(yaw)
            d_prev = math.hypot(px - tx, py - ty) - raio
            if d_prev < p['c_stop'] and d_prev < d_agora:
                return 0.0
    return v


# ======================================================================
# Nó ROS 2
# ======================================================================
class seguir_drone(Node):
    """O UGV (TurtleBot3) persegue o drone para a distância nunca passar de dist_max (trela),
    contornando as árvores já registadas pelo drone."""

    def __init__(self):
        super().__init__('seguir_drone_node')

        # ---------------- Parametros (aceitam inteiro ou decimal) ----------------
        self.declarar('dist_max', 20.0)      # trela: distância máxima UGV-UAV (m)
        self.declarar('dist_seguir', 10.0)   # começa a perseguir acima disto
        self.declarar('dist_parar', 5.0)     # pára de perseguir abaixo disto
        self.declarar('v_max', 0.5)          # m/s (o waffle real faz 0.26)
        self.declarar('w_max', 1.0)          # rad/s
        self.declarar('k_w', 1.5)
        self.declarar('k_att', 1.0)
        self.declarar('r_arvore', 0.6)       # raio considerado para cada árvore (m)
        self.declarar('r_robo', 0.25)        # raio aproximado do TurtleBot3 waffle
        self.declarar('margem', 0.35)        # folga extra ao parar ao lado de uma árvore
        self.declarar('c0', 1.0)             # folga a partir da qual as árvores repelem
        self.declarar('k_rep', 0.15)
        self.declarar('k_tan', 0.6)
        self.declarar('c_lento', 0.5)        # folga abaixo da qual abranda
        self.declarar('c_stop', 0.05)        # travão: folga mínima tolerada
        self.declarar('t_previsao', 0.8)     # s de previsão do travão
        self.declarar('spawn_x', 0.0)        # onde o TurtleBot nasce (odom -> mundo)
        self.declarar('spawn_y', 0.0)
        self.declarar('topico_drone', '/X3/gazebo/odometry')
        # posições de árvores já conhecidas: [x1, y1, x2, y2, ...] (opcional)
        self.declarar('arvores_conhecidas', [])   # aceita lista vazia ou [x1,y1,x2,y2,...]

        nomes = ['dist_max', 'dist_seguir', 'dist_parar', 'v_max', 'w_max', 'k_w', 'k_att',
                 'r_arvore', 'r_robo', 'margem', 'c0', 'k_rep', 'k_tan', 'c_lento',
                 'c_stop', 't_previsao', 'spawn_x', 'spawn_y']
        self.p = {n: float(self.get_parameter(n).value) for n in nomes}
        self.r_seg = self.p['r_arvore'] + self.p['r_robo'] + self.p['margem']

        # ---------------- Estado ----------------
        self.robo = None          # (x, y) no mundo
        self.yaw = 0.0
        self.drone = None         # (x, y) no mundo
        self.arvores = []         # [(x, y), ...]
        self.seguir = False
        self.estado_tether = 'OK'

        conhecidas = list(self.get_parameter('arvores_conhecidas').value or [])
        for i in range(0, len(conhecidas) - 1, 2):
            self.adicionar_arvore(conhecidas[i], conhecidas[i + 1], log=False)

        # ---------------- ROS ----------------
        self.create_subscription(Odometry, '/odom', self.odom_robo_cb, 10)
        self.create_subscription(Odometry, self.get_parameter('topico_drone').value,
                                 self.odom_drone_cb, 10)
        self.create_subscription(Point, '/arvores_detetadas', self.arvore_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tether_pub = self.create_publisher(String, '/tether_status', 10)

        self.create_timer(0.1, self.controlo)
        self.get_logger().info(
            f'UGV a seguir o drone (trela {self.p["dist_max"]} m). '
            f'{len(self.arvores)} árvores conhecidas à partida.')

    def declarar(self, nome, valor):
        self.declare_parameter(nome, valor, ParameterDescriptor(dynamic_typing=True))

    # ------------------------------------------------------------------
    def odom_robo_cb(self, msg):
        pos, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.robo = (pos.x + self.p['spawn_x'], pos.y + self.p['spawn_y'])
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def odom_drone_cb(self, msg):
        pos = msg.pose.pose.position
        self.drone = (pos.x, pos.y)

    def arvore_cb(self, msg):
        self.adicionar_arvore(msg.x, msg.y)

    def adicionar_arvore(self, x, y, log=True):
        # ignora repetidas (a mesma árvore registada duas vezes)
        if any(math.hypot(x - ax, y - ay) < 0.3 for ax, ay in self.arvores):
            return
        self.arvores.append((float(x), float(y)))
        if log:
            self.get_logger().info(f'Árvore registada no UGV: ({x:.2f}, {y:.2f}) | total {len(self.arvores)}')

    # ------------------------------------------------------------------
    def controlo(self):
        if self.robo is None or self.drone is None:
            self.get_logger().warn('À espera das odometrias do UGV e do UAV...', throttle_duration_sec=3.0)
            self.cmd_pub.publish(Twist())
            return

        dx, dy = self.drone[0] - self.robo[0], self.drone[1] - self.robo[1]
        dist = math.hypot(dx, dy)

        self.get_logger().info(
            f'Distância UGV-UAV: {dist:.1f} m | a seguir: {self.seguir} | árvores: {len(self.arvores)}',
            throttle_duration_sec=5.0)

        # histerese: persegue acima de dist_seguir, pára abaixo de dist_parar
        if dist > self.p['dist_seguir'] and not self.seguir:
            self.seguir = True
            self.get_logger().info(f'A perseguir o drone (distância {dist:.1f} m)')
        elif dist < self.p['dist_parar'] and self.seguir:
            self.seguir = False
            self.get_logger().info(f'Perto do drone ({dist:.1f} m): UGV parado')

        # estado da trela
        if dist >= self.p['dist_max']:
            estado = 'LIMITE'
        elif dist > 0.8 * self.p['dist_max']:
            estado = 'AVISO'
        else:
            estado = 'OK'
        if estado != self.estado_tether:
            self.get_logger().warn(f'Trela: {estado} (distância {dist:.1f} m)')
            self.estado_tether = estado
        s = String()
        s.data = estado
        self.tether_pub.publish(s)

        # objetivo = posição do drone, mas nunca em cima de uma árvore
        objetivo = ajustar_objetivo(self.drone, self.robo, self.arvores, self.r_seg)
        vx, vy, c_min = calcular_velocidade(self.robo, objetivo, self.arvores, self.p, self.seguir)
        v, w = vel_para_comando(vx, vy, self.yaw, self.p, c_min)
        v = limitar_colisao(self.robo, self.yaw, v, self.arvores, self.p)

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = seguir_drone()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()