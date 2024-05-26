import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import sys, select, os
import tty, termios, time
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data

if os.name == 'nt':
    import msvcrt

# Definindo constantes para a velocidade máxima do robô
BURGER_MAX_LIN_VEL = 0.20
BURGER_MAX_ANG_VEL = 2.5
LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.5

# Mensagem de instruções de controle
msg = """
Controle o robô SugarZ3ro
---------------------------
Controles:
        w
   a    s    d
w   : mova para frente
a/d : mova para esquerda/direita
s   : pare de andar
Q   : botão de emergência (interromper o programa)
"""

class Teleop(Node):
    def __init__(self):
        super().__init__('turtlebot3_teleop')

        # Inicializa o publisher para enviar comandos de velocidade
        self.publisher_ = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
        
        self.key_pressed = None
        self.last_key_pressed = None
        self.running = True  # Controle do ciclo de vida da thread
        self.lock = threading.Lock()
        self.mensagem = True

        # Inicializa o subscriber para receber dados do laser scan
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        
        self.min_distance = 1
        self.stop_distance = 0.3  # 30 cm
    
    # Callback para processar os dados do laser scan
    def scan_callback(self, msg):
        ranges = [distance for distance in msg.ranges if not distance == float('inf')]
        
        if ranges:
            self.min_distance = min(ranges)

    # Função para captar as teclas pressionadas pelo usuário
    def key_poll(self):
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while self.running:  # Verifica se ainda deve rodar
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    with self.lock:
                        self.key_pressed = sys.stdin.read(1)
                else:
                    with self.lock:
                        self.key_pressed = None
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)

    # Função principal para controlar o robô
    def run(self):
        target_linear_vel = 0.0
        target_angular_vel = 0.0
        key_thread = threading.Thread(target=self.key_poll)
        key_thread.start()

        try:
            print(msg)
            while rclpy.ok() and self.running:
                # Sistema de detecção de obstáculos
                if self.min_distance <= self.stop_distance:
                    print("OBSTÁCULO DETECTADO A 30cm!\nAfastando o robô do obstáculo...")
                    while self.min_distance <= self.stop_distance:
                        self.mensagem = True
                        obstacle_twist = Twist()
                        obstacle_twist.linear.x = float(target_linear_vel)
                        self.publisher_.publish(obstacle_twist)
                        target_linear_vel = -1.0
                        target_angular_vel = 0.0
                    print("O obstáculo não está mais a 30cm do robô.")

                with self.lock:
                    key = self.key_pressed
                    last_key = self.last_key_pressed

                if key == 'q':
                    print("Foi bom te conhecer...")
                    self.running = False  # Sinaliza para parar a thread
                    break  # Sai do loop para parar o nó

                # Reseta as velocidades se necessário ao trocar de tecla
                if key != last_key:
                    if key in ['w', 's', None]:
                        target_angular_vel = 0.0
                    if key in ['a', 'd', 's', None]:
                        target_linear_vel = 0.0

                # Controle de movimento do robô
                if key == 'w':
                    if self.mensagem:
                        print("Andando para frente...")
                    self.mensagem = False
                    target_linear_vel = min(target_linear_vel + LIN_VEL_STEP_SIZE, BURGER_MAX_LIN_VEL)
                elif key == 'a':
                    if self.mensagem:
                        print("Virando para esquerda...")
                    self.mensagem = False
                    target_angular_vel = min(target_angular_vel + ANG_VEL_STEP_SIZE, BURGER_MAX_ANG_VEL)
                elif key == 'd':
                    if self.mensagem:
                        print("Virando para direita...")
                    self.mensagem = False
                    target_angular_vel = max(target_angular_vel - ANG_VEL_STEP_SIZE, -BURGER_MAX_ANG_VEL)
                elif key == 's' or key is None:
                    if not self.mensagem:
                        print("Parando...")
                    self.mensagem = True
                    target_linear_vel = 0.0
                    target_angular_vel = 0.0

                self.last_key_pressed = key  # Atualiza a última tecla pressionada

                twist = Twist()
                twist.linear.x = float(target_linear_vel)
                twist.angular.z = float(target_angular_vel)
                self.publisher_.publish(twist)
                time.sleep(0.1)

        except KeyboardInterrupt:
            pass
        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.running = False
            key_thread.join()

def main(args=None):
    rclpy.init(args=args)
    teleop = Teleop()

    # Cria uma thread para o spinning
    spin_thread = threading.Thread(target=rclpy.spin, args=(teleop,))
    spin_thread.start()

    try:
        teleop.run()
    finally:
        teleop.running = False  # Sinaliza para parar a thread
        spin_thread.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
