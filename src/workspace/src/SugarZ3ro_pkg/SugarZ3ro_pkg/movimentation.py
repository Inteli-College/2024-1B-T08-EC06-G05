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

BURGER_MAX_LIN_VEL = 0.20
BURGER_MAX_ANG_VEL = 2.5
LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.5

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
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.key_pressed = None
        self.last_key_pressed = None
        self.running = True
        self.lock = threading.Lock()
        self.mensagem = True
        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
        
        self.min_distance_ahead = float('inf')
        self.min_distance_left = float('inf')
        self.min_distance_right = float('inf')
        self.stop_distance = 0.3

        self.log_counter = 0  # Contador para limitar a frequência das mensagens de log

    def scan_callback(self, msg):
        front_angles = range(len(msg.ranges)//2 - 10, len(msg.ranges)//2 + 10)
        left_angles = range(len(msg.ranges)//4*3 - 10, len(msg.ranges)//4*3 + 10)
        right_angles = range(len(msg.ranges)//4 - 10, len(msg.ranges)//4 + 10)
        
        self.min_distance_ahead = min([msg.ranges[i] for i in front_angles if 0 < msg.ranges[i] < float('inf')], default=float('inf'))
        self.min_distance_left = min([msg.ranges[i] for i in left_angles if 0 < msg.ranges[i] < float('inf')], default=float('inf'))
        self.min_distance_right = min([msg.ranges[i] for i in right_angles if 0 < msg.ranges[i] < float('inf')], default=float('inf'))

        # Reduzir a frequência de log
        self.log_counter += 1
        if self.log_counter % 10 == 0:  # Ajuste o valor conforme necessário para controlar a frequência
            self.get_logger().info(f'Distância à frente: {self.min_distance_ahead:.2f}m, à esquerda: {self.min_distance_left:.2f}m, à direita: {self.min_distance_right:.2f}m')

    def key_poll(self):
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while self.running:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    with self.lock:
                        self.key_pressed = sys.stdin.read(1)
                else:
                    with self.lock:
                        self.key_pressed = None
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attr)

    def run(self):
        target_linear_vel = 0.0
        target_angular_vel = 0.0
        key_thread = threading.Thread(target=self.key_poll)
        key_thread.start()

        try:
            print(msg)
            while rclpy.ok() and self.running:
                with self.lock:
                    key = self.key_pressed
                    last_key = self.last_key_pressed

                if key == 'q':
                    print("Foi bom te conhecer...")
                    self.running = False
                    break

                if key != last_key:
                    if key in ['w', 's', None]:
                        target_angular_vel = 0.0
                    if key in ['a', 'd', 's', None]:
                        target_linear_vel = 0.0

                if key == 'w':
                    if self.min_distance_ahead > self.stop_distance:
                        if self.mensagem:
                            print("Andando para frente...")
                        self.mensagem = False
                        target_linear_vel = min(target_linear_vel + LIN_VEL_STEP_SIZE, BURGER_MAX_LIN_VEL)
                    else:
                        if self.mensagem:
                            print("Obstáculo à frente! Movimentação para frente bloqueada.")
                        target_linear_vel = 0.0

                elif key == 'a':
                    if self.min_distance_left > self.stop_distance:
                        if self.mensagem:
                            print("Virando para esquerda...")
                        self.mensagem = False
                        target_angular_vel = min(target_angular_vel + ANG_VEL_STEP_SIZE, BURGER_MAX_ANG_VEL)
                    else:
                        if self.mensagem:
                            print("Obstáculo à esquerda! Movimentação para esquerda bloqueada.")
                        target_angular_vel = 0.0

                elif key == 'd':
                    if self.min_distance_right > self.stop_distance:
                        if self.mensagem:
                            print("Virando para direita...")
                        self.mensagem = False
                        target_angular_vel = max(target_angular_vel - ANG_VEL_STEP_SIZE, -BURGER_MAX_ANG_VEL)
                    else:
                        if self.mensagem:
                            print("Obstáculo à direita! Movimentação para direita bloqueada.")
                        target_angular_vel = 0.0

                elif key == 's':
                    if self.mensagem:
                        print("Parando...")
                    self.mensagem = True
                    target_linear_vel = -min(target_linear_vel + LIN_VEL_STEP_SIZE, BURGER_MAX_LIN_VEL)
                    target_angular_vel = 0.0

                self.last_key_pressed = key

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
    spin_thread = threading.Thread(target=rclpy.spin, args=(teleop,))
    spin_thread.start()

    try:
        teleop.run()
    finally:
        teleop.running = False
        spin_thread.join()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
