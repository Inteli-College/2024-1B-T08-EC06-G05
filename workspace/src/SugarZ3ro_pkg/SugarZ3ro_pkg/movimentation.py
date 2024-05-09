import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import sys, select, os
import tty, termios, time

if os.name == 'nt':
    import msvcrt

BURGER_MAX_LIN_VEL = 0.20
BURGER_MAX_ANG_VEL = 2.5
LIN_VEL_STEP_SIZE = 0.1
ANG_VEL_STEP_SIZE = 0.5

msg = """
Controle o nosso querido robô SugarZ3ro
---------------------------
Controles:
        w
   a    s    d
w : mova para frente
a/d : mova para a direita/esquerda
s : pare de andar
Q para sair do programa
"""

class Teleop(Node):
    def __init__(self):
        super().__init__('turtlebot3_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.key_pressed = None
        self.last_key_pressed = None
        self.running = True  # To control thread lifecycle
        self.lock = threading.Lock()
        self.mensagem = True

    def key_poll(self):
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while self.running:  # Check if still supposed to run
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
            while rclpy.ok():
                with self.lock:
                    key = self.key_pressed
                    last_key = self.last_key_pressed

                if key == 'q':
                    print("Foi bom te conhecer...")
                    self.running = False  # Signal thread to stop
                    break  # Exit the loop to stop the node

                # Reset speeds if necessary when switching keys
                if key != last_key:
                    if key in ['w', 's', None]:
                        target_angular_vel = 0.0
                    if key in ['a', 'd', 's', None]:
                        target_linear_vel = 0.0

                if key == 'w':
                    if self.mensagem:
                        print("Andando para frente")
                    self.mensagem = False
                    target_linear_vel = min(target_linear_vel + LIN_VEL_STEP_SIZE, BURGER_MAX_LIN_VEL)
                elif key == 'a':
                    if self.mensagem:
                        print("Andando para esquerda")
                    self.mensagem = False
                    target_angular_vel = min(target_angular_vel + ANG_VEL_STEP_SIZE, BURGER_MAX_ANG_VEL)
                elif key == 'd':
                    if self.mensagem:
                        print("Andando para direita")
                    self.mensagem = False
                    target_angular_vel = max(target_angular_vel - ANG_VEL_STEP_SIZE, -BURGER_MAX_ANG_VEL)
                elif key == 's' or key is None:
                    if not self.mensagem:
                        print("Parando")
                    self.mensagem = True
                    target_linear_vel = 0.0
                    target_angular_vel = 0.0

                self.last_key_pressed = key  # Update the last pressed key

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
    teleop.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
