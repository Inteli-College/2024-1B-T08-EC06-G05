import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import sys, select, os
import tty, termios, time

if os.name == 'nt':
    import msvcrt

BURGER_MAX_LIN_VEL = 0.15
BURGER_MAX_ANG_VEL = 2.5
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
w : increase linear velocity (Burger : ~ 0.15)
a/d : increase/decrease angular velocity (Burger : ~ 2.5)
s : force stop
CTRL-C to quit
"""

class Teleop(Node):
    def __init__(self):
        super().__init__('turtlebot3_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.key_pressed = None
        self.last_key_pressed = None
        self.lock = threading.Lock()

    def key_poll(self):
        old_attr = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        try:
            while True:
                if select.select([sys.stdin], [], [], 0.1)[0]:  # Wait for input for up to 0.1 seconds
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

                # Reset speeds if necessary when switching keys
                if key != last_key:
                    if key in ['w', 's', None]:
                        target_angular_vel = 0.0
                    if key in ['a', 'd', 's', None]:
                        target_linear_vel = 0.0

                if key == 'w':
                    target_linear_vel = min(target_linear_vel + LIN_VEL_STEP_SIZE, BURGER_MAX_LIN_VEL)
                elif key == 'a':
                    target_angular_vel = min(target_angular_vel + ANG_VEL_STEP_SIZE, BURGER_MAX_ANG_VEL)
                elif key == 'd':
                    target_angular_vel = max(target_angular_vel - ANG_VEL_STEP_SIZE, -BURGER_MAX_ANG_VEL)
                elif key == 's' or key is None:
                    target_linear_vel = 0.0
                    target_angular_vel = 0.0

                self.last_key_pressed = key  # Update the last pressed key

                twist = Twist()
                twist.linear.x = float(target_linear_vel)
                twist.angular.z = float(target_angular_vel)
                self.publisher_.publish(twist)
                time.sleep(0.1)  # Adjust sleep time if needed

        except KeyboardInterrupt:
            pass
        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            key_thread.join()

def main(args=None):
    rclpy.init(args=args)
    teleop = Teleop()
    teleop.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
