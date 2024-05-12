import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, os
import tty
if os.name == 'nt':
    import msvcrt, time

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
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

class Teleop(Node):
    def __init__(self):
        super().__init__('turtlebot3_teleop')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def getKey(self):
        if os.name == 'nt':
            timeout = 0.1
            startTime = time.time()
            while(1):
                if msvcrt.kbhit():
                    if sys.version_info[0] >= 3:
                        return msvcrt.getch().decode()
                    else:
                        return msvcrt.getch()
                elif time.time() - startTime > timeout:
                    return ''

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        return key

    def vels(self, target_linear_vel, target_angular_vel):
        return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

    def makeSimpleProfile(self, output, input, slop):
        if input > output:
            output = min( input, output + slop )
        elif input < output:
            output = max( input, output - slop )
        else:
            output = input

        return output

    def constrain(self, input, low, high):
        if input < low:
          input = low
        elif input > high:
          input = high
        else:
          input = input

        return input

    def checkLinearLimitVelocity(self, vel):
        vel = self.constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

        return vel

    def checkAngularLimitVelocity(self, vel):
        vel = self.constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
        return vel

    def run(self):
        status = 0
        target_linear_vel   = 0.0
        target_angular_vel  = 0.0
        control_linear_vel  = 0.0
        control_angular_vel = 0.0

        try:
            print(msg)
            while rclpy.ok():
                key = self.getKey()
                if key == 'w' :
                    target_linear_vel = self.checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                    status = status + 1
                    print(self.vels(target_linear_vel,target_angular_vel))
                elif key == 'x' :
                    target_linear_vel = self.checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                    status = status + 1
                    print(self.vels(target_linear_vel,target_angular_vel))
                elif key == 'a' :
                    target_angular_vel = self.checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(self.vels(target_linear_vel,target_angular_vel))
                elif key == 'd' :
                    target_angular_vel = self.checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                    status = status + 1
                    print(self.vels(target_linear_vel,target_angular_vel))
                elif key == ' ' or key == 's' :
                    target_linear_vel   = 0.0
                    control_linear_vel  = 0.0
                    target_angular_vel  = 0.0
                    control_angular_vel = 0.0
                    print(self.vels(target_linear_vel, target_angular_vel))
                else:
                    if (key == '\x03'):
                        break

                if status == 20 :
                    print(msg)
                    status = 0

                twist = Twist()

                control_linear_vel = self.makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
                twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

                control_angular_vel = self.makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
                twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

                self.publisher_.publish(twist)

        except KeyboardInterrupt:
            pass
        finally:
            twist = Twist()
            twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    teleop = Teleop()
    teleop.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
