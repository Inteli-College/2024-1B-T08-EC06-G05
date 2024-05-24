# obstacle_detection.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class ObstacleDetection(Node):
    def __init__(self):
        super().__init__('obstacle_detection')

        self.scan_subscriber = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)

        self.stop_distance = 0.3  # 30 cm

        self.vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10)
        self.get_logger().info("Nó de de parada")
    
    def scan_callback(self, msg):

        ranges = [distance for distance in msg.ranges if not distance == float('inf')]

        msg_parada = Twist()
        msg_parada.angular.z = 0.0
        msg_parada.linear.x = 0.0

        if ranges:
            min_distance = min(ranges)
            self.get_logger().info(f"Menor distância: {min_distance}")

            if min_distance <= self.stop_distance:
                self.get_logger().warn("Obstáculo detectado a 30cm!")
                self.vel_publisher.publish(msg_parada)
                print("PARANDO O ROBÔ")

def main(args=None):
    rclpy.init(args=args)
    obstacle_detection = ObstacleDetection()
    rclpy.spin(obstacle_detection)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
