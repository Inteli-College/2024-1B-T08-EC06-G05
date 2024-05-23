# obstacle_detection.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class ObstacleDetection(Node):
    def __init__(self):
        super().__init__('obstacle_detection')
        self.scan_subscriber = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.obstacle_publisher = self.create_publisher(Bool, 'obstacle_detected', 10)
        self.stop_distance = 0.3  # 30 cm
        self.get_logger().info("Nó de detecção de obstáculos iniciado")

    def scan_callback(self, msg):

        ranges = [distance for distance in msg.ranges if not distance == float('inf')]

        if ranges:
            min_distance = min(ranges)
            self.get_logger().info(f"Menor distância: {min_distance}")

            if min_distance <= self.stop_distance:
                self.obstacle_publisher.publish(Bool(data=True))
                self.get_logger().warn("Obstáculo detectado a 30cm!")

            else:
                self.obstacle_publisher.publish(Bool(data=False))

def main(args=None):
    rclpy.init(args=args)
    obstacle_detection = ObstacleDetection()
    rclpy.spin(obstacle_detection)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
