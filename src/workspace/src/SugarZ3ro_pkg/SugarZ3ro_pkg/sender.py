import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
import cv2
from datetime import datetime

# Configuração das constantes da câmera
IM_WIDTH = 1280
IM_HEIGHT = 720

class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, '/video_frames', 10)
        self.latency_publisher_ = self.create_publisher(String, '/latency', 60)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish every 0.1 seconds (10 Hz)
        self.latency_timer = self.create_timer(0.1, self.latency_callback)  # Calculate latency every 0.1 seconds
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            self.get_logger().error('Error - could not open video device.')
            rclpy.shutdown()
            return
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IM_HEIGHT)
        actual_video_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_video_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f'Actual video resolution: {actual_video_width:.0f}x{actual_video_height:.0f}')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Encode frame as JPEG
            _, buffer = cv2.imencode('.jpg', frame)
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = buffer.tobytes()
            self.publisher_.publish(msg)
        else:
            self.get_logger().error('Failed to read frame from camera.')

    def latency_callback(self):
        current_time = datetime.now().isoformat()
        self.get_logger().info(f'Sending timestamp: {current_time}')
        # Publish current timestamp
        latency_msg = String()
        latency_msg.data = current_time
        self.latency_publisher_.publish(latency_msg)

def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    rclpy.spin(webcam_publisher)
    webcam_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()