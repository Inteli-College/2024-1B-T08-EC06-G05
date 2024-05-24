import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import threading
# Configuração das constantes da câmera
IM_WIDTH = 1280
IM_HEIGHT = 720
class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')
        self.publisher_ = self.create_publisher(CompressedImage, '/video_frames', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # Publish every 0.1 seconds (10 Hz)
        self.cap = cv2.VideoCapture(0)
        self.latency_thread = threading.Thread(target=self.latencia)
        self.latency_thread.start()
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Encode frame as JPEG
            _, buffer = cv2.imencode('.jpg', frame)
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = buffer.tobytes()
            self.publisher_.publish(msg)
    def latencia(self):
        
        if self.cap is None or not self.cap.isOpened():
            print('\n\n')
            print('Error - could not open video device.')
            print('\n\n')
            exit(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, IM_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IM_HEIGHT)
        actual_video_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_video_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print('Actual video resolution: {:.0f}x{:.0f}'.format(actual_video_width, actual_video_height))
        prev_tick = cv2.getTickCount()
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            new_tick = cv2.getTickCount()
            latency = (new_tick - prev_tick) / cv2.getTickFrequency()
            print("Latency: {:.3f} sec".format(latency))
            prev_tick = new_tick
        self.cap.release()
        cv2.destroyAllWindows()
def main(args=None):
    rclpy.init(args=args)
    webcam_publisher = WebcamPublisher()
    rclpy.spin(webcam_publisher)
    webcam_publisher.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()