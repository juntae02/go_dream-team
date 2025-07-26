import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        timer_period = 0.033  # 30FPS 주기
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 웹캠 사용
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("웹캠을 열 수 없습니다.")

        self.bridge = CvBridge()

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.get_logger().warn('프레임을 읽을 수 없습니다.')
            return

        # OpenCV 이미지를 ROS Image 메시지로 변환
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing /camera/image_raw')

    def __del__(self):
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    video_publisher = VideoPublisher()

    try:
        rclpy.spin(video_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        video_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
