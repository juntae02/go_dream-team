import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineDetector(Node):
    def __init__(self):
        super().__init__('line_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # ROS 이미지 -> OpenCV 이미지로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 전처리: 크기 조정, 블러 등 필요시 수행
        resized = cv2.resize(frame, (640, 480))
        blurred = cv2.GaussianBlur(resized, (5, 5), 0)

        # HSV로 변환
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 색 추출 시 사용
        # H: 색 범위, S: 채도 허용, V: 밝기
        white_lower = np.array([0, 0, 30])
        white_upper = np.array([180, 50, 255])
        white_mask = cv2.inRange(hsv, white_lower, white_upper)

        # 노란색 필터링 1
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([30, 255, 255])
        
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

        # 마스크 결합
        mask = cv2.bitwise_or(white_mask, yellow_mask)
        mask = white_mask

        # 마스크 적용
        result = cv2.bitwise_and(resized, resized, mask=mask)

        # 디버그용 화면 출력
        cv2.imshow("Original", resized)
        cv2.imshow("Mask", mask)
        cv2.imshow("Filtered Lines", result)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
