# 색 인식 + 블러 + 컨투어 추출 + 퍼블리싱
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineCenterPublisher(Node):
    def __init__(self):
        super().__init__('line_center_publisher')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # 퍼블리셔 추가
        self.yellow_center_pub = self.create_publisher(Int32, '/yellow_center_x', 10)
        self.white_center_pub = self.create_publisher(Int32, '/white_center_x', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = cv_image.shape

        # 1. 블러 적용 (선택)
        blurred = cv_image  # 필요시 GaussianBlur 등으로 변경 가능

        # 2. HSV 변환
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # 3. 색상 범위 정의 및 마스크
        yellow_lower = np.array([10, 60, 100])
        yellow_upper = np.array([50, 255, 255])
        white_lower = np.array([0, 0, 180])
        white_upper = np.array([180, 20, 255])

        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        white_mask = cv2.inRange(hsv, white_lower, white_upper)

        # 4. 침식 적용
        kernel = np.ones((9, 3), np.uint8)
        white_mask = cv2.erode(white_mask, kernel, iterations=1)

        # 5. 컨투어 추출
        contours_yellow, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_white, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        MIN_LENGTH = 600

        # 6. 가장 긴 노란선 컨투어
        longest_yellow = max(
            [cnt for cnt in contours_yellow if cv2.arcLength(cnt, True) > MIN_LENGTH],
            key=lambda c: cv2.arcLength(c, True),
            default=None
        )

        # 7. 가장 긴 흰선 컨투어
        longest_white = max(
            [cnt for cnt in contours_white if cv2.arcLength(cnt, True) > MIN_LENGTH],
            key=lambda c: cv2.arcLength(c, True),
            default=None
        )

        # 8. 중심 계산 및 퍼블리시
        if longest_yellow is not None:
            M = cv2.moments(longest_yellow)
            if M['m00'] != 0:
                y_cx = int(M['m10'] / M['m00'])
                self.yellow_center_pub.publish(Int32(data=y_cx))
                # cv2.circle(cv_image, (y_cx, h // 2), 5, (0, 255, 255), -1)
                # 노란선 중심점
                cv2.circle(cv_image, (y_cx, h // 2), 5, (255, 0, 0), -1)  # 파란색 점


        if longest_white is not None:
            M = cv2.moments(longest_white)
            if M['m00'] != 0:
                w_cx = int(M['m10'] / M['m00'])
                self.white_center_pub.publish(Int32(data=w_cx))
                # cv2.circle(cv_image, (w_cx, h // 2), 5, (255, 255, 255), -1)
                # 흰선 중심점
                cv2.circle(cv_image, (w_cx, h // 2), 5, (0, 0, 255), -1)  # 빨간색 점

        # 9. 디버깅용 시각화 (선택)
        cv2.imshow("Processed", cv_image)
        cv2.imshow("Yellow Mask", yellow_mask)
        cv2.imshow("White Mask", white_mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineCenterPublisher()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
