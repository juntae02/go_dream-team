import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineCenterFollower(Node):
    def __init__(self):
        super().__init__('line_center_follower')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, w, _ = cv_image.shape

        # 마스크 범위 설정
        # yellow_lower = np.array([20, 100, 100])
        # yellow_upper = np.array([40, 255, 255])
        # 밝은 부분 추가
        # yellow_lower = np.array([10, 70, 100])
        # yellow_upper = np.array([45, 255, 255])
        yellow_lower = np.array([10, 90, 100])
        yellow_upper = np.array([45, 255, 255])
        
        # 일반 밝기 모드
        white_lower = np.array([0, 0, 200])
        white_upper = np.array([180, 20, 255])
        
        # 어두운 모드
        # white_lower = np.array([0, 0, 130])
        # white_upper = np.array([180, 70, 255])

        # 마스크 생성
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        white_mask = cv2.inRange(hsv, white_lower, white_upper)

        # 중심 좌표 계산
        y_mom = cv2.moments(yellow_mask)
        w_mom = cv2.moments(white_mask)

        y_cx = w_cx = None

        if y_mom['m00'] > 0:
            y_cx = int(y_mom['m10'] / y_mom['m00'])
            cv2.circle(cv_image, (y_cx, h // 2), 5, (0, 255, 255), -1)

        if w_mom['m00'] > 0:
            w_cx = int(w_mom['m10'] / w_mom['m00'])
            cv2.circle(cv_image, (w_cx, h // 2), 5, (255, 255, 255), -1)

        twist = Twist()

        if y_cx is not None and w_cx is not None:
            # 두 선 모두 보이는 경우 → 가운데로 주행
            target_cx = (y_cx + w_cx) // 2
            cv2.circle(cv_image, (target_cx, h // 2), 5, (0, 0, 255), -1)
            error = (w // 2) - target_cx
            twist.linear.x = 0.1
            twist.angular.z = float(error) / 200.0

        elif y_cx is not None:
            # 노란선만 보이는 경우 → 왼쪽에 두고 직진 (조금 오른쪽 보고 감)
            error = (w // 3) - y_cx  # w/3은 왼쪽 1/3 지점
            twist.linear.x = 0.08
            twist.angular.z = float(error) / 200.0

        elif w_cx is not None:
            # 흰선만 보이는 경우 → 오른쪽에 두고 직진 (조금 왼쪽 보고 감)
            error = (2 * w // 3) - w_cx  # 2w/3은 오른쪽 1/3 지점
            twist.linear.x = 0.08
            twist.angular.z = float(error) / 200.0

        else:
            # 아무 선도 안 보이면 정지
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)
        
        # 컨투어 추출
        contours_yellow, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_white, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 최소 길이나 면적 기준
        # MIN_AREA = 1000    # 너무 작으면 무시 (면적)
        MIN_LENGTH = 300   # 너무 짧으면 무시 (둘 중 하나 기준 사용)

        # 필터링된 컨투어 시각화
        for cnt in contours_yellow:
            if cv2.arcLength(cnt, True) > MIN_LENGTH:
                cv2.drawContours(cv_image, [cnt], -1, (0, 255, 0), 2)

        for cnt in contours_white:
            if cv2.arcLength(cnt, True) > MIN_LENGTH:
                cv2.drawContours(cv_image, [cnt], -1, (255, 0, 0), 2)

        # ====================디버깅용 이미지 시각화====================
        cv2.imshow("Line Center Tracking", cv_image)
        cv2.imshow("Yellow Mask", yellow_mask)
        cv2.imshow("White Mask", white_mask)

        combined_mask = cv2.bitwise_or(yellow_mask, white_mask)
        cv2.imshow("Combined Mask", combined_mask)

        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LineCenterFollower()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
