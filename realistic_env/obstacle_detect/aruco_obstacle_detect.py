import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

def detect_colored_obstacle(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_orange = np.array([5, 120, 80])
    upper_orange = np.array([25, 255, 255])
    lower_blue = np.array([105, 70, 70])
    upper_blue = np.array([130, 255, 255])
    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
    mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
    mask_obstacle = cv2.bitwise_or(mask_orange, mask_blue)

    # morphology 연산 (노이즈 제거)
    kernel = np.ones((5, 5), np.uint8)
    mask_obstacle = cv2.morphologyEx(mask_obstacle, cv2.MORPH_OPEN, kernel)
    mask_obstacle = cv2.morphologyEx(mask_obstacle, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask_obstacle, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    max_area = 0
    cx, cy = None, None
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 1000 and area > max_area:
            x, y, w, h = cv2.boundingRect(cnt)
            cx = x + w // 2
            cy = y + h // 2
            max_area = area
    return (cx, cy) if max_area > 0 else (None, None)

class MixedFollower(Node):
    def __init__(self):
        super().__init__('mixed_follower')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.last_processed_time = 0

    def image_callback(self, msg):
        now = time.time()
        if now - self.last_processed_time < 0.1:  # 0.1초(10fps)마다만 처리
            return
        self.last_processed_time = now

        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = cv_image.shape

        # 장애물 중심 검출
        cx, cy = detect_colored_obstacle(cv_image)
        # 장애물 타겟 위치 (화면 상단 가운데, y=15~20%쯤)
        target_x = w // 2
        target_y = int(h * 0.18)
        twist = Twist()

        # 1. 장애물이 있으면 우선적으로 정렬 (속도 감속 로직 추가!)
        if cx is not None and cy is not None:
            error_x = target_x - cx
            error_y = target_y - cy

            # (예시) 오차 20픽셀 이내면 "정렬 완료"
            if abs(error_x) < 20 and abs(error_y) < 20:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info("장애물 정렬 완료! 그리퍼 동작 시작")
            else:
                # >>> 속도 감속 (값 바꿔가며 실험해봐도 좋아!)
                angular_gain = 1.5   # 클수록 더 천천히 돌음
                linear_gain = 0.5    # 클수록 더 천천히 움직임
                twist.angular.z = float(error_x) / (w * angular_gain)
                twist.linear.x = 0.0
                if abs(error_y) > 10:
                    twist.linear.x = 0.05 if error_y > 0 else -0.02
                    twist.linear.x *= linear_gain

                # 오차가 작아질수록 더 천천히 (미세조정)
                if abs(error_x) < 50:
                    twist.angular.z *= 0.3

            # 시각화
            cv2.circle(cv_image, (target_x, target_y), 10, (0,255,255), 2)   # 노란 목표점
            cv2.circle(cv_image, (cx, cy), 10, (0,0,255), 2)                # 빨간 장애물 center

        # 2. 장애물 없으면 원래 라인 따라가기
        else:
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            yellow_lower = np.array([18, 60, 60])
            yellow_upper = np.array([40, 255, 255])
            white_lower = np.array([0, 0, 100])
            white_upper = np.array([180, 90, 255])
            yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
            white_mask = cv2.inRange(hsv, white_lower, white_upper)
            y_mom = cv2.moments(yellow_mask)
            w_mom = cv2.moments(white_mask)
            y_cx = w_cx = None
            if y_mom['m00'] > 0:
                y_cx = int(y_mom['m10'] / y_mom['m00'])
            if w_mom['m00'] > 0:
                w_cx = int(w_mom['m10'] / w_mom['m00'])
            if y_cx is not None and w_cx is not None:
                target_cx = (y_cx + w_cx) // 2
                error = (w // 2) - target_cx
                twist.linear.x = 0.1
                twist.angular.z = float(error) / 200.0
            elif y_cx is not None:
                error = (w // 3) - y_cx
                twist.linear.x = 0.08
                twist.angular.z = float(error) / 200.0
            elif w_cx is not None:
                error = (2 * w // 3) - w_cx
                twist.linear.x = 0.08
                twist.angular.z = float(error) / 200.0
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0

        self.cmd_pub.publish(twist)
        cv2.imshow("Mixed Follower", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MixedFollower()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
