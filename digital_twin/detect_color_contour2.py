# 색 인식 + 블러 + 컨투어 추출
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
        # 원본 이미지 수신
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w, _ = cv_image.shape

        # ================1. 블러 적용================
        blurred = cv_image # 블러 미적용
        # blurred = cv2.blur(cv_image, (5, 5)) # 평균 블러
        # blurred = cv2.GaussianBlur(cv_image, (5, 5), 0) # 가우시안 블러
        # blurred = cv2.medianBlur(cv_image, 5) # 미디언 블러


        # ================2. HSV 변환 (블러된 이미지 기준)================
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # ================3. 마스크 추출================
        # yellow_lower = np.array([10, 90, 100])
        # yellow_upper = np.array([45, 255, 255])
        
        yellow_lower = np.array([10, 60, 100])
        yellow_upper = np.array([50, 255, 255])
        
        white_lower = np.array([0, 0, 180])
        white_upper = np.array([180, 20, 255])

        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        white_mask = cv2.inRange(hsv, white_lower, white_upper)

        # ================4. 어둡게 침식 적용================
        kernel = np.ones((9, 3), np.uint8)
        white_mask = cv2.erode(white_mask, kernel, iterations=1)

        # # ✅ 5. 중심 좌표 계산
        # y_mom = cv2.moments(yellow_mask)
        # w_mom = cv2.moments(white_mask)

        # y_cx = w_cx = None
        # if y_mom['m00'] > 0:
        #     y_cx = int(y_mom['m10'] / y_mom['m00'])
        #     cv2.circle(cv_image, (y_cx, h // 2), 5, (0, 255, 255), -1)

        # if w_mom['m00'] > 0:
        #     w_cx = int(w_mom['m10'] / w_mom['m00'])
        #     cv2.circle(cv_image, (w_cx, h // 2), 5, (255, 255, 255), -1)

        # # ✅ 6. 주행 제어
        # twist = Twist()
        # if y_cx is not None and w_cx is not None:
        #     target_cx = (y_cx + w_cx) // 2
        #     cv2.circle(cv_image, (target_cx, h // 2), 5, (0, 0, 255), -1)
        #     error = (w // 2) - target_cx
        #     twist.linear.x = 0.1
        #     twist.angular.z = float(error) / 200.0
        # elif y_cx is not None:
        #     error = (w // 3) - y_cx
        #     twist.linear.x = 0.08
        #     twist.angular.z = float(error) / 200.0
        # elif w_cx is not None:
        #     error = (2 * w // 3) - w_cx
        #     twist.linear.x = 0.08
        #     twist.angular.z = float(error) / 200.0
        # else:
        #     twist.linear.x = 0.0
        #     twist.angular.z = 0.0

        # self.cmd_pub.publish(twist)

        # ✅ 7. 컨투어 추출 및 필터링
        contours_yellow, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_white, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        MIN_LENGTH = 600
        for cnt in contours_yellow:
            if cv2.arcLength(cnt, True) > 100:
                cv2.drawContours(cv_image, [cnt], -1, (0, 255, 0), 2)
        for cnt in contours_white:
            if cv2.arcLength(cnt, True) > MIN_LENGTH:
                cv2.drawContours(cv_image, [cnt], -1, (255, 0, 0), 2)
                
        # 빈 배경 이미지 생성 (검정색 배경)
        contour_only_view = np.zeros_like(cv_image)

        # 컨투어만 그린다 (선 두께는 2)
        for cnt in contours_yellow:
            if cv2.arcLength(cnt, True) > MIN_LENGTH:
                cv2.drawContours(contour_only_view, [cnt], -1, (0, 255, 0), 2)

        for cnt in contours_white:
            if cv2.arcLength(cnt, True) > MIN_LENGTH:
                cv2.drawContours(contour_only_view, [cnt], -1, (255, 0, 0), 2)

        # ✅ 8. 시각화
        cv2.imshow("Original", cv_image)
        cv2.imshow("Blurred", blurred)
        cv2.imshow("Yellow Mask", yellow_mask)
        cv2.imshow("White Mask", white_mask)
        combined_mask = cv2.bitwise_or(yellow_mask, white_mask)
        cv2.imshow("Combined Mask", combined_mask)
        cv2.imshow("Contour", contour_only_view)
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
