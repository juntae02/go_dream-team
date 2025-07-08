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

        # 색상 마스크 설정
        yellow_lower = np.array([10, 90, 100])
        yellow_upper = np.array([45, 255, 255])

        white_lower = np.array([0, 0, 200])
        white_upper = np.array([180, 20, 255])

        # 마스크 생성
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        white_mask = cv2.inRange(hsv, white_lower, white_upper)

        # ===================1========================
        # combined_mask = cv2.bitwise_or(yellow_mask, white_mask)

        # # 직선 감지를 위한 전처리 (엣지)
        # edges = cv2.Canny(combined_mask, 50, 150, apertureSize=3)
        
        # ===================2========================        
        # 1. combined mask 생성
        combined_mask = cv2.bitwise_or(yellow_mask, white_mask)

        # 2. 가우시안 블러 추가
        blurred = cv2.GaussianBlur(combined_mask, (5, 5), 0)

        # 3. 블러 처리된 결과로 엣지 추출
        edges = cv2.Canny(blurred, 50, 150, apertureSize=3)


        # 허프 직선 검출
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=100, maxLineGap=10)

        cx_list = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 255), 2)
                cx = (x1 + x2) // 2
                cx_list.append(cx)

        twist = Twist()

        if cx_list:
            avg_cx = int(np.mean(cx_list))
            cv2.circle(cv_image, (avg_cx, h // 2), 5, (0, 0, 255), -1)
            error = (w // 2) - avg_cx
            twist.linear.x = 0.1
            twist.angular.z = float(error) / 200.0
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

        # 디버깅용 시각화
        cv2.imshow("Line Center Tracking", cv_image)
        cv2.imshow("Yellow Mask", yellow_mask)
        cv2.imshow("White Mask", white_mask)
        cv2.imshow("Combined Mask", combined_mask)
        cv2.imshow("Edges", edges)

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
