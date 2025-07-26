import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineCenterFollower(Node):
    def __init__(self):
        super().__init__('line_center_follower')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.center_pub = self.create_publisher(Int32MultiArray, '/line_centers', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, w, _ = cv_image.shape

        # 색 범위 설정
        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([40, 255, 255])
        white_lower = np.array([0, 0, 200])
        white_upper = np.array([180, 30, 255])

        # 마스크 생성
        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        white_mask = cv2.inRange(hsv, white_lower, white_upper)

        # ROI 3등분 경계 설정
        thirds = [0, h // 3, 2 * h // 3, h]

        yellow_centers = []
        white_centers = []

        for i in range(3):
            y_roi = yellow_mask[thirds[i]:thirds[i+1], :]
            w_roi = white_mask[thirds[i]:thirds[i+1], :]
            cy = (thirds[i] + thirds[i+1]) // 2

            # 노란선 중심
            M_y = cv2.moments(y_roi)
            if M_y['m00'] > 0:
                y_cx = int(M_y['m10'] / M_y['m00'])
                yellow_centers.append(y_cx)
                cv2.circle(cv_image, (y_cx, cy), 5, (0, 255, 0), -1)  # 초록 (노란선)
            else:
                yellow_centers.append(-1)

            # 흰선 중심
            M_w = cv2.moments(w_roi)
            if M_w['m00'] > 0:
                w_cx = int(M_w['m10'] / M_w['m00'])
                white_centers.append(w_cx)
                cv2.circle(cv_image, (w_cx, cy), 5, (255, 0, 0), -1)  # 파랑 (흰선)
            else:
                white_centers.append(-1)

            # 중간점 시각화 (두 선 다 감지됐을 때)
            if yellow_centers[-1] != -1 and white_centers[-1] != -1:
                target_cx = (yellow_centers[-1] + white_centers[-1]) // 2
                cv2.circle(cv_image, (target_cx, cy), 5, (0, 0, 255), -1)  # 빨강 (중앙)

        # 퍼블리시: [yellow_near, yellow_mid, yellow_far, white_near, white_mid, white_far]
        msg_out = Int32MultiArray()
        msg_out.data = yellow_centers + white_centers
        self.center_pub.publish(msg_out)

        # 디버깅용 출력
        cv2.imshow("Line Center ROI x3", cv_image)
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
