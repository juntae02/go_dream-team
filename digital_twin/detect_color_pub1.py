# color detect + pub
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray  # ← 추가
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineCenterFollower(Node):
    def __init__(self):
        super().__init__('line_center_follower')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        # 👉 중심 퍼블리셔 추가
        self.center_pub = self.create_publisher(Int32MultiArray, '/line_centers', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, w, _ = cv_image.shape

        yellow_lower = np.array([20, 100, 100])
        yellow_upper = np.array([40, 255, 255])
        white_lower = np.array([0, 0, 200])
        white_upper = np.array([180, 30, 255])

        yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
        white_mask = cv2.inRange(hsv, white_lower, white_upper)

        y_mom = cv2.moments(yellow_mask)
        w_mom = cv2.moments(white_mask)        

        y_cx = w_cx = -1  # 초기값은 -1로 설정 (못 찾았을 때 표시)

        if y_mom['m00'] > 0:
            y_cx = int(y_mom['m10'] / y_mom['m00'])
            cv2.circle(cv_image, (y_cx, h // 2), 5, (0, 255, 0), -1)

        if w_mom['m00'] > 0:
            w_cx = int(w_mom['m10'] / w_mom['m00'])
            cv2.circle(cv_image, (w_cx, h // 2), 5, (255, 0, 0), -1)
        if w_mom['m00'] > 0 and y_mom['m00'] > 0:
            target_cx = (y_cx + w_cx) // 2
            cv2.circle(cv_image, (target_cx, h // 2), 5, (0, 0, 255), -1)

        # 👉 중심 퍼블리시
        center_msg = Int32MultiArray()
        center_msg.data = [y_cx, w_cx]
        self.center_pub.publish(center_msg)

        cv2.imshow("Line Center Tracking", cv_image)
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
