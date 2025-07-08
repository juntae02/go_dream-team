import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
import numpy as np

class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.bridge = CvBridge()

        # 카메라에서 직접 이미지 가져오기
        self.cap = cv2.VideoCapture(0)  # USB 카메라 0번 사용
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz 주기로 프레임 읽기

        self.Kp = 0.005
        self.Ki = 0.0001
        self.Kd = 0.001

        self.error_sum = 0.0
        self.last_error = 0.0
        self.base_speed = 0.15
        self.min_speed = 0.05

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('카메라 프레임을 읽을 수 없습니다.')
            return

        lane_center, processed_img = self.detect_lane(frame)
        if lane_center is not None:
            image_center = processed_img.shape[1] // 2
            error = image_center - lane_center
            angular_z = self.calculate_pid_control(error)
            linear_x = self.adjust_speed(error, angular_z)

            cmd = Twist()
            cmd.linear.x = linear_x
            cmd.angular.z = angular_z
            self.publisher.publish(cmd)

    def detect_lane(self, image):
        height, width = image.shape[:2]
        roi_vertices = np.array([[(0, height), (0, height * 0.6), (width, height * 0.6), (width, height)]], dtype=np.int32)
        mask = np.zeros_like(image)
        cv2.fillPoly(mask, roi_vertices, (255, 255, 255))
        masked = cv2.bitwise_and(image, mask)

        gray = cv2.cvtColor(masked, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=20, maxLineGap=150)
        line_img = np.zeros_like(image)

        left_lines = []
        right_lines = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1 + 1e-6)
                if abs(slope) < 0.3:
                    continue
                if slope < 0:
                    left_lines.append(line[0])
                else:
                    right_lines.append(line[0])
                cv2.line(line_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

        left_center = None
        right_center = None
        lane_center = None

        if len(left_lines) > 0:
            left_x = [line[0] for line in left_lines] + [line[2] for line in left_lines]
            left_center = int(sum(left_x) / len(left_x))

        if len(right_lines) > 0:
            right_x = [line[0] for line in right_lines] + [line[2] for line in right_lines]
            right_center = int(sum(right_x) / len(right_x))

        if left_center and right_center:
            lane_center = (left_center + right_center) // 2
        elif left_center:
            lane_center = left_center + width // 4
        elif right_center:
            lane_center = right_center - width // 4

        return lane_center, line_img

    def calculate_pid_control(self, error):
        p_term = self.Kp * error

        self.error_sum += error
        self.error_sum = max(-1000, min(1000, self.error_sum))
        i_term = self.Ki * self.error_sum

        d_term = self.Kd * (error - self.last_error)
        self.last_error = error

        angular_z = p_term + i_term + d_term
        return max(-1.0, min(1.0, angular_z))

    def adjust_speed(self, error, angular_z):
        speed_factor = 1.0 - min(1.0, abs(angular_z))
        linear_x = self.base_speed * speed_factor
        return max(self.min_speed, linear_x)

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.cap.release()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()