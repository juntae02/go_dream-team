import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObstaclePub(Node):
    def __init__(self):
        super().__init__('obstacle_pub')
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.img_callback, 10)
        self.image_pub = self.create_publisher(Image, '/obstacle_image', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

        # 이전 프레임의 원형 장애물 크기 저장
        self.prev_circle_area_b = None 
        self.prev_circle_area_g = None 
        # 장애물 상태가 활성화 상태인지 저장
        self.danger_active = False
        # 현재 설정된 속도
        self.current_speed = 0.0

    def img_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        draw_img = img.copy()
        image_area = img.shape[0] * img.shape[1]

        # 각 장애물 처리
        danger_b = self.process_blue_obstacle(img, draw_img, image_area)    # 파란색: 좌우
        danger_g = self.process_green_obstacle(img, draw_img, image_area)   # 초록색: 기역자
        danger_r = self.process_red_speedbump(img, draw_img, image_area)  # 빨간색: 방지턱

        twist = Twist()

        # 장애물이 감지되었을 경우
        if danger_b or danger_g or danger_r:
            # 현재 판단된 속도를 전송
            twist.linear.x = self.current_speed
            self.cmd_vel_pub.publish(twist)
            self.danger_active = True
        elif self.danger_active:
            # 장애물이 사라졌을 경우 기본 전진
            twist.linear.x = 0.1  # 즉시 전진
            self.cmd_vel_pub.publish(twist)
            self.danger_active = False

        # 디버그 이미지 publish
        img_msg = self.bridge.cv2_to_imgmsg(draw_img, encoding='bgr8')
        self.image_pub.publish(img_msg)

    # 파란색 장애물(사람): 가까워지면 정지, 사라지거나 멀어지면 전진
    def process_blue_obstacle(self, img, draw_img, image_area):
        danger_b = False
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 100, 50])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            # 너무 작은 객체는 무시
            if area < 0.02 * image_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
            # 꼭지점이 너무 적으면 원형으로 인식하지 않음
            if len(approx) < 6:
                continue  # 원이 아님

            cx = x + w // 2
            cv2.rectangle(draw_img, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(draw_img, "Person", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # 이전 프레임 대비 객체 크기 비교
            if self.prev_circle_area_b is not None:
                # 가까워짐 -> 정지
                if area > self.prev_circle_area_b * 1.2:
                    self.current_speed = 0.0
                # 멀어짐 -> 전진
                elif area < self.prev_circle_area_b * 0.8:
                    self.current_speed = 0.1
                # 변환없음 -> 정지 유지
                else:
                    self.current_speed = 0.0
            # 최초 감지 시 정지
            else:
                self.current_speed = 0.0

            self.prev_circle_area_b = area
            danger_b = True
            break
        return danger_b

    # 초록색 장애물(차): 가까워지면 후진, 멀어지면 전진
    def process_green_obstacle(self, img, draw_img, image_area):
        danger_g = False
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40, 100, 50])
        upper_green = np.array([80, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            # 너무 작은 객체는 무시
            if area < 0.02 * image_area:
                continue

            x, y, w, h = cv2.boundingRect(cnt)
            peri = cv2.arcLength(cnt, True)
            circularity = 4 * np.pi * area / (peri ** 2) if peri != 0 else 0
            # 꼭지점이 너무 적으면 원형으로 인식하지 않음
            if circularity < 0.6:
                continue

            cx = x + w // 2
            cv2.rectangle(draw_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(draw_img, "Car", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 이전 프레임 대비 객체 크기 비교
            if self.prev_circle_area_g is not None:
                # 가까워짐 -> 정지
                if area > self.prev_circle_area_g * 1.2:
                    # self.current_speed = -0.7     # 후진 x
                    self.current_speed = 0.0  
                # 멀어짐 -> 전진
                elif area < self.prev_circle_area_g * 0.8:
                    self.current_speed = 0.1
                # 변화없음 -> 정지 유지
                else:
                    self.current_speed = 0.0
            # 최초 감지 시 정지
            else:
                self.current_speed = 0.0

            self.prev_circle_area_g = area
            danger_g = True
            break
        return danger_g

    # 빨간색 방지터: 감지 시, 직진
    def process_red_speedbump(self, img, draw_img, image_area):
        danger_r = False
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            # 카메라에서 방지턱의 크기가 클 때만
            if area > 0.05 * image_area:
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(draw_img, (x, y), (x + w, y + h), (0, 0, 255), 2)
                cv2.putText(draw_img, "Speed_Bump", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                
                twist = Twist()
                twist.linear.x = 0.05   # 천천히 전진 
                twist.angular.z = 0.0   # 회전 금지
                self.cmd_vel_pub.publish(twist)

                self.current_speed = 0.05 # 상태 저장용
                danger_r = True
                break
        return danger_r

def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()