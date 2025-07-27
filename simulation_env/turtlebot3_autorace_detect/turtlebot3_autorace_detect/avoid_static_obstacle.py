import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObstaclePub(Node):
    def __init__(self):
        super().__init__('obstacle_pub')
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.img_callback, 10)
        self.image_pub = self.create_publisher(Image, '/obstacle_image', 10)
        self.action_pub = self.create_publisher(String, '/obstacle_action', 10)
        self.bridge = CvBridge()
        # === NEW: 마지막 action과 장애물 cx 저장 ===
        self.last_action = "none"
        self.last_cx = None

    def img_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        draw_img = img.copy()

        # ROI: 이미지 상단 2/3만 사용 (아래쪽 1/3은 무시)
        img_h, img_w = img.shape[:2]
        roi_limit = int(img_h * (2/3))
        roi_img = img[:roi_limit, :]

        # === 노란색을 미리 검정색(0)으로 만듦 ===
        hsv = cv2.cvtColor(roi_img, cv2.COLOR_BGR2HSV)
        yellow_mask = cv2.inRange(hsv, (15, 50, 50), (35, 255, 255))  # 노란색 범위
        roi_img_no_yellow = roi_img.copy()
        roi_img_no_yellow[yellow_mask > 0] = 0  # 노란색 부분을 검정색으로

        # === 이후 이진화/컨투어 검출은 노란색 제외된 ROI에서! ===
        gray = cv2.cvtColor(roi_img_no_yellow, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
        kernel = np.ones((5, 15), np.uint8)
        thresh = cv2.erode(thresh, kernel, iterations=1)

        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        image_area = img.shape[0] * img.shape[1]
        action = "none"
        min_area_ratio = 0.13  # 전체 이미지의 13% 이상이면 장애물(튜닝!)
        biggest = None

        for cnt in sorted(contours, key=cv2.contourArea, reverse=True):
            x, y, w, h = cv2.boundingRect(cnt)
            area = w * h
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
            aspect = w / h if h != 0 else 0
            if aspect < 0.3 or aspect > 3.0:
                continue
            if area < min_area_ratio * image_area:
                continue
            roi = gray[y:y+h, x:x+w]
            mean_brightness = np.mean(roi)
            if mean_brightness < 180:
                continue

            biggest = (x, y, w, h)
            break

        # ========== 하단 1/3 ROI에서 차선 검출 ==========
        bot_roi = img[roi_limit:, :]
        hsv_bot = cv2.cvtColor(bot_roi, cv2.COLOR_BGR2HSV)
        mask_white = cv2.inRange(hsv_bot, np.array([0, 0, 180]), np.array([180, 50, 255]))
        mask_yellow = cv2.inRange(hsv_bot, np.array([15, 60, 100]), np.array([35, 255, 255]))
        sum_white = np.sum(mask_white)
        sum_yellow = np.sum(mask_yellow)
        has_white_line = sum_white > 5000
        has_yellow_line = sum_yellow > 5000

        # === debug print ===
        print(f"mask_yellow sum: {sum_yellow} | mask_white sum: {sum_white}")
        print(f"has_white_line: {has_white_line}, has_yellow_line: {has_yellow_line}")

        # ====== 장애물 행동 판단 ======
        cx = None
        if biggest is not None:
            x, y, w, h = biggest
            cx = x + w // 2  # 중심 x좌표 (ROI기준)
            cv2.rectangle(draw_img, (x, y), (x+w, y+h), (0, 0, 255), 2)
            cv2.putText(draw_img, "Obstacle!", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

            third = img_w // 3
            margin = int(third * 1.2)  # ← margin을 1.2배로 넓힘

            # === 여기 로그 추가 ===
            print(f"biggest cx: {cx} | margin: {margin} | img_w: {img_w}")

            # 1. 왼쪽 장애물 → 오른쪽
            if cx < margin:
                action = "right"
            # 2. 오른쪽 장애물 → 왼쪽
            elif cx > img_w - margin:
                action = "left"
            # 3. 진짜 중앙이면 mask sum으로 회피 방향(임계치 옵션 적용!)
            else:
                mask_threshold = 10000  # (옵션) mask sum 임계값, 필요에 따라 조정
                if sum_white < mask_threshold and sum_yellow < mask_threshold:
                    action = "none"
                elif sum_white > sum_yellow:
                    action = "left"
                else:
                    action = "right"
        
        else:
    # 장애물이 없으면 바로 action 해제
            action = "none"


        # === NEW: action 유지 조건(중앙에서 벗어나면 action 해제) ===
        if self.last_action in ("left", "right"):
            # 장애물 중심 cx가 아예 안 잡히거나, 중앙에서 threshold만큼 멀어지면 해제
            threshold_dist = int(img_w * 0.23)  # 조절 가능, 23%로 예시
            # 직전 action이 유지 중이고, cx가 있을 때
            if (cx is not None) and (abs(cx - img_w // 2) > threshold_dist):
                print(f"Action '{self.last_action}' 해제: cx={cx}, img_w/2={img_w//2}, threshold={threshold_dist}")
                action = "none"
        # ===

        self.last_action = action
        self.last_cx = cx

        # 파란색 박스(전체 이미지 기준)
        hsv_full = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 100, 50])
        upper_blue = np.array([140, 255, 255])
        blue_mask = cv2.inRange(hsv_full, lower_blue, upper_blue)
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in blue_contours:
            x, y, w, h = cv2.boundingRect(cnt)
            area = w * h
            if area > 0.10 * image_area:
                cv2.rectangle(draw_img, (x, y), (x+w, y+h), (255, 0, 0), 2)
                cv2.putText(draw_img, "Blue Obstacle!", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        self.action_pub.publish(String(data=action))
        img_msg = self.bridge.cv2_to_imgmsg(draw_img, encoding='bgr8')
        self.image_pub.publish(img_msg)

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

