import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import time

class HoughLaneVisualizer(Node):
    def __init__(self):
        super().__init__('hough_lane_visualizer')
        self.bridge = CvBridge()    # ROS 이미지 -> OpenCV 변환기
        self.width = 320  # 이미지 가로 해상도

        self.crosswalk_detected = False
        self.crosswalk_detected_time = 0.0 
        self.last_linear_x = 0.0
        self.last_angular_z = 0.0

        # PID 제어용 파라미터
        self.Kp = 0.005
        self.Ki = 0.0001
        self.Kd = 0.001

        self.error_sum = 0.0
        self.last_error = 0.0
        self.base_speed = 0.07
        self.min_speed = 0.05

        # 이미지 토픽 구독자 생성
        self.sub_image_type = 'raw'         
        self.pub_image_type = 'compressed' 

        if self.sub_image_type == 'compressed':
            self.image_sub = self.create_subscription(
                CompressedImage, '/camera/image_raw/compressed', self.image_callback, 10
                )
        elif self.sub_image_type == 'raw':
            self.image_sub = self.create_subscription(
                Image, '/camera/image_raw', self.image_callback, 10
                )
            
        # 속도 명령 퍼블리셔 (모터 제어용)
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_publisher = self.create_publisher(Twist, 'lane/cmd_vel', 10)

    
    # 콜백 함수
    def image_callback(self, msg):
        """
        카메라 이미지가 들어올 때마다 호출되는 콜백 함수.
        허프 변환을 통해 차선을 인식하고 화면에 중심선을 시각화함.
        """
        try:
            # ROS 이미지 메시지를 OpenCV BGR 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")   
           
            lane_center, result = self.detect_lane(cv_image)
            self.detect_crosswalk(cv_image)


            if lane_center is not None:
                image_center = self.width // 2
                error = image_center - lane_center
                angular_z = self.calculate_pid_control(error)
                linear_x = self.adjust_speed(error, angular_z)

                cmd = Twist()
                cmd.linear.x = linear_x
                cmd.angular.z = angular_z
                # time.sleep(1.0)
                self.cmd_publisher.publish(cmd)
                self.last_linear_x = linear_x
                self.last_angular_z = angular_z

        except Exception as e:
            self.get_logger().error(f"Processing error: {e}")
            return

        # 차선 검출 처리
        if result is not None:
            # 빨간선으로 표시
            if lane_center is not None:
                cv2.line(result, (lane_center, result.shape[0]), (lane_center, int(result.shape[0] * 0.5)), (0, 0, 255), 2) 
            # 결과 이미지 출력
            cv2.imshow("Hough Lane Detection", result)
            cv2.waitKey(1)

    # 차선을 감지하는 함수
    def detect_lane(self, cv_image):
        """
        Hough Transform을 적용하여 직선 차선을 감지하고,
        좌/우 차선의 중심을 기반으로 주행 중심선을 계산함.
        """
        height = cv_image.shape[0]
        width = cv_image.shape[1]
    
    # 1. ROI 설정
        roi_start_y = int(height * 0.01) 
        roi_end_y = height
        roi_start_x = 0 
        roi_end_x = int(width * 0.95)
        # 최종 ROI 적용
        roi = cv_image[roi_start_y:roi_end_y, roi_start_x:roi_end_x]

    # 2. 전처리
        # ROI를 그레이스케일로 변환
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)    # 차선을 색상이 아닌 밝기로 구별하기 위해

        # 가우시안 블러로 노이즈 제거
        blur = cv2.GaussianBlur(gray, (5, 5), 0)        # 노이즈의 강도를 줄여주는 역할
            # 노이즈는 약하고 작기 때문에 블러에 의해 쉽게 흐려지고 사라짐
            # 차선은 굵고 명확한 밝기 변화를 가지고 있어서, 감지됨

        # 모폴로지 열기 연산으로 잔여 노이즈 정제
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        opened = cv2.morphologyEx(blur, cv2.MORPH_OPEN, kernel) # 침식 -> 팽창
            # 노이즈의 형태를 제거하는 방식(작은 점, 얇은 선, 끊긴 점 등)
        
        # Canny 엣지 검출 (선의 윤곽 감지)
        edges = cv2.Canny(opened, 65, 200)  # 차선 하나에 엣지가 두 개 발생
        # edges = cv2.Canny(blur, 65, 200)  
        cv2.imshow("Canny Edges", edges)

    # 3. 허프 변환으로 직선 검출
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50,
                                minLineLength=25, maxLineGap=20)
        # 선들을 그릴 빈 이미지 생성(원본과 같은 크기)
        line_image = np.zeros_like(cv_image)

        # 왼쪽과 오른쪽 차선을 나눠 저장할 리스트
        left_lines = []
        right_lines = []

        # 직선이 검출되지 않으면 원본 반환
        if lines is None:
            return None, cv_image  

        # 원본 이미지의 중간 x좌표
        mid_x_original = width // 2  

    # 4. 검출된 선 필터링
        for line in lines:
            x1_roi, y1_roi, x2_roi, y2_roi = line[0]
            
            # ROI 좌표를 원본 이미지의 전역 좌표로 변환
            x1_orig = x1_roi + roi_start_x
            y1_orig = y1_roi + roi_start_y
            x2_orig = x2_roi + roi_start_x
            y2_orig = y2_roi + roi_start_y
            
            # 선 길이
            dx = x2_orig - x1_orig
            dy = y2_orig - y1_orig
            length = np.sqrt(dx**2 + dy**2)
            # 짧은 선 제거
            if length < 24:
                continue    

            # 기울기
            slope = dy / (dx + 1e-6)  # 0으로 나눔 방지   
            # 수평 또는 수직에 가까운 선은 제외
            if abs(slope) < 0.05 or abs(slope) > 112.0:
                continue

            

            # ROI 내 좌표로 다시 계산
            x1_rel = x1_orig - roi_start_x
            y1_rel = y1_orig - roi_start_y
            x2_rel = x2_orig - roi_start_x
            y2_rel = y2_orig - roi_start_y

            # 선 두께 측정
            thickness_ok = False
            num_thick_pixels = 0

            # 선 중심선 좌표 생성
            points = np.linspace((x1_rel, y1_rel), (x2_rel, y2_rel), 10).astype(int)
            for px, py in points:
                for offset in range(-2, 3):  # 수직 방향 ±2픽셀
                    ox = int(px - offset * slope / (np.sqrt(1 + slope**2) + 1e-6))
                    oy = int(py + offset / (np.sqrt(1 + slope**2) + 1e-6))

                    if 0 <= oy < edges.shape[0] and 0 <= ox < edges.shape[1]:
                        if edges[oy, ox] > 0:
                            num_thick_pixels += 1
            
            # 차선 두께 맞춤
            if num_thick_pixels <= 19 and num_thick_pixels >= 10:  
                thickness_ok = True
            if not thickness_ok:
                continue  

            # 선 근처 밝기 측정 (선 중간 10 지점을 샘플링)
            brightness_sample_points = np.linspace((x1_rel, y1_rel), (x2_rel, y2_rel), 10).astype(int)
            # 해당 지점의 밝기 평균
            brightness_values = [gray[min(p[1], gray.shape[0]-1), 
                                      min(p[0], gray.shape[1]-1)] 
                                 for p in brightness_sample_points]
            mean_brightness = np.mean(brightness_values)
            # 밝기가 낮으면 빛을 받은 검정으로 간주
            if mean_brightness < 75:
                continue

            # 좌/우 선을 구분
            if slope < 0 and x2_orig < mid_x_original + 50: # 왼쪽 차선 (음의 기울기, 중간보다 왼쪽) 
                left_lines.append((x1_orig, y1_orig, x2_orig, y2_orig))
            elif slope > 0 and x1_orig > mid_x_original - 50: # 오른쪽 차선 (양의 기울기, 중간보다 오른쪽)
                right_lines.append((x1_orig, y1_orig, x2_orig, y2_orig)) 

            # 검출된 라인 그리기
            cv2.line(line_image, (x1_orig, y1_orig), (x2_orig, y2_orig), (0, 255, 0), 2)  

        # 5. 중심선 계산
        cx_left = None
        cx_right = None
        if left_lines:
            cx_left = int(np.mean([(line[0] + line[2]) / 2 for line in left_lines])) 
        if right_lines:
            cx_right = int(np.mean([(line[0] + line[2]) / 2 for line in right_lines])) 

        lane_center = None

        # 좌/우 차선이 모두 감지되면 중앙 계산, 하나만 감지되면 보정
        if cx_left is not None and cx_right is not None:
            lane_center = int((cx_left + cx_right) / 2) # 좌우 평균
        elif cx_left is not None:
            lane_center = int(cx_left + (width / 4))  # 오른쪽이 안 보이면 오른쪽 방향으로 보정
        elif cx_right is not None:
            lane_center = int(cx_right - (width / 4))  # 왼쪽이 안 보이면 왼쪽 방향으로 보정
        else:
            return None, cv_image  # 아무 차선도 감지되지 않음
        # width * 0.1
        
        
        # 6. 최종 결과 이미지 새성 -> 선 합성
        # 원본 이미지와 차선 검출 결과 합성
        result = cv2.addWeighted(cv_image, 0.8, line_image, 1, 0)
        return lane_center, result

###
    def detect_crosswalk(self, cv_image):
        # 횡단보도 감지용 ROI
        height_c = cv_image.shape[0]
        width_c = cv_image.shape[1]
        cw_roi = cv_image[int(height_c * 0.01):height_c, 0 :int(width_c * 0.95)]
        # cw_roi = cv_image[int(height_c * 0.01):int(height_c * 0.70), int(width_c * 0.15):int(width_c * 0.8)]
        
        cw_image = np.zeros_like(cw_roi)

        # 현재 시간
        current_time = time.time()

        # HSV 흰색상 mask
        hsv = cv2.cvtColor(cw_roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 130])
        upper_white = np.array([180, 80, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # 횡단보도 감지
        cw_edges = cv2.Canny(mask_white, 50, 150)
        cw_lines = cv2.HoughLinesP(cw_edges, 1, np.pi / 180, threshold=50,
                                minLineLength=25, maxLineGap=20)

        vertical_lines = []
        if cw_lines is not None:
            for line in cw_lines:
                x1, y1, x2, y2 = line[0]
                if abs(x2 - x1) < 10 and abs(y2 - y1) > 20: # 수직 선 판단 + 짧은 선 방지
                    vertical_lines.append((x1, y1, x2, y2))
        
        for (x1, y1, x2, y2) in vertical_lines:
            cv2.line(cw_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        if len(vertical_lines) >= 4:
            if (not self.crosswalk_detected) or (current_time - self.crosswalk_detected_time > 15.0):
                self.get_logger().info("횡단보도 감지 → STOP")
                time.sleep(0.1)
                stop_cmd = Twist()
                stop_cmd.linear.x = 0.0
                stop_cmd.angular.z = 0.0
                self.cmd_publisher.publish(stop_cmd)
                time.sleep(1.0)
                self.crosswalk_detected = True

                resume_cmd = Twist()
                resume_cmd.linear.x = self.last_linear_x
                resume_cmd.angular.z = self.last_angular_z
                self.cmd_publisher.publish(resume_cmd)
                self.crosswalk_detected_time = current_time

        result = cv2.addWeighted(cw_roi, 0.8, cw_image, 1, 0)

        cv2.imshow("Crosswalk ROI", result)
        cv2.waitKey(1)

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
    node = HoughLaneVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()