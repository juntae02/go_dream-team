#!/usr/bin/env python3
#
# Copyright 2018 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors:
#   - Leon Jung, Gilbert, Ashe Kim, Hyungyu Kim, ChanHyeong Lee
#   - Special Thanks : Roger Sacchelli

import os
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


from scipy.interpolate import splrep, splev
import random
import time

class DetectLane(Node):

    def __init__(self):
        super().__init__('detect_lane')
        self.prev_time = 0
        self.fps = 10

        self.cvBridge = CvBridge()

        # self.Kp = 0.005
        self.Kp = 0.002
        self.Ki = 0.0001
        self.Kd = 0.001

        self.error_sum = 0.0
        self.last_error = 0.0
        self.base_speed = 0.05
        self.min_speed = 0.01

        self.sub_image_type = 'raw'         # you can choose image type 'compressed', 'raw'
        self.pub_image_type = 'compressed'  # you can choose image type 'compressed', 'raw'

        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage, '/camera/image_raw/compressed', self.image_callback, 1
                )
        elif self.sub_image_type == 'raw':
            self.sub_image_original = self.create_subscription(
                Image, '/camera/image_raw', self.image_callback, 1
                )

        if self.pub_image_type == 'compressed':
            self.pub_image_lane = self.create_publisher(
                CompressedImage, '/detect/image_output/compressed', 1
                )
        elif self.pub_image_type == 'raw':
            self.pub_image_lane = self.create_publisher(
                Image, '/detect/image_output', 1
                )

        self.processed_image_pub = self.create_publisher(Image, 'processed_image', 10)

        self.cmd_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info('Lane Detection Node initialized')


    def image_callback(self, msg):
        """Process incoming image messages"""

        current_time = time.time()
        if current_time - self.prev_time < 1.0 / self.fps:
            # 간격이 아직 안 됐으면 처리 안 함 (프레임 스킵)
            return
        self.prev_time = current_time
        
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.cvBridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process the image
            final_center = self.process_image(cv_image)
            
            # Calculate
            if final_center is not None:
                image_center = self.width // 2
                error = image_center - final_center
                angular_z = self.calculate_pid_control(error)
                linear_x = self.adjust_speed(error, angular_z)

                cmd = Twist()
                cmd.linear.x = linear_x
                cmd.angular.z = angular_z
                self.cmd_publisher.publish(cmd)


            # Publish processed image
            processed_msg = self.cvBridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.processed_image_pub.publish(processed_msg)
            
            # Display image (optional - comment out for headless operation)
            cv2.imshow('Lane Detection', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


    def draw_lines(self, img, lines):
        """Draw detected lines on image"""
        for line in lines:
            x1, y1, x2, y2 = line[0]
            color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
            img = cv2.line(img, (x1, y1 + self.offset), (x2, y2 + self.offset), color, 2)
        return img


    def draw_rectangle(self, img, lpos, rpos, offset=0):
        """Draw position indicators on image"""
        center = int((lpos + rpos) / 2)
        msg_desired_center = Float64()
        msg_desired_center.data = (lpos + rpos) / 2
        self.pub_lane.publish(msg_desired_center)
        
        # Left position indicator (green)
        cv2.rectangle(img, (lpos - 5, 15 + offset), (lpos + 5, 25 + offset), (0, 255, 0), 2)
        # Right position indicator (green)
        cv2.rectangle(img, (rpos - 5, 15 + offset), (rpos + 5, 25 + offset), (0, 255, 0), 2)
        # Center of lanes (green)
        cv2.rectangle(img, (center - 5, 15 + offset), (center + 5, 25 + offset), (0, 255, 0), 2)
        # Image center reference (red)
        # cv2.rectangle(img, (315, 15 + offset), (325, 25 + offset), (0, 0, 255), 2)
        cv2.rectangle(img, (center, 15 + offset), (center, 25 + offset), (0, 0, 255), 2)

        return img

    def get_line_params(self, lines):
        """Calculate average slope and intercept of lines"""
        x_sum = 0.0
        y_sum = 0.0
        m_sum = 0.0

        size = len(lines)
        if size == 0:
            return 0, 0

        for line in lines:
            x1, y1, x2, y2 = line[0]
            x_sum += x1 + x2
            y_sum += y1 + y2
            m_sum += float(y2 - y1) / float(x2 - x1)

        x_avg = x_sum / (size * 2)
        y_avg = y_sum / (size * 2)
        m = m_sum / size
        b = y_avg - m * x_avg

        return m, b


    def get_line_pos(self, img, lines, left=False, right=False):
        """Get lane position from detected lines"""
        m, b = self.get_line_params(lines)

        if m == 0 and b == 0:
            if left:
                pos = 0
            if right:
                pos = self.width
        else:
            y = self.gap / 2
            pos = (y - b) / m

            b += self.offset
            x1 = (self.height - b) / float(m)
            x2 = ((self.height/2) - b) / float(m)

            cv2.line(img, (int(x1), self.height), (int(x2), (self.height//2)), (255, 0, 0), 3)

        return img, int(pos)

    def get_stable_center_from_contours(self, child_contours):
        num_samples = 6
        step = self.height // num_samples

        mid_x = self.width // 2
        y_min = step * 2
        # y_max = self.height - step * 1
        y_max = self.height


        left_xs = []
        right_xs = []
        for contour in child_contours:
            for pt in contour:
                x, y = pt[0]
                if y_min <= y <= y_max:
                    if x < mid_x:
                        left_xs.append(x)
                    else:
                        right_xs.append(x)

        left_xs_filtered = self.remove_outliers_mad(left_xs,2.0)
        right_xs_filtered = self.remove_outliers_mad(right_xs,2.0)

        lpos = int(np.mean(left_xs_filtered)) if left_xs_filtered else int(self.width * 0.2)
        rpos = int(np.mean(right_xs_filtered)) if right_xs_filtered else int(self.width * 0.8)
        center_now = (lpos + rpos) // 2

        points = []

        for i in range(4):
            roi_y = step * i
            temp_left, temp_right = [], []    

            for contour in child_contours:
                for pt in contour:
                    x, y = pt[0]
                    if roi_y <= y < roi_y + step:
                        if x < mid_x:
                            temp_left.append(x)
                        else:
                            temp_right.append(x)

            temp_left = self.remove_outliers_mad(temp_left,2.0)
            temp_right = self.remove_outliers_mad(temp_right,2.0)

            left_mean = np.median(temp_left) if temp_left else None
            right_mean = np.median(temp_right) if temp_right else None

            if (left_mean is not None) and (right_mean is not None):
                cx = (left_mean + right_mean) // 2
                points.append((cx, roi_y + 2))
         
        
        if len(points) >= 3:
            pts = np.array(points, dtype=np.float32)
            vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
           
            if abs(vy) > 1e-5:
                future_y = 60
                future_x = int(((future_y - y0) * vx / vy) + x0)
            else:
                future_x = center_now
           
            line_length = 100
            pt1 = (int(x0 - vx * line_length), int(y0 - vy * line_length))
            pt2 = (int(x0 + vx * line_length), int(y0 + vy * line_length))
            cv2.line(self.frame, pt1, pt2, (255, 0, 0), 2)
        else:
            future_x = center_now

        ratio = 0.8
        stable_center = int(ratio * center_now + (1-ratio) * future_x)
        # stable_center = center_now
        alpha = 0.3
        if not hasattr(self, 'prev_center'):
            self.prev_center = stable_center
        final_center = int(alpha * self.prev_center + (1 - alpha) * stable_center)
        self.prev_center = final_center
    
        return final_center, lpos, rpos

    def remove_outliers_mad(self, data, thresh=2.5):
        data = np.array(data)
        if len(data) < 4:
            return data.tolist()
        median = np.median(data)
        deviations = np.abs(data - median)
        mad = np.median(deviations)
        if mad == 0:
            return data.tolist()
        z_scores = 0.6745 * (data - median) / mad
        return data[np.abs(z_scores) < thresh].tolist()


    def process_image(self, frame):
        self.frame = frame
        self.height = frame.shape[0]
        self.width = frame.shape[1]

        start = int(0.07 * self.width)
        end = int(0.93 * self.width)

        # roi = frame[100:240, :]
        roi = self.frame

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        kernel_size = 7
        blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

        low_threshold = 130
        high_threshold = 170

        edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

        _, binary = cv2.threshold(edge_img, 200, 255, cv2.THRESH_BINARY_INV)

        contours, hierarchy = cv2.findContours(binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)

        child_contours = []
        for i,h in enumerate(hierarchy[0]):
            if h[3] != -1:
                child_contours.append(contours[i])
                cv2.drawContours(binary, contours, i , (0,255,0),2)

        cv2.imshow('binary2',binary)

        final_center, lpos, rpos= self.get_stable_center_from_contours(child_contours)
        
        cv2.rectangle(frame, (lpos - 5, 215), (lpos + 5, 225), (0, 255, 0), 2)
        cv2.rectangle(frame, (rpos - 5, 215), (rpos + 5, 225), (0, 255, 0), 2)
        cv2.rectangle(frame, (final_center - 5, 215), (final_center + 5, 225), (0, 255, 0), 2)
        cv2.rectangle(frame, (final_center, 215), (final_center, 225), (0, 0, 255), 2)  

        processed_msg = self.cvBridge.cv2_to_imgmsg(frame, "bgr8")
        self.processed_image_pub.publish(processed_msg)
        return final_center
    
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


    def destroy_node(self):
        """Clean up resources"""
        if hasattr(self, 'cap'):
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
