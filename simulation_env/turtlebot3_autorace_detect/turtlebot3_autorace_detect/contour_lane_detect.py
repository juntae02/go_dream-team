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
from rcl_interfaces.msg import IntegerRange
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from std_msgs.msg import Float32
from std_msgs.msg import UInt8

import random
import math
import time

class DetectLane(Node):

    def __init__(self):
        super().__init__('detect_lane')

        self.width = 1000
        self.height = 600
        self.offset = 0
        self.gap = 600

        self.lane_gap = None
        self.GAP_THRESHOLD = 710

        self.cvBridge = CvBridge()

        self.sub_image_type = 'raw'         # you can choose image type 'compressed', 'raw'
        self.pub_image_type = 'compressed'  # you can choose image type 'compressed', 'raw'

        if self.sub_image_type == 'compressed':
            self.sub_image_original = self.create_subscription(
                CompressedImage, '/camera/image_projected/compressed', self.image_callback, 1
                )
        elif self.sub_image_type == 'raw':
            self.sub_image_original = self.create_subscription(
                Image, '/camera/image_projected', self.image_callback, 1
                )

        if self.pub_image_type == 'compressed':
            self.pub_image_lane = self.create_publisher(
                CompressedImage, '/detect/image_output/compressed', 1
                )
        elif self.pub_image_type == 'raw':
            self.pub_image_lane = self.create_publisher(
                Image, '/detect/image_output', 1
                )

        self.pub_lane = self.create_publisher(Float64, '/detect/lane', 1)
        self.pub_lane_state = self.create_publisher(UInt8, '/detect/lane_state', 1)


        # self.steer_angle_pub = self.create_publisher(Float32, 'steer_angle', 10)
        self.processed_image_pub = self.create_publisher(Image, 'processed_image', 10)

        self.get_logger().info('Lane Detection Node initialized')


    def image_callback(self, msg):
        """Process incoming image messages"""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.cvBridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process the image
            lpos, rpos = self.process_image(cv_image)
            
            # Calculate steering angle
            center = (lpos + rpos) / 2
            angle = int(self.width/2) - center  # 320 is half of 640 (image center)
            steer_angle = angle * 0.4
            
            # # Publish steering angle
            # steer_msg = Float32()
            # steer_msg.data = float(steer_angle)
            # self.steer_angle_pub.publish(steer_msg)
            
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
        width = self.width
        offset = self.offset
        gap = self.gap

        num_samples = 6
        step = self.gap // num_samples

        mid_x = width // 2
        y_min = offset + step * 2
        y_max = offset + gap - step * 1

        points = []

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

        left_xs_filtered = self.remove_outliers_mad(left_xs)
        right_xs_filtered = self.remove_outliers_mad(right_xs)

        lpos = int(np.mean(left_xs_filtered)) if left_xs_filtered else int(self.width * 0.2)
        rpos = int(np.mean(right_xs_filtered)) if right_xs_filtered else int(self.width * 0.8)
        center_now = (lpos + rpos) // 2

        points = []
        right_means = []

        for i in range(4):
            roi_y = self.offset + step * i
            temp_left, temp_right = [], []
            

            for contour in child_contours:
                for pt in contour:
                    x, y = pt[0]
                    if roi_y <= y < roi_y + step:
                        if x < mid_x:
                            temp_left.append(x)
                        else:
                            temp_right.append(x)

            temp_left = self.remove_outliers_mad(temp_left)
            temp_right = self.remove_outliers_mad(temp_right)

            left_mean = np.median(temp_left) if temp_left else None
            right_mean = np.median(temp_right) if temp_right else None

            if right_mean is not None:
                right_means.append(right_mean)

            if (left_mean is not None) and (right_mean is not None):
                cx = (left_mean + right_mean) // 2
                points.append((cx, roi_y + 2))
            # else:
            #     self.get_logger().info(f"line no detect at y = {roi_y} -> predict X")
        
        if len(points) >= 3:
            pts = np.array(points, dtype=np.float32)
            vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
            
            # if hasattr(self, 'prev_vx') and hasattr(self, 'prev_vy'):
            #     dot = vx * self.prev_vx + vy * self.prev_vy
            #     if dot < 0:
            #         vx, vy = -vx, -vy
            #         dot = -dot
            #     direction_change = np.arccos(np.clip(dot, -1.0, 1.0))
            #     if direction_change > np.radians(15):
            #         print("Too abrupt change -> keep previous line")
            #         vx = self.prev_vx
            #         vy = self.prev_vy
            #         x0 = self.prev_x0
            #         y0 = self.prev_y0
            #     else:
            #         self.prev_vx = vx
            #         self.prev_vy = vy
            #         self.prev_x0 = x0
            #         self.prev_y0 = y0
            # else:
            #     self.prev_vx = vx
            #     self.prev_vy = vy
            #     self.prev_x0 = x0
            #     self.prev_y0 = y0
           
            if abs(vy) > 1e-5:
                future_y = offset + 60
                future_x = int(((future_y - y0) * vx / vy) + x0)
            else:
                future_x = center_now
           
            line_length = 100
            pt1 = (int(x0 - vx * line_length), int(y0 - vy * line_length))
            pt2 = (int(x0 + vx * line_length), int(y0 + vy * line_length))
            cv2.line(self.frame, pt1, pt2, (255, 0, 0), 2)
        else:
            future_x = center_now


        self.lane_gap = abs(lpos - rpos)

        is_fake_split = self.lane_gap < 80
        is_real_split = self.lane_gap >= self.GAP_THRESHOLD
        
        if is_fake_split or is_real_split:
            log_msg = "Misdetected Y-split" if is_fake_split else "Real Y-junction"
            self.get_logger().info(f"{log_msg} — using right lane only")
            
            if len(right_means) >= 1:
                left_mean = mid_x + abs(mid_x - right_means[0])
                final_center = int((left_mean + right_means[0]) // 2)
                # left_mean2 = mid_x + abs(mid_x - right_means[0])
                # final_center = int((left_mean2 + right_means[0]) // 2)
                # final_center = int(right_means[0] - 100)
                print(1)
            # if right_mean is not None:
            #     # right_lane_center = (rpos + width) // 2
            #     # final_center = right_lane_center - mid_x
                
            #     # max_offset = width // 4
            #     # final_center = max(-max_offset, min(max_offset, final_center))
            #     # print(final_center)
            #     left_mean = mid_x + abs(mid_x - right_mean)
            #     final_center = (left_mean + right_mean) // 2
            else:
                final_center = int(center_now)
        else:
            ratio = 0.35
            stable_center = int(ratio * center_now + (1-ratio) * future_x)

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
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur
        kernel_size = 5
        blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

        # Apply Canny edge detection
        low_threshold = 60
        high_threshold = 70
        edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)

        # Define ROI and apply Hough transform
        roi = edge_img[self.offset : self.offset + self.gap, 0 : self.width]
        # cv2.imshow('roi',roi)

        
        _, binary = cv2.threshold(roi, 200, 255, cv2.THRESH_BINARY_INV)

        # cv2.imshow('binary',binary)
        y_min = self.offset
        y_max = self.offset + self.gap
        contours, hierarchy = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        
        child_contours = []
        for i,h in enumerate(hierarchy[0]):
            if h[3] != -1:
                child_contours.append(contours[i])
                cv2.drawContours(binary, contours, i , (0,255,0),2)
        
        # cv2.imshow('binary2',binary)

        debug_contour_img = frame.copy()
        cv2.drawContours(debug_contour_img, child_contours, -1, (0,255,255),2)
        # cv2.imshow('contour', debug_contour_img)

        final_center, lpos, rpos= self.get_stable_center_from_contours(child_contours)
        
        
        # width = frame.shape[1]
        # mid_x = width // 2
        # left_xs = []
        # right_xs = []
        # for contour in child_contours:
        #     for pt in contour:
        #         x, y = pt[0]
        #         if y_min <= y <= y_max:
        #             if x < mid_x:
        #                 left_xs.append(x)
        #             else:
        #                 right_xs.append(x)
        # left_xs_filtered = self.remove_outliers_mad(left_xs)
        # right_xs_filtered = self.remove_outliers_mad(right_xs)

        # lpos = int(np.mean(left_xs_filtered)) if left_xs_filtered else 0
        # rpos = int(np.mean(right_xs_filtered)) if right_xs_filtered else self.width
        # center = int((lpos + rpos) / 2)

        cv2.rectangle(frame, (lpos - 5, 215), (lpos + 5, 225), (0, 255, 0), 2)
        cv2.rectangle(frame, (rpos - 5, 215), (rpos + 5, 225), (0, 255, 0), 2)
        cv2.rectangle(frame, (final_center - 5, 215), (final_center + 5, 225), (0, 255, 0), 2)
        cv2.rectangle(frame, (final_center, 215), (final_center, 225), (0, 0, 255), 2)  

        angle = self.width - final_center
        steer_angle = angle * 0.4

        processed_msg = self.cvBridge.cv2_to_imgmsg(frame, "bgr8")
        self.processed_image_pub.publish(processed_msg)
        self.pub_lane.publish(Float64(data=float(final_center)))
        return lpos, rpos

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
