import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32, Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Image

import tkinter as tk
from PIL import Image as PILImage, ImageTk
import cv2
import numpy as np
import math
import threading
import os

class GuiNode(Node):
    def __init__(self, gui_callback_image):
        super().__init__('gui_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/gui/cmd_vel', 10)
        self.max_vel_pub = self.create_publisher(Float64, '/control/max_vel', 10)
        self.auto_mode_pub = self.create_publisher(Bool, '/gui/auto_mode_active', 10)

        self.latest_linear = 0.0
        self.latest_angular = 0.0
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        self.gui_callback_image = gui_callback_image

    def publish_cmd_vel(self, linear, angular):
        self.latest_linear = round(linear, 3)
        self.latest_angular = round(angular, 3)
        self.get_logger().info(f'Set cmd_vel: linear={self.latest_linear:.3f}, angular={self.latest_angular:.3f}')

    def publish_cmd_vel_now(self):
        msg = Twist()
        msg.linear.x = round(self.latest_linear, 3)
        msg.angular.z = round(self.latest_angular, 3)
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info(f'Immediate publish: linear={msg.linear.x:.3f}, angular={msg.angular.z:.3f}')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = round(self.latest_linear, 3)
        msg.angular.z = round(self.latest_angular, 3)
        self.cmd_vel_pub.publish(msg)

    def publish_max_vel(self, value):
        msg = Float64()
        msg.data = round(value, 3)
        self.max_vel_pub.publish(msg)
        self.get_logger().info(f'Published max_vel: {msg.data:.3f}')

    def publish_auto_mode(self, active):
        msg = Bool()
        msg.data = active
        self.auto_mode_pub.publish(msg)
        self.get_logger().info(f'Published auto_mode_active: {msg.data}')

    def image_callback(self, msg):
        try:
            img = np.ndarray(shape=(msg.height, msg.width, 3), dtype=np.uint8, buffer=msg.data)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(img)
            if self.gui_callback_image:
                self.gui_callback_image(pil_image)
        except Exception as e:
            self.get_logger().error(f"이미지 처리 실패: {e}")

class SteeringHandle:
    def __init__(self, parent, on_angle_change):
        self.canvas = tk.Canvas(parent, width=200, height=200, bg='white', highlightthickness=0)
        self.canvas.pack(pady=10)
        self.center = (100, 100)
        self.radius = 80
        self.angle = 0.0
        self.on_angle_change = on_angle_change

        script_dir = os.path.dirname(os.path.abspath(__file__))
        resource_path = os.path.join(script_dir, 'handle.png')

        self.handle_img_orig = PILImage.open(resource_path).resize((160, 160))
        self.handle_img = ImageTk.PhotoImage(self.handle_img_orig)
        self.handle_image_id = self.canvas.create_image(self.center[0], self.center[1], image=self.handle_img)

        self.canvas.bind("<B1-Motion>", self.on_drag)

    def on_drag(self, event):
        dx = event.x - self.center[0]
        dy = event.y - self.center[1]

        angle_rad = math.atan2(-dy, dx) - math.pi / 2

        if angle_rad > math.pi:
            angle_rad -= 2 * math.pi
        elif angle_rad < -math.pi:
            angle_rad += 2 * math.pi

        self.angle = angle_rad

        rotated_img = self.handle_img_orig.rotate(math.degrees(angle_rad), resample=PILImage.BICUBIC)
        self.handle_img = ImageTk.PhotoImage(rotated_img)
        self.canvas.itemconfig(self.handle_image_id, image=self.handle_img)

        if self.on_angle_change:
            self.on_angle_change(angle_rad)

class GUI:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("ROS 2 Control GUI")
        self.root.geometry("800x1000")

        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.auto_mode = False

        self.speed_label = tk.Label(self.root, text="선속도: 0.0 m/s", font=("Arial", 14))
        self.speed_label.pack(pady=10)

        accel_brake_frame = tk.Frame(self.root)
        accel_brake_frame.pack(pady=5)

        self.accel_button = tk.Button(accel_brake_frame, text="엑셀 ↑", width=10, command=self.on_accel)
        self.accel_button.grid(row=0, column=0, padx=10)

        self.brake_button = tk.Button(accel_brake_frame, text="브레이크 ↓", width=10, command=self.on_brake)
        self.brake_button.grid(row=0, column=1, padx=10)

        self.angular_label = tk.Label(self.root, text="각속도: 0.0 rad/s", font=("Arial", 14))
        self.angular_label.pack(pady=10)

        self.steering = SteeringHandle(self.root, self.on_handle_angle_change)

        self.max_vel_scale = tk.Scale(self.root, from_=0, to=5, resolution=0.1,
                                      orient="horizontal", label="최고 속도")
        self.max_vel_scale.set(1.0)
        self.max_vel_scale.pack()

        self.max_vel_button = tk.Button(self.root, text="최고 속도 설정 퍼블리시", command=self.on_publish_max_vel)
        self.max_vel_button.pack(pady=10)

        self.auto_mode_button = tk.Button(self.root, text="자동 모드 OFF", bg="lightgray", command=self.toggle_auto_mode)
        self.auto_mode_button.pack(pady=10)

        self.stop_button = tk.Button(self.root, text="정지 ⏹", fg="red", width=20, command=self.on_stop)
        self.stop_button.pack(pady=10)

        self.image_label = tk.Label(self.root)
        self.image_label.pack()

        self.ros_node = None

    def set_ros_node(self, node):
        self.ros_node = node

    def update_speed_label(self):
        self.speed_label.config(text=f"선속도: {self.current_linear_speed:.3f} m/s")

    def update_angular_label(self):
        self.angular_label.config(text=f"각속도: {self.current_angular_speed:.3f} rad/s")

    def on_accel(self):
        self.current_linear_speed = min(self.current_linear_speed + 0.1, 2.0)
        self.update_speed_label()
        self.publish_cmd()

    def on_brake(self):
        self.current_linear_speed = max(self.current_linear_speed - 0.1, -2.0)
        self.update_speed_label()
        self.publish_cmd()

    def on_handle_angle_change(self, angle_rad):
        self.current_angular_speed = max(min(angle_rad, 3.14), -3.14)
        self.update_angular_label()
        self.publish_cmd()

    def on_stop(self):
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.update_speed_label()
        self.update_angular_label()
        self.publish_cmd()

    def publish_cmd(self):
        if self.ros_node:
            self.ros_node.publish_cmd_vel(self.current_linear_speed, self.current_angular_speed)
            self.ros_node.publish_cmd_vel_now()

    def on_publish_max_vel(self):
        if self.ros_node:
            value = self.max_vel_scale.get()
            self.ros_node.publish_max_vel(value)

    def toggle_auto_mode(self):
        self.auto_mode = not self.auto_mode
        state_text = "자동 모드 ON" if self.auto_mode else "자동 모드 OFF"
        color = "lightgreen" if self.auto_mode else "lightgray"
        self.auto_mode_button.config(text=state_text, bg=color)
        if self.ros_node:
            self.ros_node.publish_auto_mode(self.auto_mode)

    def update_image(self, image):
        tk_img = ImageTk.PhotoImage(image)
        self.image_label.after(0, lambda: self.image_label.config(image=tk_img))
        self.image_label.image = tk_img

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)

    gui = GUI()
    node = GuiNode(gui.update_image)
    gui.set_ros_node(node)

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    gui.run()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
