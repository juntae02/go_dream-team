import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist

class PIDLineFollower(Node):
    def __init__(self):
        super().__init__('pid_line_follower')

        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/line_centers',
            self.line_callback,
            10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 카메라 해상도 가로폭 (오차 기준값)
        # self.image_width = 640
        self.image_width = 320
        self.center_x = self.image_width // 2

        # PID 파라미터
        self.Kp = 0.005
        self.Ki = 0.0001
        self.Kd = 0.002

        self.error_sum = 0.0
        self.last_error = 0.0

        # 속도 설정
        self.base_speed = 0.15
        self.min_speed = 0.1

    def line_callback(self, msg):
        if len(msg.data) != 6:
            return

        y_vals = msg.data[:3]
        w_vals = msg.data[3:]

        roi_weights = [0.5, 0.3, 0.2]  # near > mid > far
        errors = []
        weights = []

        for i in range(3):
            y = y_vals[i]
            w = w_vals[i]

            if y != -1 and w != -1:
                cx = (y + w) // 2
            elif y != -1:
                cx = y
            elif w != -1:
                cx = w
            else:
                continue

            error = self.center_x - cx
            errors.append(error)
            weights.append(roi_weights[i])

        if not errors:
            self.stop_robot()
            return

        # 가중 평균 에러 계산
        total_weight = sum(weights)
        weighted_error = sum(e * w for e, w in zip(errors, weights)) / total_weight

        angular_z = self.calculate_pid_control(weighted_error)
        linear_x = self.adjust_speed(weighted_error, angular_z)

        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_pub.publish(twist)

        self.get_logger().info(f"Error: {weighted_error:.1f}, angular.z: {angular_z:.3f}, linear.x: {linear_x:.2f}")

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

    def stop_robot(self):
        twist = Twist()
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PIDLineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
