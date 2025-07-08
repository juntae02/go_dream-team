import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/line_centers',
            self.listener_callback,
            10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 화면 폭 (수정 필요) 예: 640x480 카메라라면 640
        self.image_width = 640

    def listener_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().warn('Invalid data received')
            return

        y_cx, w_cx = msg.data
        if y_cx == -1 or w_cx == -1:
            self.get_logger().warn('One or both lines not detected')
            return

        target_cx = (y_cx + w_cx) // 2
        center_x = self.image_width // 2
        error_x = center_x - target_cx  # 왼쪽으로 치우치면 양수

        # Pure Pursuit 스타일 회전 제어
        # (여기서는 간소화된 비례제어 적용)
        twist = Twist()
        twist.linear.x = 0.1  # 전진 속도
        twist.angular.z = 0.003 * error_x  # 회전 속도 (보정 계수 조절)

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
