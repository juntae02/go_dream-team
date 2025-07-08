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

        self.image_width = 640  # 실제 카메라 해상도에 맞게 설정
        self.lookahead_gain = 0.005  # 회전 민감도 조절 계수
        self.linear_speed = 0.1  # 기본 전진 속도

    def listener_callback(self, msg):
        if len(msg.data) < 2:
            return

        y_cx, w_cx = msg.data
        if y_cx == -1 or w_cx == -1:
            # 한 쪽 선이라도 인식 못하면 정지
            self.stop_robot()
            return

        target_cx = (y_cx + w_cx) // 2
        error = (self.image_width // 2) - target_cx

        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.lookahead_gain * error

        self.cmd_pub.publish(twist)
        self.get_logger().info(f"Target: {target_cx}, Error: {error}, Angular Z: {twist.angular.z:.3f}")

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
