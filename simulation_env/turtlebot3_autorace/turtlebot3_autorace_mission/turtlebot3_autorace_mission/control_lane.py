# Author : 지예은
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String
import math

class ControlLane(Node):

    def __init__(self):
        super().__init__('control_lane')

        self.sub_lane = self.create_subscription(
            Float64,
            '/control/lane',
            self.callback_follow_lane,
            1
        )
        self.sub_max_vel = self.create_subscription(
            Float64,
            '/control/max_vel',
            self.callback_get_max_vel,
            1
        )
        self.sub_avoid_cmd = self.create_subscription(
            Twist,
            '/avoid_control',
            self.callback_avoid_cmd,
            1
        )
        self.sub_avoid_active = self.create_subscription(
            Bool,
            '/avoid_active',
            self.callback_avoid_active,
            1
        )
        self.sub_obstacle_action = self.create_subscription(
            String,
            '/obstacle_action',
            self.callback_obstacle_action,
            1
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/control/cmd_vel',
            1
        )

        # PD control 관련 변수
        self.last_error = 0
        self.last_angular_z = 0.0
        self.MAX_VEL = 0.1

        # Avoidance mode 관련 변수
        self.avoid_active = False
        self.avoid_twist = Twist()

        self.obstacle_action = "none"  # none/left/right/stop

    def callback_get_max_vel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data

    def callback_obstacle_action(self, msg):
        self.obstacle_action = msg.data
        twist = Twist()
        if self.obstacle_action == "left":
            twist.linear.x = 0.07
            twist.angular.z = 0.4
            self.pub_cmd_vel.publish(twist)
        elif self.obstacle_action == "right":
            twist.linear.x = 0.07
            twist.angular.z = -0.4
            self.pub_cmd_vel.publish(twist)
        elif self.obstacle_action == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub_cmd_vel.publish(twist)
        # "none"이면 lane 따라가기

    def callback_follow_lane(self, desired_center):
        # 장애물 행동/회피 중이면 차선 무시
        if self.obstacle_action != "none":
            return
        if self.avoid_active:
            return

        center = desired_center.data

        # ====== [추가] 차선 인식 실패 시 저속 직진 ======
        if (center is None or
            math.isnan(center) or
            center < 0 or center > 1000):
            twist = Twist()
            twist.linear.x = 0.02   # 아주 천천히 직진
            twist.angular.z = 0.0
            self.pub_cmd_vel.publish(twist)
            return

        error = center - 500
        Kp = 0.0025
        Kd = 0.007
        angular_z = Kp * error + Kd * (error - self.last_error)
        self.last_error = error

        # ====== [추가] Steer 변화 제한 ======
        max_delta_steer = 0.2  # 각속도 변화 한계
        delta = angular_z - self.last_angular_z
        if abs(delta) > max_delta_steer:
            angular_z = self.last_angular_z + max_delta_steer * (1 if delta > 0 else -1)
        self.last_angular_z = angular_z

        twist = Twist()
        # Linear velocity: adjust speed based on error (최대 0.05 제한)
        twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 500, 0) ** 2.2), 0.05)
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        self.pub_cmd_vel.publish(twist)

    def callback_avoid_cmd(self, twist_msg):
        self.avoid_twist = twist_msg
        if self.avoid_active:
            self.pub_cmd_vel.publish(self.avoid_twist)

    def callback_avoid_active(self, bool_msg):
        self.avoid_active = bool_msg.data
        if self.avoid_active:
            self.get_logger().info('Avoidance mode activated.')
        else:
            self.get_logger().info('Avoidance mode deactivated. Returning to lane following.')

    def shut_down(self):
        self.get_logger().info('Shutting down. cmd_vel will be 0')
        twist = Twist()
        self.pub_cmd_vel.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()