from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Int32
from std_msgs.msg import String

class ObstacleState(Enum):
    CLEAR = 0
    BLOCKED = 1
    CLEARED = 2

class Mode(Enum):
    MANUAL = 0
    AUTO = 1


class Server(Node):
    def __init__(self):
        super().__init__('server_node')

        self.mode = Mode.MANUAL
        self.obstacle = ObstacleState.CLEAR

        # 이전 상태 추적용 (중복 로그 방지)
        self._prev_mode = self.mode
        self._prev_obstacle = self.obstacle
        self._blocked_logged = False
        self._cleared_logged = False

        # 속도 메시지 저장
        self.gui_cmd = Twist()
        self.aruco_cmd = Twist()
        self.auto_cmd = Twist()

        # 타이머 관련
        self._cleared_timer_active = False
        self._reset_timer = None

        # Subscriber
        self.create_subscription(Bool, 'gui/auto_mode_active', self.callback_driving_mode, 1)
        self.create_subscription(Twist, 'gui/cmd_vel', self.callback_gui_cmd, 1)
        self.create_subscription(Int32, 'aruco/obstacle_detect', self.callback_obst_detect, 1)
        self.create_subscription(Twist, 'aruco/cmd_vel', self.callback_aruco_cmd, 1)
        self.create_subscription(Twist, 'lane/cmd_vel', self.callback_auto_cmd, 1)

        # Publisher
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 1)

        # GUI
        self.pub_obstacle_state = self.create_publisher(String, 'gui/obstacle_detect', 1)
        str_msg = String()
        str_msg.data = self.obstacle.name
        self.pub_obstacle_state.publish(str_msg)

        # Main loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info('server node start!!')

    def callback_driving_mode(self, msg: Bool):
        self.mode = Mode.AUTO if msg.data else Mode.MANUAL
        if self.mode != self._prev_mode:
            self.get_logger().info(f'모드 변경: {self._prev_mode.name} → {self.mode.name}')
            self._prev_mode = self.mode

    def callback_gui_cmd(self, msg: Twist):
        self.gui_cmd = msg

    def callback_obst_detect(self, msg: Int32):
        new_state = {
            0: ObstacleState.CLEAR,
            1: ObstacleState.BLOCKED,
            2: ObstacleState.CLEARED
        }.get(msg.data, None)

        if new_state is None:
            self.get_logger().warn(f'알 수 없는 장애물 상태: {msg.data}')
            return

        self.obstacle = new_state
        if self.obstacle != self._prev_obstacle:
            self.get_logger().info(f'장애물 상태 변경: {self._prev_obstacle.name} → {self.obstacle.name}')
            self._prev_obstacle = self.obstacle
            self._blocked_logged = False
            self._cleared_logged = False

            str_msg = String()
            str_msg.data = self.obstacle.name
            self.pub_obstacle_state.publish(str_msg)

    def callback_aruco_cmd(self, msg: Twist):
        self.aruco_cmd = msg

    def callback_auto_cmd(self, msg: Twist):
        self.auto_cmd = msg

    def reset_obstacle_state(self):
        self.obstacle = ObstacleState.CLEAR
        self._cleared_timer_active = False
        self._blocked_logged = False
        self._cleared_logged = False

        if self._reset_timer is not None:
            self._reset_timer.cancel()
            self._reset_timer = None

        self.get_logger().info('장애물 상태 CLEAR 전환')
        str_msg = String()
        str_msg.data = self.obstacle.name
        self.pub_obstacle_state.publish(str_msg)

    def control_loop(self):
        cmd = Twist()

        if self.obstacle == ObstacleState.CLEAR:
            cmd = self.gui_cmd if self.mode == Mode.MANUAL else self.auto_cmd

        elif self.obstacle == ObstacleState.BLOCKED:
            cmd = self.aruco_cmd
            if not self._blocked_logged:
                self.get_logger().warn("장애물 감지됨")
                self._blocked_logged = True

        elif self.obstacle == ObstacleState.CLEARED:
            cmd = Twist()  # 정지
            if not self._cleared_logged:
                self.get_logger().warn("장애물 제거 완료 - 복귀 대기")
                self._cleared_logged = True
            if not self._cleared_timer_active:
                self._reset_timer = self.create_timer(1.0, self.reset_obstacle_state)
                self._cleared_timer_active = True

        self.pub_cmd_vel.publish(cmd)

    def destroy_node(self):
        if self._reset_timer:
            self._reset_timer.cancel()
        if self.control_timer:
            self.control_timer.cancel()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    server = Server()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('서버 종료')
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
