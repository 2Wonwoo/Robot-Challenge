#!/usr/bin/env python3
"""AK60 네 바퀴 전진/후진 전용 노드 (회전 없음).

``FourWheelDriveNode`` 를 상속해 CAN 송신, MIT 모드 진입, 명령 타임아웃,
종료 시 정지를 그대로 물려받고 ``self.targets`` 만 직접 채운다.
``/cmd_vel`` 경로에는 "후진 금지" 안전 규칙이 걸려 있어 그 경로로는 후진이
0 으로 잘리므로, 전진·후진을 모두 쓰려면 이렇게 우회해야 한다.

회전을 아예 넣지 않았으므로 좌우 속도차가 생기지 않는다. 축 정렬이나 안쪽
바퀴 클램핑 같은 변수를 배제한 채로 전후진만 확인할 때 쓴다.
"""

import select
import sys
import termios
import tty

import rclpy

from ak60_driver.four_wheel_drive_node import FourWheelDriveNode

from .straight_drive import MOTOR_DIRECTION, straight_motor_targets


HELP = """
AK60 전진/후진 (회전 없음)
--------------------------------
  W : 전진        S : 후진
  SPACE / X : 정지
  + / - : 속도 조절
  Q : 정지 후 종료
"""


class StraightDriveNode(FourWheelDriveNode):
    """키보드로 네 바퀴를 같은 속도로 전진/후진시킨다."""

    SPEED_STEP = 0.02      # m/s, + / - 한 번에 바뀌는 양

    def __init__(self) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError(
                "straight_drive_node 는 대화형 터미널에서 실행해야 한다"
            )

        super().__init__()
        self.declare_parameter("drive_speed", 0.05)      # m/s
        self.declare_parameter("max_drive_speed", 0.30)  # m/s
        self.declare_parameter("poll_rate", 20.0)

        self.speed = float(self.get_parameter("drive_speed").value)
        self.max_speed = float(self.get_parameter("max_drive_speed").value)
        poll_rate = float(self.get_parameter("poll_rate").value)

        if self.speed <= 0.0:
            raise ValueError("drive_speed must be positive")
        if self.max_speed < self.speed:
            raise ValueError("max_drive_speed must not be smaller than drive_speed")
        if poll_rate <= 0.0:
            raise ValueError("poll_rate must be positive")

        self.direction = 0.0      # +1 전진, -1 후진, 0 정지
        self.old_terminal = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.terminal_restored = False
        self.keyboard_timer = self.create_timer(1.0 / poll_rate, self.update_keyboard)

        print(HELP, flush=True)
        self.get_logger().info(
            f"전진/후진 준비 완료: 속도 {self.speed:.3f} m/s "
            f"(최대 {self.max_speed:.3f}), 회전 없음"
        )

    @staticmethod
    def read_key():
        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        return sys.stdin.read(1).lower() if ready else None

    def drive(self, direction: float) -> None:
        """direction: +1 전진, -1 후진, 0 정지."""
        self.direction = direction
        self.targets = straight_motor_targets(
            direction * self.speed,
            self.wheel_radius,
            self.max_wheel_velocity,
            MOTOR_DIRECTION,
        )
        self.last_command_time = self.get_clock().now()
        self.timed_out = False

    def change_speed(self, delta: float) -> None:
        self.speed = min(max(self.speed + delta, self.SPEED_STEP), self.max_speed)
        self.get_logger().info(f"속도 {self.speed:.3f} m/s")

    def update_keyboard(self) -> None:
        key = self.read_key()
        if key == "w":
            self.direction = 1.0
        elif key == "s":
            self.direction = -1.0
        elif key in (" ", "x"):
            self.direction = 0.0
        elif key in ("+", "="):
            self.change_speed(self.SPEED_STEP)
        elif key in ("-", "_"):
            self.change_speed(-self.SPEED_STEP)
        elif key == "q":
            self.drive(0.0)
            rclpy.shutdown()
            return

        # 키를 누르지 않아도 마지막 선택을 유지하며 워치독을 갱신한다.
        # 이 갱신이 끊기면 구동 노드가 0.5초 뒤 모터를 세운다.
        self.drive(self.direction)

    def restore_terminal(self) -> None:
        if not self.terminal_restored:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_terminal)
            self.terminal_restored = True

    def destroy_node(self) -> bool:
        self.restore_terminal()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StraightDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
