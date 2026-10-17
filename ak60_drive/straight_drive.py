"""Pure straight-line (forward/reverse only) kinematics for the four AK60 wheels.

회전이 없으므로 네 바퀴가 항상 같은 속도로 돈다. 좌우 속도차가 아예 생기지
않아 축 정렬 오차나 안쪽 바퀴 클램핑 같은 변수를 배제한 상태에서 전진·후진만
확인할 수 있다.

ROS 와 CAN 에 의존하지 않으므로 로봇 없이 테스트할 수 있다.
"""

from typing import Dict


# 바퀴를 전진시키기 위해 각 모터에 보내야 하는 부호.
# 왼쪽(2, 4)은 물리적으로 반대를 향해 설치되어 있다.
#
#         앞
#    ID2 ●────● ID1
#        │    │
#    ID4 ●────● ID3
#         뒤
MOTOR_DIRECTION: Dict[int, float] = {
    1: 1.0,   # 앞-오른쪽
    2: -1.0,  # 앞-왼쪽
    3: 1.0,   # 뒤-오른쪽
    4: -1.0,  # 뒤-왼쪽
}


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def straight_wheel_velocity(
    speed: float, wheel_radius: float, max_wheel_velocity: float
) -> float:
    """주행 속도(m/s) 를 바퀴 각속도(rad/s) 로 바꾼다. 전진이 양수.

    반환값은 **물리적** 바퀴 속도다. 모터 설치 방향 보정은 하지 않는다.
    """
    if wheel_radius <= 0.0:
        raise ValueError("wheel_radius must be positive")
    if max_wheel_velocity <= 0.0:
        raise ValueError("max_wheel_velocity must be positive")
    return clamp(speed / wheel_radius, -max_wheel_velocity, max_wheel_velocity)


def straight_motor_targets(
    speed: float,
    wheel_radius: float,
    max_wheel_velocity: float,
    motor_direction: Dict[int, float] = MOTOR_DIRECTION,
) -> Dict[int, float]:
    """네 모터에 보낼 컨트롤러 기준 속도.

    왼쪽 모터는 물리적으로 반대를 향해 설치되어 있으므로 ``motor_direction``
    으로 부호를 뒤집는다. 전진·후진 모두 네 바퀴가 같은 물리 속도로 돈다.
    """
    wheel = straight_wheel_velocity(speed, wheel_radius, max_wheel_velocity)
    return {motor_id: sign * wheel for motor_id, sign in motor_direction.items()}
