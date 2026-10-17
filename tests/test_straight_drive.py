import pytest

from ak60_drive.straight_drive import (
    MOTOR_DIRECTION,
    straight_motor_targets,
    straight_wheel_velocity,
)


DIRECTION = MOTOR_DIRECTION
RADIUS = 0.10
MAX = 2.0


def targets(speed, **overrides):
    settings = dict(
        wheel_radius=RADIUS, max_wheel_velocity=MAX, motor_direction=DIRECTION
    )
    settings.update(overrides)
    return straight_motor_targets(speed, **settings)


def physical(motor_targets):
    """컨트롤러 값을 다시 물리 바퀴 속도로 되돌린다 (부호 보정 해제)."""
    return {
        motor_id: value * DIRECTION[motor_id]
        for motor_id, value in motor_targets.items()
    }


def test_forward_drives_all_four_wheels_the_same_way():
    moving = physical(targets(0.1))
    assert set(moving) == {1, 2, 3, 4}
    assert all(v > 0.0 for v in moving.values())
    assert len(set(moving.values())) == 1      # 네 바퀴가 완전히 같은 속도


def test_reverse_drives_all_four_wheels_backwards():
    moving = physical(targets(-0.1))
    assert all(v < 0.0 for v in moving.values())
    assert len(set(moving.values())) == 1


def test_left_motors_are_mirrored_in_controller_frame():
    # 왼쪽(2,4)은 반대로 설치되어 있어 컨트롤러 값의 부호가 뒤집힌다
    command = targets(0.1)
    assert command[1] > 0.0 and command[3] > 0.0
    assert command[2] < 0.0 and command[4] < 0.0


def test_zero_speed_stops_every_wheel():
    assert all(v == 0.0 for v in targets(0.0).values())


def test_speed_converts_through_the_wheel_radius():
    # 0.2 m/s / 0.10 m = 2.0 rad/s
    assert straight_wheel_velocity(0.2, RADIUS, MAX) == pytest.approx(2.0)


def test_speed_is_clamped_in_both_directions():
    assert straight_wheel_velocity(99.0, RADIUS, MAX) == pytest.approx(MAX)
    assert straight_wheel_velocity(-99.0, RADIUS, MAX) == pytest.approx(-MAX)


def test_reverse_is_not_clipped_to_zero():
    """/cmd_vel 경로는 후진을 0 으로 자르지만 이 경로는 그러면 안 된다."""
    assert straight_wheel_velocity(-0.1, RADIUS, MAX) < 0.0


def test_invalid_geometry_is_rejected():
    with pytest.raises(ValueError):
        straight_wheel_velocity(0.1, 0.0, MAX)
    with pytest.raises(ValueError):
        straight_wheel_velocity(0.1, RADIUS, 0.0)
