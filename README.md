# robotchallenge

AK60-6 네 바퀴 로봇의 **전진/후진 주행** 코드.

회전을 넣지 않아 네 바퀴가 항상 같은 속도로 돈다. 좌우 속도차가 생기지 않으므로
축 정렬 오차나 안쪽 바퀴 클램핑 같은 변수를 배제한 상태에서 전후진만 확인할 수 있다.

> 같은 바퀴 배치를 OpenCR + DYNAMIXEL로 바퀴마다 키보드로 구동하는 코드(ROS 2 아님,
> PC에서 USB로 OpenCR 직접 연결)는 [`opencr_dxl_jog/`](opencr_dxl_jog/README.md) 참고.

## Quick Start

### A. AK60 전진/후진 주행 (ROS 2, CAN)

`straight_drive_node.py` 는 `ak60_driver` 패키지가 있는 워크스페이스 안에 두어야 한다
(아래 [ROS 2 노드로 주행](#ros-2-노드로-주행) 참고).

```bash
~/can_check.sh can1 1000000          # CAN 버스 기동 (터미널 열 때마다)
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run ak60_driver straight_drive_node
```

`W` 전진 · `S` 후진 · `SPACE`/`X` 정지 · `+`/`-` 속도 · `Q` 종료

### B. OpenCR + DYNAMIXEL 바퀴별 키보드 구동 (ROS 2 아님, USB)

최초 설치·업로드는 [`opencr_dxl_jog/README.md`](opencr_dxl_jog/README.md) 의 Quick Start 참고.

```bash
cd opencr_dxl_jog
python3 jog_each.py --scan           # ID 1~4 응답 확인
python3 key_jog.py                   # 키보드 구동
```

`q/a` ID1 · `w/s` ID2 · `e/d` ID3 · `r/f` ID4 · `space` 정지 · `h` 원점 · `b` 과부하 복구 · `x` 종료

### 기구학 테스트 (로봇 없이)

```bash
PYTHONPATH=. python3 -m pytest tests/ -q
```

## 바퀴 배치

```
        앞
   ID2 ●────● ID1
       │    │
   ID4 ●────● ID3
        뒤
```

| ID | 위치 | 모터 부호 |
|---|---|---|
| 1 | 앞-오른쪽 | +1 |
| 2 | 앞-왼쪽 | −1 |
| 3 | 뒤-오른쪽 | +1 |
| 4 | 뒤-왼쪽 | −1 |

왼쪽 모터는 물리적으로 반대를 향해 설치되어 있어 부호가 뒤집힌다.
그래서 **전진이 양수**인 값을 넣으면 `straight_motor_targets` 가 알아서 보정한다.

## 구성

| 파일 | 역할 | 의존성 |
|---|---|---|
| `ak60_drive/straight_drive.py` | 전진/후진 기구학 (순수 파이썬) | 없음 |
| `ak60_drive/straight_drive_node.py` | 키보드 주행 ROS 2 노드 | ROS 2, `ak60_driver` 패키지 |
| `tests/test_straight_drive.py` | 기구학 테스트 | pytest |

## 테스트

로봇도 ROS 도 없이 돌아간다:

```bash
PYTHONPATH=. python3 -m pytest tests/ -q
```

## 사용법

### 기구학만 쓰기

```python
from ak60_drive.straight_drive import straight_motor_targets

# 0.1 m/s 전진, 바퀴 반지름 0.10 m, 상한 2.0 rad/s
targets = straight_motor_targets(0.1, wheel_radius=0.10, max_wheel_velocity=2.0)
# {1: 1.0, 2: -1.0, 3: 1.0, 4: -1.0}

straight_motor_targets(-0.1, 0.10, 2.0)   # 후진
# {1: -1.0, 2: 1.0, 3: -1.0, 4: 1.0}
```

### ROS 2 노드로 주행

`straight_drive_node.py` 는 `ak60_driver` 패키지의 `FourWheelDriveNode` 를 상속해
CAN 송신·MIT 모드 진입·명령 타임아웃·종료 시 정지를 물려받는다. 따라서 **그 패키지가
있는 워크스페이스 안에 두어야** 실행된다.

```bash
ros2 run ak60_driver straight_drive_node
```

```
  W : 전진        S : 후진
  SPACE / X : 정지
  + / - : 속도 조절
  Q : 정지 후 종료
```

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `drive_speed` | 0.05 | 주행 속도 (m/s) |
| `max_drive_speed` | 0.30 | 속도 상한 (m/s) |
| `poll_rate` | 20.0 | 키 입력 확인 주기 (Hz) |

## 알아둘 점

### 후진은 두 군데에서 막힐 수 있다

`ak60_driver` 패키지에는 `/cmd_vel` 경로에 후진 금지 규칙이 있다. 실제로 다음
**두 곳**에서 음수 속도가 0 으로 잘린다:

```python
forward_only_wheel_velocities(...)   # clamp(..., 0.0, max)
motor_controller_targets(...)        # max(0.0, ...)
```

한쪽만 풀면 여전히 후진이 안 된다. 이 노드는 `self.targets` 를 직접 채워 두 경로를
모두 우회한다.

### 토크는 `kd` 가 정한다

MIT 모드에서 `kp=0` 으로 쓰므로 토크는 속도 오차에만 비례한다:

```
토크 = kd × (목표속도 − 현재속도)
```

낮은 속도로 주행하면 토크도 같이 작아진다. 바퀴가 힘없이 기어가면 속도가 아니라
`kd` 를 올려야 한다 (프로토콜 상한 5.0).
