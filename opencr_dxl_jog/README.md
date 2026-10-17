# opencr_dxl_jog

**OpenCR + DYNAMIXEL(XM430-W350, Protocol 2.0) 4바퀴를 키보드로 개별 구동.**
Linux PC에서 USB로 OpenCR에 직접 연결한다. ROS 2는 쓰지 않고, Arduino 펌웨어
(텍스트 명령 브릿지) + 순수 Python 스크립트로 동작한다.

```
Linux PC (USB) ---- OpenCR ---- DXL 버스(57600bps) ---- DYNAMIXEL ID 1~4
```

## 바퀴 배치

```
        앞
   ID2 ●────● ID1
       │    │
   ID4 ●────● ID3
        뒤
```

| ID | 위치 | 전진 부호 |
|---|---|---|
| 1 | 앞-오른쪽 | +1 |
| 2 | 앞-왼쪽 | −1 |
| 3 | 뒤-오른쪽 | +1 |
| 4 | 뒤-왼쪽 | −1 |

## 준비 (최초 1회)

```bash
# arduino-cli + OpenCR 코어 + Dynamixel2Arduino
mkdir -p ~/.local/bin
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
export PATH="$HOME/.local/bin:$PATH"
arduino-cli config init
arduino-cli config set board_manager.additional_urls \
    https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json
arduino-cli core update-index
arduino-cli core install OpenCR:OpenCR
arduino-cli lib install Dynamixel2Arduino

# OpenCR 컴파일러(arm-none-eabi-g++)가 32비트 바이너리라 i386 라이브러리가 필요하다
sudo apt install -y libc6:i386 libstdc++6:i386 libncurses5:i386

pip install pyserial
```

## 업로드

```bash
cd opencr_dxl_jog
arduino-cli compile --fqbn OpenCR:OpenCR:OpenCR dxl_bridge
arduino-cli upload -p /dev/ttyACM1 --fqbn OpenCR:OpenCR:OpenCR dxl_bridge
```

포트는 `ls -l /dev/serial/by-id/` 에서 `ROBOTIS_OpenCR` 로 연결된 ttyACM을 쓴다
(같은 PC에 Jetson 콘솔 등 다른 ttyACM이 있을 수 있음).

## 실행

```bash
python3 jog_each.py --scan          # 버스 스캔만
python3 jog_each.py                 # ID 1~4를 하나씩 7도 움직였다 복귀
python3 key_jog.py                  # 키보드로 개별 구동
```

### key_jog.py 키

| 모터 | 정방향(전진) | 역방향 |
|---|---|---|
| ID1 앞오른쪽 | `q` | `a` |
| ID2 앞왼쪽 | `w` | `s` |
| ID3 뒤오른쪽 | `e` | `d` |
| ID4 뒤왼쪽 | `r` | `f` |

`space` 전체 정지 · `h` 원점 복귀(시작 시점 위치) · `b` 하드웨어 에러 모터 재부팅/복구 · `x` 종료(토크 OFF)

| 인자 | 기본값 | 설명 |
|---|---|---|
| `--step` | 15 | 키 반복 1회당 이동량(틱, 4096틱=360도) |
| `--lead` | 100 | 목표가 현재 위치보다 앞설 수 있는 최대 틱 |
| `--vel` | 50 | Profile Velocity (0.229rpm 단위) |
| `--p` / `--i` / `--d` | 1500 / 200 / 0 | Position PID 게인 (모터 기본값 800 / 0 / 0) |

## 펌웨어 명령 (`dxl_bridge.ino`, USB Serial 115200, 한 줄씩)

| 명령 | 설명 |
|---|---|
| `SCAN` | 57600/1000000에서 ID 0~20 스캔 |
| `BAUD <bps>` | DXL 버스 보드레이트 변경 |
| `PING <id>` | 핑 + 모델 번호 |
| `TORQUE <id> <0\|1>` | 토크 ON/OFF |
| `MOVE <id> <goal>` | 목표 위치 |
| `READ <id>` | 현재 위치 |
| `INIT <id> <vel> [EXT]` | Position(EXT면 Extended Position) 모드 + Profile Velocity + 토크 ON |
| `GAIN <id> <P> <I> <D>` | Position PID 게인 (RAM) |
| `JOG <id> <delta> <lead>` | 목표 += delta, 단 현재 위치 ±lead 이내로 제한 |
| `HOLD <id>` | 목표 = 현재 위치 |
| `REBOOT <id>` | 모터 재부팅 (하드웨어 에러 해제) |
| `HWERR <id>` | Hardware Error Status |
| `DIAG <id>` | 전압/온도/에러/모드/전류제한/PWM제한/게인/프로파일/현재전류 |

## 알아둘 점

### 토크가 약하면 게인 문제다

위치 제어는 `(목표 − 현재)` 오차에 비례해 힘을 낸다. 키 입력마다 목표를 조금씩만
앞세우므로 기본 게인(P=800, I=0)으로는 최대 출력의 10% 정도밖에 안 나온다.
그래서 P를 올리고 I를 넣어, 막혀 있는 동안 출력이 커지게 했다. 떨림이 생기면
게인을 낮춘다.

### 오래 막히면 과부하로 토크가 꺼진다 (Hardware Error 32)

최대 출력이 수 초 이어지면 모터가 스스로 토크를 끄고 LED가 깜빡인다. 이를 줄이려고

- 펌웨어 `JOG`가 목표를 현재 위치에서 `lead` 틱 이상 앞서지 못하게 막고,
- 키를 떼면(0.6초 입력 없음) `HOLD`로 목표를 현재 위치에 맞춰 밀던 힘을 푼다.

그래도 막힌 채로 키를 계속 누르면 I 누적으로 날 수 있다. `key_jog.py`가 1초마다
에러를 확인해 알려주며, `b`로 재부팅하면 풀린다.

### 재부팅하면 다회전 위치가 초기화된다

Extended Position 모드의 다회전 값은 재부팅 시 한 바퀴 안으로 돌아간다.
`b`로 복구할 때 재부팅 전후 위치 차이를 한 바퀴 단위로 계산해 원점을 보정한다.
게인·Profile Velocity도 RAM 값이라 재부팅/전원 차단 시 기본값으로 돌아가므로
복구 과정에서 다시 설정한다.
