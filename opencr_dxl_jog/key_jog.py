"""
OpenCR(dxl_bridge.ino 펌웨어 업로드된 상태)로 다이나믹셀 ID 1~4를 키보드로 개별 구동.
키를 누르고 있는 동안 해당 모터가 계속 돌고, 떼면 멈춘다.

동작 방식:
  Extended Position Mode(다회전)로 초기화한 뒤, 키 입력(누르고 있으면 자동 반복)마다
  JOG 명령으로 목표 위치를 STEP만큼 옮긴다. 펌웨어가 목표를 현재 위치에서 ±LEAD틱
  이상 앞서지 못하게 막고, 키를 떼면(RELEASE_SEC 동안 입력 없음) HOLD로 목표를
  현재 위치에 맞춰 밀던 힘을 푼다.

키 배치 (정방향 / 역방향):
  ID1 앞오른쪽 : q / a
  ID2 앞왼쪽   : w / s
  ID3 뒤오른쪽 : e / d
  ID4 뒤왼쪽   : r / f
  space : 전체 정지 (현재 위치에서 고정)
  h     : 원점 복귀 (프로그램 시작 시점의 위치로 전체 복귀)
  b     : 과부하 등 하드웨어 에러로 토크가 꺼진 모터 재부팅 후 복구
  x     : 종료 (토크 OFF)

정방향은 바퀴 기준 전진 방향 (ID1,3은 +, ID2,4는 - 부호, 좌우 대칭 장착).

사용법:
  python3 key_jog.py [--port /dev/ttyACM1] [--step 15] [--lead 100] [--vel 50]
                     [--p 1500 --i 200 --d 0]

토크:
  위치 제어는 (목표 - 현재) 오차에 비례해 힘을 낸다. 목표가 몇 틱만 앞서 있으므로
  P 게인만으로는 부하가 걸리면 힘이 모자라 멈춘다. I 게인이 멈춘 동안 오차를 누적해
  출력을 키워준다. 게인이 너무 높으면 떨림(진동)이 생기니 그때는 낮출 것.

과부하(Hardware Error 32):
  최대 출력이 수 초 이상 이어지면 모터가 스스로 토크를 끈다. 막힌 상태에서 키를 계속
  누르고 있으면 LEAD 제한이 있어도 I 게인이 누적되어 생길 수 있다. 1초마다 에러를
  확인해 알려주며, b로 재부팅하면 풀린다. 재부팅하면 다회전 위치가 한 바퀴 안으로
  초기화되므로 원점은 재부팅 전후 위치 차이만큼 보정한다.
"""

import argparse
import select
import sys
import termios
import time
import tty

from jog_each import DEFAULT_PORT, TICKS_PER_DEG, OpenCRDxl

# key -> (id, +1 정방향 / -1 역방향)
KEYMAP = {
    "q": (1, +1), "a": (1, -1),
    "w": (2, +1), "s": (2, -1),
    "e": (3, +1), "d": (3, -1),
    "r": (4, +1), "f": (4, -1),
}
FORWARD_SIGN = {1: +1, 2: -1, 3: +1, 4: -1}
IDS = [1, 2, 3, 4]
TICKS_PER_REV = 4096
RELEASE_SEC = 0.6     # 키 자동 반복 시작 지연(~0.5s)보다 길어야 누르는 중에 HOLD가 끼지 않는다
HWERR_POLL_SEC = 1.0
HWERR_NAMES = {0: "입력전압", 2: "과열", 3: "엔코더", 4: "전기충격", 5: "과부하"}


def read_key(timeout=0.05):
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1)
    return None


def pump(dxl: OpenCRDxl, goals: dict):
    """도착한 응답 줄을 모두 읽고, JOG/HOLD 응답이면 goals를 갱신한다."""
    lines = []
    while dxl.ser.in_waiting:
        raw = dxl.ser.readline().decode(errors="ignore").strip()
        if not raw:
            continue
        parts = raw.split()
        if raw.startswith(("OK JOG", "OK HOLD")) and len(parts) == 4:
            goals[int(parts[2])] = int(parts[3])
        lines.append(raw)
    return lines


def query(dxl: OpenCRDxl, goals: dict, cmd: str, prefix: str, timeout=0.3):
    """명령을 보내고 prefix로 시작하는 응답 줄을 반환 (없으면 None)."""
    dxl._send(cmd)
    deadline = time.time() + timeout
    while time.time() < deadline:
        for line in pump(dxl, goals):
            if line.startswith(prefix):
                return line
        time.sleep(0.002)
    return None


def read_pos(dxl, goals, motor_id):
    line = query(dxl, goals, f"READ {motor_id}", f"OK READ {motor_id} ")
    return int(line.split()[-1]) if line else None


def read_hwerr(dxl, goals, motor_id):
    line = query(dxl, goals, f"HWERR {motor_id}", f"OK HWERR {motor_id} ")
    return int(line.split()[-1]) if line else None


def hwerr_text(value: int) -> str:
    names = [name for bit, name in HWERR_NAMES.items() if value & (1 << bit)]
    return f"{value} ({', '.join(names) or '기타'})"


def setup_motor(dxl, goals, motor_id, args):
    """INIT(EXT) + GAIN + HOLD. 성공하면 현재 위치, 실패하면 None."""
    if not query(dxl, goals, f"INIT {motor_id} {args.vel} EXT", f"OK INIT {motor_id}"):
        return None
    if not query(dxl, goals, f"GAIN {motor_id} {args.p} {args.i} {args.d}", f"OK GAIN {motor_id}"):
        print(f"ID {motor_id} GAIN 설정 실패 (기본 게인으로 진행)")
    if not query(dxl, goals, f"HOLD {motor_id}", f"OK HOLD {motor_id}"):
        return None
    return goals[motor_id]


def reboot_motor(dxl, goals, origins, motor_id, args):
    """재부팅으로 하드웨어 에러를 풀고 다시 초기화. 원점은 한 바퀴 단위로 보정."""
    before = read_pos(dxl, goals, motor_id)
    if not query(dxl, goals, f"REBOOT {motor_id}", f"OK REBOOT {motor_id}"):
        return False
    time.sleep(1.0)  # 재부팅 대기
    after = setup_motor(dxl, goals, motor_id, args)
    if after is None:
        return False
    if before is not None and motor_id in origins:
        turns = round((before - after) / TICKS_PER_REV)
        origins[motor_id] -= turns * TICKS_PER_REV
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--step", type=int, default=15, help="키 반복 1회당 이동량(틱)")
    ap.add_argument("--lead", type=int, default=100, help="목표가 현재 위치보다 앞설 수 있는 최대 틱")
    ap.add_argument("--vel", type=int, default=50, help="Profile Velocity (0.229rpm 단위)")
    ap.add_argument("--p", type=int, default=1500, help="Position P gain (기본값 800)")
    ap.add_argument("--i", type=int, default=200, help="Position I gain (기본값 0)")
    ap.add_argument("--d", type=int, default=0, help="Position D gain (기본값 0)")
    args = ap.parse_args()

    dxl = OpenCRDxl(port=args.port)
    print(f"[연결됨] {args.port}")

    goals = {}
    try:
        for motor_id in IDS:
            err = read_hwerr(dxl, goals, motor_id)
            if err is None:
                print(f"ID {motor_id} 응답 없음 -> 건너뜀")
                continue
            if err:
                print(f"ID {motor_id} 하드웨어 에러 {hwerr_text(err)} -> 재부팅")
                query(dxl, goals, f"REBOOT {motor_id}", f"OK REBOOT {motor_id}")
                time.sleep(1.0)
            if setup_motor(dxl, goals, motor_id, args) is None:
                print(f"ID {motor_id} 초기화 실패 -> 건너뜀")
                goals.pop(motor_id, None)
        if not goals:
            print("구동 가능한 모터가 없습니다.")
            return
        origins = dict(goals)  # 시작 시점 위치 = 원점

        print(f"준비 완료: ID {sorted(goals)}  (step {args.step}틱 = "
              f"{args.step / TICKS_PER_DEG:.1f}도, lead {args.lead}틱)")
        print(f"게인: P={args.p} I={args.i} D={args.d}")
        print(f"원점: {origins}")
        print("ID1 q/a  ID2 w/s  ID3 e/d  ID4 r/f  |  space 정지  h 원점  b 복구  x 종료")

        faulted = set()
        jogged = set()        # 키를 뗐을 때 HOLD할 모터
        last_key = 0.0
        last_poll = time.time()

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while True:
                key = read_key()
                pump(dxl, goals)
                now = time.time()

                if key is None:
                    if jogged and now - last_key > RELEASE_SEC:
                        for motor_id in jogged:
                            dxl._send(f"HOLD {motor_id}")
                        jogged.clear()
                    if now - last_key > RELEASE_SEC and now - last_poll > HWERR_POLL_SEC:
                        last_poll = now
                        for motor_id in goals:
                            err = read_hwerr(dxl, goals, motor_id)
                            if err and motor_id not in faulted:
                                faulted.add(motor_id)
                                print(f"\n[경고] ID {motor_id} 토크 꺼짐: 하드웨어 에러 "
                                      f"{hwerr_text(err)} -> b 로 복구")
                    continue

                last_key = now
                key = key.lower()
                if key == "x":
                    break
                if key == " ":
                    for motor_id in goals:
                        dxl._send(f"HOLD {motor_id}")
                    jogged.clear()
                    print("\n전체 정지")
                    continue
                if key == "h":
                    for motor_id, origin in origins.items():
                        goals[motor_id] = origin
                        dxl._send(f"MOVE {motor_id} {origin}")
                    jogged.clear()
                    print("\n원점 복귀")
                    continue
                if key == "b":
                    targets = sorted(faulted) or [
                        m for m in goals if read_hwerr(dxl, goals, m)]
                    if not targets:
                        print("\n하드웨어 에러 난 모터 없음")
                        continue
                    for motor_id in targets:
                        ok = reboot_motor(dxl, goals, origins, motor_id, args)
                        print(f"\nID {motor_id} 복구 {'완료' if ok else '실패'}")
                        if ok:
                            faulted.discard(motor_id)
                    last_key = time.time()
                    continue
                if key in KEYMAP:
                    motor_id, direction = KEYMAP[key]
                    if motor_id not in goals:
                        continue
                    delta = direction * FORWARD_SIGN[motor_id] * args.step
                    dxl._send(f"JOG {motor_id} {delta} {args.lead}")
                    jogged.add(motor_id)
                    print(f"\rID{motor_id} goal {goals[motor_id]:>8}", end="", flush=True)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    except KeyboardInterrupt:
        pass
    finally:
        time.sleep(0.2)
        for motor_id in IDS:
            dxl.torque(motor_id, False)
        dxl.close()
        print("\n토크 OFF, 종료")


if __name__ == "__main__":
    main()
