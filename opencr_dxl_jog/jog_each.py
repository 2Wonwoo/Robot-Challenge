"""
OpenCR(dxl_bridge.ino 펌웨어 업로드된 상태)로 다이나믹셀을 하나씩 조금씩 움직여 보는 스크립트.

동작 (ID마다 차례로):
  INIT(Position Mode, 느린 Profile Velocity, 토크 ON) -> 현재 위치 읽기
  -> +step 이동 -> 원위치 복귀 -> 다음 ID
  끝나면(또는 Ctrl+C) 모든 모터 토크 OFF.

사용법:
  python3 jog_each.py                       # ID 1~4, 80틱(약 7도)
  python3 jog_each.py --ids 1 3 --step 40   # ID 1, 3만, 40틱(약 3.5도)
  python3 jog_each.py --scan                # 먼저 버스 스캔만 하고 종료
"""

import argparse
import time

import serial

DEFAULT_PORT = "/dev/ttyACM1"
DEFAULT_BAUD = 115200
POS_MIN = 0
POS_MAX = 4095
TICKS_PER_DEG = 4096 / 360.0


class OpenCRDxl:
    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD, timeout=2.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)  # OpenCR 리셋 및 부팅 대기
        self.ser.reset_input_buffer()

    def close(self):
        self.ser.close()

    def _send(self, line: str):
        self.ser.write((line.strip() + "\n").encode())

    def _read_until_idle(self, wait=0.3):
        lines = []
        deadline = time.time() + wait
        while time.time() < deadline:
            if self.ser.in_waiting:
                raw = self.ser.readline().decode(errors="ignore").strip()
                if raw:
                    lines.append(raw)
                    deadline = time.time() + wait
        return lines

    def _ok(self, line: str, prefix: str):
        self._send(line)
        return any(l.startswith(prefix) for l in self._read_until_idle())

    def scan(self):
        self._send("SCAN")
        lines = self._read_until_idle(wait=1.5)
        motors = []
        for l in lines:
            if l.startswith("FOUND"):
                # FOUND ID <id> BAUD <baud> MODEL <model>
                parts = l.split()
                motors.append({"id": int(parts[2]), "baud": int(parts[4]),
                               "model": int(parts[6])})
        return motors

    def set_baud(self, bps: int):
        return self._ok(f"BAUD {bps}", "OK BAUD")

    def init_position_mode(self, motor_id: int, profile_vel: int):
        return self._ok(f"INIT {motor_id} {profile_vel}", "OK INIT")

    def torque(self, motor_id: int, on: bool):
        return self._ok(f"TORQUE {motor_id} {1 if on else 0}", "OK TORQUE")

    def move(self, motor_id: int, goal_position: int):
        return self._ok(f"MOVE {motor_id} {goal_position}", "OK MOVE")

    def read_position(self, motor_id: int):
        self._send(f"READ {motor_id}")
        for l in self._read_until_idle():
            if l.startswith("OK READ"):
                return int(l.split()[-1])
        return None

    def wait_until(self, motor_id: int, goal: int, tol=10, timeout=5.0):
        """목표 위치 tol틱 이내에 들어올 때까지 대기. 마지막으로 읽은 위치 반환."""
        deadline = time.time() + timeout
        pos = None
        while time.time() < deadline:
            pos = self.read_position(motor_id)
            if pos is not None and abs(pos - goal) <= tol:
                break
        return pos


def clamp(p: int) -> int:
    return max(POS_MIN, min(POS_MAX, p))


def jog(dxl: OpenCRDxl, motor_id: int, step: int, profile_vel: int):
    print(f"\n[ID {motor_id}]")
    if not dxl.init_position_mode(motor_id, profile_vel):
        print("  INIT 실패 (응답 없음/배선 확인) -> 건너뜀")
        return False

    start = dxl.read_position(motor_id)
    if start is None:
        print("  위치 읽기 실패 -> 건너뜀")
        return False
    target = clamp(start + step)
    print(f"  시작 {start} -> 목표 {target} ({(target - start) / TICKS_PER_DEG:+.1f}도)")

    dxl.move(motor_id, target)
    reached = dxl.wait_until(motor_id, target)
    print(f"  도달 {reached}")
    time.sleep(0.3)

    dxl.move(motor_id, start)
    back = dxl.wait_until(motor_id, start)
    print(f"  복귀 {back}")
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4])
    ap.add_argument("--step", type=int, default=80, help="이동량(틱), 4096틱=360도")
    ap.add_argument("--vel", type=int, default=10, help="Profile Velocity (1이 최저)")
    ap.add_argument("--scan", action="store_true", help="스캔만 하고 종료")
    args = ap.parse_args()

    dxl = OpenCRDxl(port=args.port)
    print(f"[연결됨] {args.port}")
    try:
        motors = dxl.scan()
        if not motors:
            print("모터를 찾지 못했습니다. 전원/배선/보드레이트를 확인하세요.")
            return
        for m in motors:
            print(f"  FOUND ID={m['id']}  BAUD={m['baud']}  MODEL={m['model']}")
        if args.scan:
            return
        dxl.set_baud(motors[0]["baud"])

        found = {m["id"] for m in motors}
        for motor_id in args.ids:
            if motor_id not in found:
                print(f"\n[ID {motor_id}] 스캔에서 안 보임 -> 건너뜀")
                continue
            jog(dxl, motor_id, args.step, args.vel)
    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        for motor_id in args.ids:
            dxl.torque(motor_id, False)
        dxl.close()
        print("토크 OFF, 종료")


if __name__ == "__main__":
    main()
