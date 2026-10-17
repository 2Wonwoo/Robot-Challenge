/*
  OpenCR + DYNAMIXEL(X-series, Protocol 2.0) 제어 펌웨어
  - PC에서 USB Serial로 텍스트 명령을 보내면 DXL 버스를 제어하고 결과를 회신한다.
  - 필요한 라이브러리: Arduino IDE 라이브러리 매니저에서 "Dynamixel2Arduino" (ROBOTIS) 설치

  명령 목록 (한 줄씩, 개행으로 종료):
    SCAN                    -> 57600 / 1000000 두 보드레이트에서 ID 0~20 스캔, 발견된 ID와 모델번호 출력
    BAUD <bps>              -> DXL 버스 통신 보드레이트 변경 (기본 57600)
    PING <id>
    TORQUE <id> <0|1>
    MOVE <id> <goal_pos>    -> Position Control Mode 기준, goal_pos: 0~4095
    READ <id>               -> 현재 위치 출력
    INIT <id> <profile_vel> [EXT]
                            -> 토크 OFF -> Position Control Mode(EXT면 Extended Position, 다회전)
                               + Profile Velocity 설정 -> 토크 ON
    GAIN <id> <P> <I> <D>   -> Position P/I/D 게인 설정 (RAM, 모터 전원 끄면 기본값 복귀)
    JOG <id> <delta> <lead> -> 목표를 delta만큼 이동하되, 현재 위치에서 ±lead틱 이상 앞서지 않게 제한
    HOLD <id>               -> 목표 = 현재 위치 (밀던 힘을 풀고 그 자리에 정지)
    REBOOT <id>             -> 모터 재부팅 (과부하 등 하드웨어 에러 해제, RAM 설정 초기화)
    HWERR <id>              -> Hardware Error Status 출력 (32 = 과부하)
    DIAG <id>               -> 전압/온도/하드웨어에러/전류제한/PID게인/현재전류/모드 출력
    HELP
*/

#include <Dynamixel2Arduino.h>

#define DXL_SERIAL   Serial3
#define DEBUG_SERIAL Serial
const int DXL_DIR_PIN = 84; // OpenCR 보드의 DXL 방향 제어 핀

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);
using namespace ControlTableItem;

const uint8_t SCAN_ID_MIN = 0;
const uint8_t SCAN_ID_MAX = 20;
const long SCAN_BAUDRATES[] = {57600, 1000000};

long currentBaud = 57600;

void setup() {
  DEBUG_SERIAL.begin(115200);
  while (!DEBUG_SERIAL) {}

  dxl.begin(currentBaud);
  dxl.setPortProtocolVersion(2.0);

  DEBUG_SERIAL.println("READY OpenCR DXL Controller");
  DEBUG_SERIAL.println("HELP 입력 시 명령 목록 출력");
}

void loop() {
  if (DEBUG_SERIAL.available()) {
    String line = DEBUG_SERIAL.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      handleCommand(line);
    }
  }
}

void printHelp() {
  DEBUG_SERIAL.println("SCAN");
  DEBUG_SERIAL.println("BAUD <bps>");
  DEBUG_SERIAL.println("PING <id>");
  DEBUG_SERIAL.println("TORQUE <id> <0|1>");
  DEBUG_SERIAL.println("MOVE <id> <goal_pos>");
  DEBUG_SERIAL.println("READ <id>");
  DEBUG_SERIAL.println("INIT <id> <profile_vel> [EXT]");
  DEBUG_SERIAL.println("GAIN <id> <P> <I> <D>");
  DEBUG_SERIAL.println("JOG <id> <delta> <lead>");
  DEBUG_SERIAL.println("HOLD <id>");
  DEBUG_SERIAL.println("REBOOT <id>");
  DEBUG_SERIAL.println("HWERR <id>");
  DEBUG_SERIAL.println("DIAG <id>");
}

void handleCommand(String line) {
  int sp1 = line.indexOf(' ');
  String cmd = (sp1 == -1) ? line : line.substring(0, sp1);
  cmd.toUpperCase();

  if (cmd == "HELP") {
    printHelp();
  } else if (cmd == "SCAN") {
    doScan();
  } else if (cmd == "BAUD") {
    long bps = line.substring(sp1 + 1).toInt();
    if (bps > 0) {
      currentBaud = bps;
      dxl.begin(currentBaud);
      DEBUG_SERIAL.print("OK BAUD ");
      DEBUG_SERIAL.println(currentBaud);
    } else {
      DEBUG_SERIAL.println("ERR BAUD");
    }
  } else if (cmd == "PING") {
    uint8_t id = (uint8_t)line.substring(sp1 + 1).toInt();
    if (dxl.ping(id)) {
      DEBUG_SERIAL.print("OK PING ");
      DEBUG_SERIAL.print(id);
      DEBUG_SERIAL.print(" MODEL ");
      DEBUG_SERIAL.println(dxl.getModelNumber(id));
    } else {
      DEBUG_SERIAL.print("ERR PING ");
      DEBUG_SERIAL.println(id);
    }
  } else if (cmd == "TORQUE") {
    int sp2 = line.indexOf(' ', sp1 + 1);
    uint8_t id = (uint8_t)line.substring(sp1 + 1, sp2).toInt();
    int on = line.substring(sp2 + 1).toInt();
    bool ok = on ? dxl.torqueOn(id) : dxl.torqueOff(id);
    DEBUG_SERIAL.println(ok ? ("OK TORQUE " + String(id) + " " + String(on))
                             : ("ERR TORQUE " + String(id)));
  } else if (cmd == "MOVE") {
    int sp2 = line.indexOf(' ', sp1 + 1);
    uint8_t id = (uint8_t)line.substring(sp1 + 1, sp2).toInt();
    int32_t goal = line.substring(sp2 + 1).toInt();
    bool ok = dxl.setGoalPosition(id, goal);
    DEBUG_SERIAL.println(ok ? ("OK MOVE " + String(id) + " " + String(goal))
                             : ("ERR MOVE " + String(id)));
  } else if (cmd == "READ") {
    uint8_t id = (uint8_t)line.substring(sp1 + 1).toInt();
    int32_t pos = dxl.getPresentPosition(id);
    DEBUG_SERIAL.print("OK READ ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.println(pos);
  } else if (cmd == "INIT") {
    int sp2 = line.indexOf(' ', sp1 + 1);
    uint8_t id = (uint8_t)line.substring(sp1 + 1, sp2).toInt();
    int sp3 = (sp2 == -1) ? -1 : line.indexOf(' ', sp2 + 1);
    int32_t vel = (sp2 == -1) ? 10 : line.substring(sp2 + 1, sp3 == -1 ? line.length() : sp3).toInt();
    bool ext = (sp3 != -1) && line.substring(sp3 + 1).equalsIgnoreCase("EXT");
    bool ok = dxl.torqueOff(id)
           && dxl.setOperatingMode(id, ext ? OP_EXTENDED_POSITION : OP_POSITION)
           && dxl.writeControlTableItem(PROFILE_VELOCITY, id, vel)
           && dxl.torqueOn(id);
    DEBUG_SERIAL.println(ok ? ("OK INIT " + String(id) + " " + String(vel) + (ext ? " EXT" : ""))
                             : ("ERR INIT " + String(id)));
  } else if (cmd == "GAIN") {
    int sp2 = line.indexOf(' ', sp1 + 1);
    int sp3 = line.indexOf(' ', sp2 + 1);
    int sp4 = line.indexOf(' ', sp3 + 1);
    uint8_t id = (uint8_t)line.substring(sp1 + 1, sp2).toInt();
    int32_t p = line.substring(sp2 + 1, sp3).toInt();
    int32_t i = line.substring(sp3 + 1, sp4).toInt();
    int32_t d = line.substring(sp4 + 1).toInt();
    bool ok = sp2 != -1 && sp3 != -1 && sp4 != -1
           && dxl.writeControlTableItem(POSITION_P_GAIN, id, p)
           && dxl.writeControlTableItem(POSITION_I_GAIN, id, i)
           && dxl.writeControlTableItem(POSITION_D_GAIN, id, d);
    DEBUG_SERIAL.println(ok ? ("OK GAIN " + String(id) + " " + String(p) + " " + String(i) + " " + String(d))
                             : ("ERR GAIN " + String(id)));
  } else if (cmd == "JOG") {
    int sp2 = line.indexOf(' ', sp1 + 1);
    int sp3 = line.indexOf(' ', sp2 + 1);
    uint8_t id = (uint8_t)line.substring(sp1 + 1, sp2).toInt();
    int32_t delta = line.substring(sp2 + 1, sp3).toInt();
    int32_t lead = line.substring(sp3 + 1).toInt();
    int32_t present = (int32_t)dxl.getPresentPosition(id);
    int32_t goal = dxl.readControlTableItem(GOAL_POSITION, id) + delta;
    if (goal > present + lead) goal = present + lead;
    if (goal < present - lead) goal = present - lead;
    bool ok = sp2 != -1 && sp3 != -1 && dxl.setGoalPosition(id, goal);
    DEBUG_SERIAL.println(ok ? ("OK JOG " + String(id) + " " + String(goal))
                             : ("ERR JOG " + String(id)));
  } else if (cmd == "HOLD") {
    uint8_t id = (uint8_t)line.substring(sp1 + 1).toInt();
    int32_t present = (int32_t)dxl.getPresentPosition(id);
    bool ok = dxl.setGoalPosition(id, present);
    DEBUG_SERIAL.println(ok ? ("OK HOLD " + String(id) + " " + String(present))
                             : ("ERR HOLD " + String(id)));
  } else if (cmd == "REBOOT") {
    uint8_t id = (uint8_t)line.substring(sp1 + 1).toInt();
    bool ok = dxl.reboot(id);
    DEBUG_SERIAL.println(ok ? ("OK REBOOT " + String(id)) : ("ERR REBOOT " + String(id)));
  } else if (cmd == "HWERR") {
    uint8_t id = (uint8_t)line.substring(sp1 + 1).toInt();
    DEBUG_SERIAL.print("OK HWERR ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(" ");
    DEBUG_SERIAL.println(dxl.readControlTableItem(HARDWARE_ERROR_STATUS, id));
  } else if (cmd == "DIAG") {
    uint8_t id = (uint8_t)line.substring(sp1 + 1).toInt();
    DEBUG_SERIAL.print("OK DIAG ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.print(" volt=");  DEBUG_SERIAL.print(dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, id));
    DEBUG_SERIAL.print(" temp=");  DEBUG_SERIAL.print(dxl.readControlTableItem(PRESENT_TEMPERATURE, id));
    DEBUG_SERIAL.print(" hwerr="); DEBUG_SERIAL.print(dxl.readControlTableItem(HARDWARE_ERROR_STATUS, id));
    DEBUG_SERIAL.print(" mode=");  DEBUG_SERIAL.print(dxl.readControlTableItem(OPERATING_MODE, id));
    DEBUG_SERIAL.print(" curlim=");DEBUG_SERIAL.print(dxl.readControlTableItem(CURRENT_LIMIT, id));
    DEBUG_SERIAL.print(" pwmlim=");DEBUG_SERIAL.print(dxl.readControlTableItem(PWM_LIMIT, id));
    DEBUG_SERIAL.print(" P=");     DEBUG_SERIAL.print(dxl.readControlTableItem(POSITION_P_GAIN, id));
    DEBUG_SERIAL.print(" I=");     DEBUG_SERIAL.print(dxl.readControlTableItem(POSITION_I_GAIN, id));
    DEBUG_SERIAL.print(" D=");     DEBUG_SERIAL.print(dxl.readControlTableItem(POSITION_D_GAIN, id));
    DEBUG_SERIAL.print(" pvel=");  DEBUG_SERIAL.print(dxl.readControlTableItem(PROFILE_VELOCITY, id));
    DEBUG_SERIAL.print(" pacc=");  DEBUG_SERIAL.print(dxl.readControlTableItem(PROFILE_ACCELERATION, id));
    DEBUG_SERIAL.print(" cur=");   DEBUG_SERIAL.println((int16_t)dxl.readControlTableItem(PRESENT_CURRENT, id));
  } else {
    DEBUG_SERIAL.println("ERR UNKNOWN_CMD");
  }
}

void doScan() {
  DEBUG_SERIAL.println("SCAN_START");
  for (long baud : SCAN_BAUDRATES) {
    dxl.begin(baud);
    for (uint8_t id = SCAN_ID_MIN; id <= SCAN_ID_MAX; id++) {
      if (dxl.ping(id)) {
        DEBUG_SERIAL.print("FOUND ID ");
        DEBUG_SERIAL.print(id);
        DEBUG_SERIAL.print(" BAUD ");
        DEBUG_SERIAL.print(baud);
        DEBUG_SERIAL.print(" MODEL ");
        DEBUG_SERIAL.println(dxl.getModelNumber(id));
      }
    }
  }
  dxl.begin(currentBaud);
  DEBUG_SERIAL.println("SCAN_END");
}
