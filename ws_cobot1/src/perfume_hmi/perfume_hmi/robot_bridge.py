"""두산 협동로봇 M0609 HMI-로봇 브릿지 (ROS2, 모니터링 + 정지/홈 복귀 전용).

관리자 모니터링/대시보드 패키지로, 로봇 연결 상태·조인트 각도·그리퍼 상태·TCP 힘·
로그(에러뿐 아니라 INFO/WARN 이벤트도 포함)를 보여주고 로봇 정지·홈 복귀만 수행한다. 손님 주문(조향 시작)
요청은 이 워크스페이스가 아니라 로봇팔 제어부 쪽에서 별도로 개발 중인
패키지(cobot_control)가 직접 받는다.
(perfume_order_srv/Order.srv 클라이언트는 키오스크, 로봇팔 제어부 패키지에서 사용).

[역할]
- ROS2 토픽 구독으로 조인트 각도/에러/연결 끊김 이벤트를 모아 admin 화면에
  보여줄 상태 스냅샷(get_status())을 만든다.
- 그리퍼 상태(컨트롤박스 디지털 출력)와 TCP 힘/토크는 로봇 드라이버 서비스를
  주기적으로 직접 조회한다 — cobot_control과 무관하게 로봇에서 바로 받아오는
  값이라, 그쪽 패키지가 아직 미완성이어도 동작한다.
- 로봇 정지(stop_robot)는 /stop_perfume 신호 발행뿐이다 — 실제 정지(move_stop
  호출 + 시퀀스 중단)는 cobot_control이 수행한다. 정지 로직을 한 곳에 모은
  이유와 트레이드오프는 STOP_PERFUME_TOPIC 주석 참고.
- 두산 표준 서비스로 홈 복귀(move_home)를 수행한다.
  일시정지/재개(move_pause/move_resume)는 제공하지 않는다 — 두산 ROS2
  드라이버가 이 두 서비스를 모션 서비스(move_joint 등)와 같은
  MutuallyExclusive 콜백 그룹에 등록해 둬서, 정작 모션이 진행 중일 때는
  (movej_cb가 그룹을 점유하고 있어) 일시정지 요청이 모션이 끝날 때까지 응답을
  못 받는다. 즉 필요한 순간에 구조적으로 동작하지 않아 실로봇 확인 후 제거했다.
  (move_stop만 드라이버가 별도 콜백 그룹(cb_group_)에 등록해 둬서 모션 중에도
  동작한다 — cobot_control의 정지 구현이 이 성질에 기대고 있다.)
- cobot_control의 제조 완료/실패 신호(/perfume_done, Bool)를 구독해서,
  false(제조 실패)를 받으면 자동으로 로봇을 정지시킨다.
- cobot_control의 제조 공정 단계 신호(/perfume_status, Int32)를 구독해서
  관리자 화면 '제조 현황'(현재 단계/경과 시간/최근 제조 이력)을 채운다 —
  코드↔단계 이름 매핑은 STATUS_NAMES 규약(모듈 상단) 참고. 성공/실패로
  마감된 제조는 1건 1행으로 SQLite(~/.perfume/hmi_history.db)에 저장해
  재시작 후에도 최근 이력이 남는다 (실시간 상태는 메모리 캐시만 사용).

[용어 주의 — 이 '정지'는 안전 정지/비상 정지가 아니다]
여기서 말하는 정지는 ROS2 토픽 신호(/stop_perfume)와 그걸 받은 cobot_control의
move_stop 호출, 즉 이더넷 경유의 일반 소프트웨어 명령이다. 안전
정지(STO/SS1/SS2)나 비상 정지(E-Stop)는 안전 등급 하드와이어 전기
신호(컨트롤러 안전 입력 단자, 물리 비상정지 버튼)로만 발동되며, 네트워크
경로로는 불가능하다. 네트워크/드라이버/cobot_control이 죽으면 이 정지도
동작하지 않으므로 물리 비상정지 버튼을 대체할 수 없다 — 그래서 이 코드에서는
'비상 정지'가 아니라 그냥 '정지'라고 부른다.

[구조]
ROS2 통신 상태(노드/클라이언트/상태 캐시)는 전부 RobotBridge 클래스에
캡슐화되어 있다 — perfume_kiosk의 RobotOrderClient와 같은 패턴으로, Flask 앱
시작 시(app.py main()) 인스턴스를 하나만 만들어 서버가 켜져 있는 동안
재사용한다. 두산 서비스 경로·DO 인덱스 같은 상수는 cobot_control과 맞춰야
하는 규약이라 눈에 잘 띄게 모듈 레벨에 그대로 둔다.

[스레드 구조]
Flask(werkzeug) 개발 서버는 동기식이라 ROS2 이벤트 루프(spin)를 요청
스레드에서 직접 돌릴 수 없다. 그래서:
- RobotBridge() 생성 시 ROS2 노드를 만들고, 전용 백그라운드 스레드에서 계속
  spin 시켜 서비스 응답 콜백이 처리되게 한다.
- stop_robot()/move_home() 등은 call_async로 요청만 보내고, future.done()이
  True가 될 때까지 짧게 sleep하며 폴링한다.
"""
import math
import os
import sqlite3
import threading
import time
from collections import deque

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32

from dsr_msgs2.msg import RobotError, RobotDisconnection
from dsr_msgs2.srv import (
    MoveJoint,
    GetToolForce,
    GetCtrlBoxDigitalOutput,
    SetCtrlBoxDigitalOutput,
    GetRobotSpeedMode,
    SetRobotSpeedMode,
)
# 로봇 쪽 노드 일치해야 하는 상수

ROBOT_NAMESPACE = "dsr01"                # 두산 로봇 네임스페이스

# 관리자 HMI용 두산 표준 토픽/서비스 이름 변수 참조하게 만들어서 재사용. (다른 팀 공유해줄때는 로봇 네임스페이스 바꿔서 쓸 수 있게 전달)
# 조인트 상태는 드라이버(joint_state_broadcaster)가 발행하는 /dsr01/joint_states만
# 구독한다. 루트 /joint_states는 bringup의 joint_state_publisher가 시각화용으로
# 발행하는 토픽인데, 드라이버가 죽거나 로봇이 연결 안 돼 있어도 URDF 기본값(0)으로
# 계속 발행되기 때문에 연결 판정에 쓰면 '로봇 연결됨'으로 오판한다.
JOINT_STATES_TOPIC = f"/{ROBOT_NAMESPACE}/joint_states"
ROBOT_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
ROBOT_ERROR_TOPIC = f"/{ROBOT_NAMESPACE}/error"
ROBOT_DISCONNECTION_TOPIC = f"/{ROBOT_NAMESPACE}/robot_disconnection"
MOVE_JOINT_SERVICE = f"/{ROBOT_NAMESPACE}/motion/move_joint"

# 제조 완료/실패 신호 토픽 (std_msgs/Bool) — cobot_control이 발행, kiosk와 HMI가
# 함께 구독한다. true=제조 성공, false=제조 실패. false를 받으면 HMI가 자동으로
# 로봇 정지(stop_robot)를 건다 — cobot_control 쪽은 실패 시 false를 "모션을 더 보내기
# 전에" 발행해야 이 자동 정지가 의미가 있다 (kiosk의 ORDER_DONE_TOPIC과 동일해야 함).
ORDER_DONE_TOPIC = "/perfume_done"

# 키오스크 잠금 상태 토픽 (std_msgs/Bool) — HMI가 발행, kiosk가 구독한다
# (kiosk의 KIOSK_LOCK_TOPIC과 동일해야 함). true=잠금(점검 중), false=해제.
# 잠금 상태의 단일 소스는 HMI(app.py의 kiosk_locked)이고, 이 토픽은 그 값을
# 래치(transient_local QoS)로 흘려보내는 통로다 — kiosk가 나중에 켜져도
# 마지막 잠금 상태를 바로 받는다. 예전에는 kiosk가 HTTP(GET /internal/lock_status)로
# 폴링했는데, HMI PC의 IP를 환경변수로 맞춰줘야 해서 다른 채널(/order_perfume,
# /perfume_done)처럼 DDS 자동 발견을 쓰는 토픽으로 교체했다.
KIOSK_LOCK_TOPIC = "/kiosk_locked"


# 제조 공정 단계 토픽 (std_msgs/Int32) — cobot_control이 공정 단계가 바뀔 때마다
# 아래 STATUS_* 코드를 발행한다. 숫자 코드↔단계 이름 매핑은 ROS 인터페이스가 아니라
# 애플리케이션 규약이라, cobot_control 쪽 상수 블록을 그대로 복사해 값을 맞춘다
# (변경 시 양쪽 동시 수정 필수). HMI는 이 코드로 관리자 화면의 '제조 현황'을 채운다.
PERFUME_STATUS_TOPIC = "/perfume_status"

STATUS_IDLE = 0
STATUS_ORDER_RECEIVED = 10
STATUS_PROCESS_START = 20
STATUS_MOVE_TO_PERFUME = 30
STATUS_CHECK_PERFUME = 40
STATUS_OPEN_PERFUME_LID = 50
STATUS_STORE_PERFUME_LID = 60
STATUS_SCENT_PROCESS_START = 100
STATUS_MOVE_TO_SCENT = 110
STATUS_EXTRACT_SCENT = 120
STATUS_GRIP_SCENT_LID = 130
STATUS_OPEN_SCENT_LID = 140
STATUS_MOVE_TO_MIX_BOTTLE = 150
STATUS_DISPENSE_SCENT = 160
STATUS_GRIP_SCENT_LID_RETURN = 170
STATUS_RETURN_TO_SCENT = 180
STATUS_CLOSE_SCENT_LID = 190
STATUS_SCENT_PROCESS_DONE = 200
STATUS_GET_PERFUME_LID = 210
STATUS_MOVE_LID_TO_PERFUME = 220
STATUS_CLOSE_PERFUME_LID = 230
STATUS_GRIP_FINISHED_PERFUME = 240
STATUS_MOVE_TO_HOME = 250
STATUS_SHAKE_PERFUME = 260
STATUS_TILT_MIX_PERFUME = 270
STATUS_MOVE_TO_PICKUP = 280
STATUS_PLACE_PERFUME = 290
STATUS_RELEASE_PERFUME = 300
STATUS_PROCESS_COMPLETE = 310
STATUS_RETURN_HOME = 320
STATUS_READY = 330

STATUS_NAMES = {
    STATUS_IDLE: "IDLE",
    STATUS_ORDER_RECEIVED: "ORDER_RECEIVED",
    STATUS_PROCESS_START: "PROCESS_START",
    STATUS_MOVE_TO_PERFUME: "MOVE_TO_PERFUME",
    STATUS_CHECK_PERFUME: "CHECK_PERFUME",
    STATUS_OPEN_PERFUME_LID: "OPEN_PERFUME_LID",
    STATUS_STORE_PERFUME_LID: "STORE_PERFUME_LID",
    STATUS_SCENT_PROCESS_START: "SCENT_PROCESS_START",
    STATUS_MOVE_TO_SCENT: "MOVE_TO_SCENT",
    STATUS_EXTRACT_SCENT: "EXTRACT_SCENT",
    STATUS_GRIP_SCENT_LID: "GRIP_SCENT_LID",
    STATUS_OPEN_SCENT_LID: "OPEN_SCENT_LID",
    STATUS_MOVE_TO_MIX_BOTTLE: "MOVE_TO_MIX_BOTTLE",
    STATUS_DISPENSE_SCENT: "DISPENSE_SCENT",
    STATUS_GRIP_SCENT_LID_RETURN: "GRIP_SCENT_LID_RETURN",
    STATUS_RETURN_TO_SCENT: "RETURN_TO_SCENT",
    STATUS_CLOSE_SCENT_LID: "CLOSE_SCENT_LID",
    STATUS_SCENT_PROCESS_DONE: "SCENT_PROCESS_DONE",
    STATUS_GET_PERFUME_LID: "GET_PERFUME_LID",
    STATUS_MOVE_LID_TO_PERFUME: "MOVE_LID_TO_PERFUME",
    STATUS_CLOSE_PERFUME_LID: "CLOSE_PERFUME_LID",
    STATUS_GRIP_FINISHED_PERFUME: "GRIP_FINISHED_PERFUME",
    STATUS_MOVE_TO_HOME: "MOVE_TO_HOME",
    STATUS_SHAKE_PERFUME: "SHAKE_PERFUME",
    STATUS_TILT_MIX_PERFUME: "TILT_MIX_PERFUME",
    STATUS_MOVE_TO_PICKUP: "MOVE_TO_PICKUP",
    STATUS_PLACE_PERFUME: "PLACE_PERFUME",
    STATUS_RELEASE_PERFUME: "RELEASE_PERFUME",
    STATUS_PROCESS_COMPLETE: "PROCESS_COMPLETE",
    STATUS_RETURN_HOME: "RETURN_HOME",
    STATUS_READY: "READY",
}

# '제조 중'이 아닌 코드들 — IDLE/READY는 대기, PROCESS_COMPLETE(310)와 그 뒤의
# RETURN_HOME(320)은 향수가 이미 픽업대에 놓인 뒤라(RELEASE_PERFUME=300 이후)
# 제조로 치지 않는다. 이 집합에 없으면 전부 '제조 중(active)'.
MAKING_INACTIVE_CODES = {STATUS_IDLE, STATUS_PROCESS_COMPLETE, STATUS_RETURN_HOME, STATUS_READY}

# 제조 이력 DB — 성공/실패로 마감된 제조 1건당 1행만 저장한다 (단계 전이는 저장 안 함).
# 실시간 표시(제조 중/단계/경과)는 지금처럼 메모리 캐시로만 가고, DB는 재시작 후에도
# '최근 제조 이력'을 보여주기 위한 영속화 전용. kiosk(~/.perfume/kiosk.db)와 같은
# 컨벤션으로 소스 트리 밖에 둬서 colcon build를 다시 해도 지워지지 않는다.
MAKING_HISTORY_DB_DIR = os.path.expanduser("~/.perfume")
MAKING_HISTORY_DB_PATH = os.path.join(MAKING_HISTORY_DB_DIR, "hmi_history.db")
MAKING_HISTORY_LIMIT = 5         # 관리자 화면에 보여줄 최근 제조 이력 개수 (DB에는 전부 쌓인다)

# 제조 시퀀스 중단 신호 토픽 (std_msgs/Bool) — 정지 로직의 단일 경로.
# HMI는 이 신호를 발행만 하고, 실제 정지(진행 중 모션 move_stop + 시퀀스 중단)는
# 전부 cobot_control이 수행한다: true 수신 → 스스로 move_stop 호출 →
# KeyboardInterrupt로 시퀀스 중단 → perfume_done=false 발행 후 종료
# (검증된 참고 구현: cobot_control/main_thread.py의 stop_callback).
#
# (참고) 한때 HMI가 move_stop을 직접 호출했고(1회 + 0.2초 반복 가드까지),
# 실로봇 테스트에서 모션 1개는 끊겨도 cobot_control 시퀀스가 wait/그리퍼 I/O를
# 타고 계속 진행돼 정지 효과가 없었다. 어중간하게 끊긴 자세에서 다음 파지/토출
# 스텝이 이어지는 게 오히려 위험해, 정지 로직을 cobot_control 한 곳으로 모으고
# HMI 쪽 move_stop 호출은 전부 제거했다. 트레이드오프: cobot_control이 죽거나
# 행업된 상태에서는 이 신호로 로봇을 멈출 수 없다 — 그 경우의 최후 수단은
# 물리 비상정지 버튼이다 (아래 [용어 주의] 참고).
STOP_PERFUME_TOPIC = "/stop_perfume"

# 그리퍼 상태 / TCP 힘 조회용 — 로봇 드라이버가 직접 제공하는 서비스
# cobot_control의 grip()/release()가 컨트롤박스 디지털 출력 1/2/3번으로 그리퍼를
# 여닫으므로(set_digital_output), 같은 인덱스를 조회, 상태를 역산한다.
GET_TOOL_FORCE_SERVICE = f"/{ROBOT_NAMESPACE}/aux_control/get_tool_force"
GET_DIGITAL_OUTPUT_SERVICE = f"/{ROBOT_NAMESPACE}/io/get_ctrl_box_digital_output"
SET_DIGITAL_OUTPUT_SERVICE = f"/{ROBOT_NAMESPACE}/io/set_ctrl_box_digital_output"
GET_SPEED_MODE_SERVICE = f"/{ROBOT_NAMESPACE}/system/get_robot_speed_mode"
SET_SPEED_MODE_SERVICE = f"/{ROBOT_NAMESPACE}/system/set_robot_speed_mode"
# cobot_control의 grip()/release() DO 인덱스와 동일해야 함.
# 3번(RESERVED)은 아직 별도 의미가 없다 — cobot_control이 grip/release를 부를
# 때마다 1~3번을 전부 OFF로 방어적 리셋한 뒤 목표 조합을 쓰는데, 그 리셋
# 대상에 포함되는 핀이라 나중에 의미가 생길 걸 대비해 여기서도 같이 추적한다.
GRIPPER_DO_GRIP_INDEX, GRIPPER_DO_RELEASE_INDEX, GRIPPER_DO_RESERVED_INDEX = 1, 2, 3
# 주의: 두산 srv 파일 주석은 "0:ON, 1:OFF"라고 적혀 있지만 틀렸다 — 실제 규약은
# DSR_ROBOT2.py의 ON=1, OFF=0 (cobot_control이 이 상수로 set_digital_output을 부르고,
# 래퍼가 그 값을 그대로 서비스 요청에 넣는다). Get도 같은 값 체계로 돌아온다.
DO_ON, DO_OFF = 1, 0
TOOL_FORCE_REF_BASE = 0          # DSR_ROBOT2.DR_BASE — get_tool_force의 기준 좌표계
SPEED_MODE_NORMAL, SPEED_MODE_REDUCED = 0, 1  # 두산 speed_mode 값 (SPEED_NORMAL/REDUCED_MODE)
IO_TIMEOUT_SEC = 3.0             # 그리퍼 DO 쓰기/속도 모드 전환 응답 대기 한도(초)
EXTRA_STATE_POLL_SEC = 1.0       # 그리퍼/힘/속도 모드 값을 다시 물어보는 주기(초)

POLL_INTERVAL_SEC = 0.1          # 응답 도착 여부를 확인하는 폴링 주기(초)

JOINT_STALE_SEC = 2.0            # joint_states가 이 시간 이상 안 오면 "연결 끊김"으로 판단
ERROR_LOG_MAX = 20               # 관리자 화면에 보여줄 최근 로그 보관 개수 (에러 외 INFO/WARN도 포함)
HOME_POSJ = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]  # 프로젝트 공용 홈 자세 (rokey 코드와 동일)
HOME_VEL, HOME_ACC = 30.0, 30.0               # 홈 복귀 속도/가속 (안전하게 낮게)
MOVE_HOME_TIMEOUT_SEC = 60.0     # 홈 복귀 완료 응답 대기 한도(초)


def _wait_future(future, timeout_sec):
    """call_async future가 끝날 때까지 폴링 대기. 시간 초과 시 None 반환.

    call_async()는 요청만 보내고 즉시 future를 돌려주는 논블로킹 호출이라,
    실제 완료는 future.done()이 True가 될 때까지 기다려야 한다. asyncio 같은
    진짜 비동기 대신 짧게(POLL_INTERVAL_SEC) sleep하며 폴링하는 이유는, 이걸
    부르는 Flask 요청 스레드 입장에서는 "끝날 때까지 단순 블로킹"하는 형태로
    맞추는 게 가장 간단하기 때문이다 — stop_robot()/move_home() 둘 다 이 함수의
    반환을 기다렸다가 그대로 HTTP 응답으로 돌려준다.
    """
    waited = 0.0
    while not future.done():
        time.sleep(POLL_INTERVAL_SEC)
        waited += POLL_INTERVAL_SEC
        if waited >= timeout_sec:
            future.cancel()
            return None
    return future.result()


def _gripper_label(do1, do2):
    """cobot_control의 grip()/release() DO 조합을 역산해서 그리퍼 상태를 추정."""
    if do1 == DO_ON and do2 == DO_OFF:
        return "grip"
    if do1 == DO_OFF and do2 == DO_ON:
        return "release"
    return "unknown"


class RobotBridge:
    """ROS2 통신(모니터링 + 제어)을 한 덩어리로 묶은 브릿지.

    생성 시 ROS2를 초기화하고 서비스 클라이언트/구독/폴링 타이머를 만들어
    백그라운드 spin 스레드를 시작한다. Flask 앱 시작 시(app.py main())
    하나만 만들어 재사용하고, 종료 시 shutdown()으로 정리한다.
    """

    def __init__(self):
        rclpy.init()
        self._node = Node("perfume_hmi_client")

        # 관리자 HMI용 제어 클라이언트 (홈 복귀)
        # 로봇 정지는 서비스 호출이 아니라 /stop_perfume 발행뿐이라 클라이언트가 없다
        # (정지 로직은 cobot_control 한 곳에 모음 — STOP_PERFUME_TOPIC 주석 참고).
        # 홈 복귀용 move_joint 서비스 (SYNC 모션)
        self._move_joint_client = self._node.create_client(MoveJoint, MOVE_JOINT_SERVICE)

        # 그리퍼 상태 / TCP 힘 / 속도 모드 조회 클라이언트 (로봇 드라이버 직접 조회, cobot_control 불필요)
        self._get_tool_force_client = self._node.create_client(GetToolForce, GET_TOOL_FORCE_SERVICE)
        self._get_digital_output_client = self._node.create_client(GetCtrlBoxDigitalOutput, GET_DIGITAL_OUTPUT_SERVICE)
        self._get_speed_mode_client = self._node.create_client(GetRobotSpeedMode, GET_SPEED_MODE_SERVICE)

        # 관리자 제어용 쓰기 클라이언트 (그리퍼 수동 개폐 / 속도 모드 전환)
        self._set_digital_output_client = self._node.create_client(SetCtrlBoxDigitalOutput, SET_DIGITAL_OUTPUT_SERVICE)
        self._set_speed_mode_client = self._node.create_client(SetRobotSpeedMode, SET_SPEED_MODE_SERVICE)

        # ---- 관리자 HMI용 상태 저장소 ----
        # ROS2 콜백 스레드와 Flask 요청 스레드가 동시에 접근하므로 락으로 보호한다.
        self._status_lock = threading.Lock()
        self._robot_status = {
            "joints_deg": [0.0] * 6,   # 현재 조인트 각도 [deg]
            "last_joint_time": 0.0,    # joint_states 마지막 수신 시각 (연결 판단용)
            "do1": None,               # 컨트롤박스 디지털 출력 1번 (1=ON/0=OFF, 아직 못 읽었으면 None)
            "do2": None,               # 컨트롤박스 디지털 출력 2번
            "do3": None,               # 컨트롤박스 디지털 출력 3번 (예비 — 현재는 항상 OFF)
            "tool_force": [0.0] * 6,   # 최근 TCP 힘/토크 실측값 [Fx,Fy,Fz,Mx,My,Mz] (DR_BASE 기준)
            "speed_mode": None,        # 0=일반/1=감속, 아직 못 읽었으면 None
            "making_code": STATUS_IDLE,   # /perfume_status 최근 공정 단계 코드
            "making_started_at": 0.0,     # 이번 제조 시작 시각 (경과 시간 계산용)
        }
        self._error_log = deque(maxlen=ERROR_LOG_MAX)  # 최근 에러 목록 (최신이 앞) 20개 최대

        # 최근 제조 이력 (최신이 앞) — 시작 시 SQLite에서 로드해 재시작에도 유지된다.
        # 이후 마감(_record_making_result)마다 메모리와 DB에 함께 기록한다.
        self._making_history = deque(self._load_making_history(), maxlen=MAKING_HISTORY_LIMIT)

        # 관리자 HMI용 모니터링 구독 (조인트 상태 / 에러 / 연결 끊김 이벤트)
        self._node.create_subscription(JointState, JOINT_STATES_TOPIC, self._joint_states_callback, 10)
        self._node.create_subscription(RobotError, ROBOT_ERROR_TOPIC, self._robot_error_callback, 10)
        # robot_disconnection 이벤트는 ROS2에서 한 번만 발생하고, 재연결 시점은 알 수 없으므로
        # 연결 여부 판단은 joint_states 수신 시각으로 한다. 다만 연결 끊김 이벤트는 로그에 기록해서 관리자가 볼 수 있게 한다.
        self._node.create_subscription(
            RobotDisconnection, ROBOT_DISCONNECTION_TOPIC, self._robot_disconnection_callback, 10
        )
        # 제조 완료/실패 신호 — false(제조 실패)를 받으면 자동으로 로봇을 정지시킨다.
        self._node.create_subscription(Bool, ORDER_DONE_TOPIC, self._order_done_callback, 10)
        # 제조 공정 단계 신호 — 관리자 화면 '제조 현황'의 소스 (STATUS_* 규약 참고).
        self._node.create_subscription(Int32, PERFUME_STATUS_TOPIC, self._perfume_status_callback, 10)
        # 제조 시퀀스 중단 신호 발행용 — stop_robot()의 유일한 동작이다.
        self._stop_perfume_pub = self._node.create_publisher(Bool, STOP_PERFUME_TOPIC, 10)

        # 키오스크 잠금 상태 발행용 — 래치(transient_local) QoS라 늦게 켜진 kiosk도
        # 마지막 값을 바로 받는다. HMI 재시작 시 잠금이 False로 초기화되므로
        # (app.py의 kiosk_locked 전역과 동일한 규약) 시작하자마자 False를 한 번
        # 발행해서 래치 값과 전역 값이 어긋나지 않게 한다.
        lock_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._kiosk_lock_pub = self._node.create_publisher(Bool, KIOSK_LOCK_TOPIC, lock_qos)
        self.publish_kiosk_lock(False)

        # 그리퍼/힘 값은 토픽이 아니라 서비스라 push로 오지 않는다 — 타이머로 주기적으로
        # call_async를 쏘고, 응답이 오면 add_done_callback으로 캐시만 갱신한다. get_status()는
        # 여전히 캐시만 읽으므로 Flask 요청 스레드가 서비스 응답을 기다리며 막히지 않는다.
        self._extra_state_timer = self._node.create_timer(EXTRA_STATE_POLL_SEC, self._poll_extra_state)

        # MultiThreadedExecutor + daemon thread: 메인(Flask) 스레드를 막지 않고
        # ROS2 콜백(서비스 응답 등)을 백그라운드에서 계속 처리하기 위함.
        # executor.spin()은 블로킹 호출이라 메인 스레드에서 그냥 부르면 Flask가
        # 멈춘다 — 그래서 별도 스레드에 맡기고 생성자는 바로 리턴한다. 이 스레드가
        # 계속 돌면서 위에서 등록한 구독 콜백(_joint_states_callback 등)과
        # stop_robot()/move_home()이 보낸 서비스 요청의 응답을 처리한다.
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self._node.get_logger().info("[perfume_hmi] ROS2 준비 완료 (모니터링 + stop/home)")

    def shutdown(self):
        """Flask 앱 종료 시 ROS2 자원을 정리한다."""
        self._executor.shutdown()
        # spin 스레드가 완전히 끝난 뒤에 rclpy.shutdown()을 불러야 한다.
        # 순서를 지키지 않으면 인터프리터 종료 시 세그폴트가 날 수 있다.
        self._spin_thread.join(timeout=2.0)
        self._node.destroy_node()
        rclpy.shutdown()

    # ==============================================================
    # 모니터링 콜백 (ROS2 spin 스레드에서 실행 — 캐시 갱신만 한다)
    # ==============================================================

    def _joint_states_callback(self, msg):
        """로봇 조인트 상태 수신 — 각도 저장 + 수신 시각 기록(연결 판단 근거).

        그리퍼 조인트 등이 섞여 와도 되도록, 이름으로 로봇 6축만 골라낸다.
        """
        pos_by_name = dict(zip(msg.name, msg.position))
        if not all(n in pos_by_name for n in ROBOT_JOINT_NAMES):
            return  # 로봇 6축이 없는 메시지(그리퍼 단독 등)는 무시
        with self._status_lock:
            self._robot_status["joints_deg"] = [
                round(math.degrees(pos_by_name[n]), 1) for n in ROBOT_JOINT_NAMES
            ]
            self._robot_status["last_joint_time"] = time.time()

    def _robot_error_callback(self, msg):
        """로봇 에러/경고 발생 시 최근 에러 목록에 추가 (최신이 앞)."""
        level_str = {1: "INFO", 2: "WARN", 3: "ERROR"}.get(msg.level, str(msg.level))
        with self._status_lock:
            self._error_log.appendleft({
                "time": time.strftime("%H:%M:%S"),
                "level": level_str,
                "code": msg.code,
                "message": msg.msg1 or f"group={msg.group} code={msg.code}",
            })

    def _robot_disconnection_callback(self, msg):
        """로봇 연결 끊김 이벤트 — 에러 목록에 함께 기록해서 관리자가 볼 수 있게."""
        with self._status_lock:
            self._error_log.appendleft({
                "time": time.strftime("%H:%M:%S"),
                "level": "ERROR",
                "code": 0,
                "message": "로봇 연결이 끊어졌습니다 (robot_disconnection)",
            })



    def _perfume_status_callback(self, msg):
        """제조 공정 단계 수신 — 코드 저장 + 시작/완료 전이 처리.

        cobot_control은 단계가 바뀔 때만 발행한다(주기 발행 아님). 그래서 경과
        시간은 여기서 받은 값이 아니라, '대기→제조 중' 전이 시각을 기록해 두고
        get_status()가 매번 현재 시각과의 차이로 계산한다.
        """
        code = msg.data
        started = False
        completed = False
        with self._status_lock:
            # 제조 중(active) 상태 전이 감지 — MAKING_INACTIVE_CODES에 없으면 active, 있으면 inactive.
            prev_active = self._robot_status["making_code"] not in MAKING_INACTIVE_CODES
            # 이번 제조 단계가 active인지 판단
            active = code not in MAKING_INACTIVE_CODES
            # 제조 단계 코드 갱신
            self._robot_status["making_code"] = code
            # 제조 시작 시각 기록
            if active and not prev_active:
                self._robot_status["making_started_at"] = time.time()
                started = True
            # 제조 완료 전이 감지 — active 상태에서 완료 계열 코드(PROCESS_COMPLETE/
            # RETURN_HOME/READY)로 넘어오면 '성공'으로 마감한다. 310 하나만 기다리면
            # cobot_control이 310을 건너뛰거나 HMI가 그 메시지 하나를 놓친 경우
            # 이력이 통째로 누락된다 (이후 perfume_done=true 보정도 이미 inactive라
            # 통과 못 함). IDLE(0)로의 전이는 초기화/이상 종료 쪽 신호일 수 있어
            # 성공으로 기록하지 않는다 — 실패 마감은 perfume_done=false 콜백 담당.
            if prev_active and not active and code != STATUS_IDLE:
                completed = True
        # 이력 기록은 DB 쓰기가 있어서 락 밖에서 (락은 캐시 갱신용으로만 짧게 쥔다)
        if started:
            self._node.get_logger().info(
                f"[making] 제조 시작 감지 (code={code} {STATUS_NAMES.get(code, '?')})")
        if completed:
            self._record_making_result("success", code)

    # 제조 실패 시 자동 정지 시나리오
    def _order_done_callback(self, msg):
        """제조 완료/실패 신호 — false(제조 실패)면 자동으로 로봇을 정지시킨다.

        stop_robot()은 서비스 응답까지 몇 초 블로킹될 수 있어서, 구독 콜백(spin
        스레드)을 붙잡지 않도록 별도 데몬 스레드에서 실행한다. true(성공)는
        kiosk가 완료 화면 전환에 쓰는 신호라 HMI는 아무것도 하지 않는다.
        """
        if msg.data:
            # 제조 성공 — kiosk용 완료 신호. 보통 PROCESS_COMPLETE(310)가 제조 현황을
            # 이미 마감했지만, 그 메시지를 놓친 경우를 대비해 아직 '제조 중'이면
            # 여기서 성공으로 마감한다 (310이 처리됐으면 inactive라 아무것도 안 함).
            finish_step = None
            with self._status_lock:
                if self._robot_status["making_code"] not in MAKING_INACTIVE_CODES:
                    finish_step = self._robot_status["making_code"]
                    self._robot_status["making_code"] = STATUS_IDLE
            if finish_step is not None:
                self._record_making_result("success", finish_step)
            return

        self._node.get_logger().error("[auto] 제조 실패 신호(perfume_done=false) 수신 — 자동 정지 실행")
        # 자동 정지 시도 전, 로그에 먼저 기록해서 관리자가 볼 수 있게 한다.
        # 실패하면 cobot_control 프로세스가 종료돼 /perfume_status가 더 안 오므로,
        # 제조 현황도 여기서 '실패'로 마감한다.
        with self._status_lock:
            self._error_log.appendleft({
                "time": time.strftime("%H:%M:%S"),
                "level": "ERROR",
                "code": 0,
                "message": "제조 실패 신호 수신 — 자동 정지를 실행합니다",
            })
            # 실패 시점의 공정 단계를 이력에 남긴다 — "어느 단계에서 실패했나" 진단용
            fail_step = self._robot_status["making_code"]
            self._robot_status["making_code"] = STATUS_IDLE
        self._record_making_result("fail", fail_step)
        # 자동 정지는 stop_robot()이 서비스 응답까지 블로킹될 수 있으므로, spin 스레드를 잡지 않도록 별도 데몬 스레드에서 실행한다.
        threading.Thread(target=self._auto_stop, daemon=True).start()

    def _auto_stop(self):
        """제조 실패에 의한 자동 정지 — 결과를 로그에 남겨 관리자가 볼 수 있게 한다."""
        result = self.stop_robot()
        ok = result["status"] == "success"
        with self._status_lock:
            self._error_log.appendleft({
                "time": time.strftime("%H:%M:%S"),
                "level": "WARN" if ok else "ERROR",
                "code": 0,
                "message": ("자동 정지 완료 — 현장 확인 후 재가동하세요"
                            if ok else f"자동 정지 실패: {result['message']}"),
            })

    # ==============================================================
    # 제조 이력 (SQLite 영속화 — 마감된 제조 1건당 1행)
    # ==============================================================
    # 실시간 상태(제조 중/단계/경과)는 메모리 캐시로만 가고, 여기는 성공/실패로
    # 마감된 결과만 저장한다. sqlite3 연결은 스레드 간 공유하면 안 되므로
    # (쓰기: ROS2 spin 스레드 / 읽기: 시작 시 메인 스레드) 매번 짧게 열고 닫는다
    # — 쓰기가 제조당 1회뿐이라 부담 없다. DB가 고장 나도 화면 표시(메모리)는
    # 계속 동작하도록 모든 DB 접근은 실패해도 경고 로그만 남기고 넘어간다.

    def _load_making_history(self):
        """시작 시 DB에서 최근 이력을 읽는다 (최신이 앞). 실패하면 빈 목록."""
        try:
            os.makedirs(MAKING_HISTORY_DB_DIR, exist_ok=True)
            conn = sqlite3.connect(MAKING_HISTORY_DB_PATH)
            try:
                conn.execute(
                    "CREATE TABLE IF NOT EXISTS making_history ("
                    "  id INTEGER PRIMARY KEY AUTOINCREMENT,"
                    "  finished_at TEXT NOT NULL,"
                    "  result TEXT NOT NULL,"
                    "  duration_sec INTEGER,"
                    "  last_step INTEGER)"
                )
                conn.commit()
                rows = conn.execute(
                    "SELECT finished_at, result, duration_sec, last_step "
                    "FROM making_history ORDER BY id DESC LIMIT ?",
                    (MAKING_HISTORY_LIMIT,),
                ).fetchall()
            finally:
                conn.close()
        except Exception as e:
            self._node.get_logger().warn(f"[history] 제조 이력 DB 로드 실패 (이력 없이 시작): {e}")
            return []
        return [
            {
                "finished_at": r[0],
                "result": r[1],
                "duration_sec": r[2],
                "last_step": r[3],
                "last_step_name": STATUS_NAMES.get(r[3], str(r[3])),
            }
            for r in rows
        ]

    def _record_making_result(self, result, last_step):
        """제조 1건 마감 기록 — 메모리(최근 N건)와 SQLite에 함께 남긴다.

        호출 지점 3곳: PROCESS_COMPLETE(310) 수신, perfume_done=true 보정 마감,
        perfume_done=false 실패 마감. last_step은 마감 시점의 공정 단계 코드
        (실패면 "어느 단계에서 실패했나"가 된다).
        """
        now = time.time()
        with self._status_lock:
            started = self._robot_status["making_started_at"]
        record = {
            "finished_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            "result": result,
            # 시작 전이를 못 본 경우(예: HMI가 제조 도중 재시작) 소요 시간은 알 수 없다
            "duration_sec": int(now - started) if started else None,
            "last_step": last_step,
            "last_step_name": STATUS_NAMES.get(last_step, str(last_step)),
        }
        with self._status_lock:
            self._making_history.appendleft(record)
        self._node.get_logger().info(
            f"[history] 제조 이력 기록: {result} "
            f"(last_step={record['last_step_name']}, duration={record['duration_sec']}s)")
        try:
            conn = sqlite3.connect(MAKING_HISTORY_DB_PATH)
            try:
                conn.execute(
                    "INSERT INTO making_history (finished_at, result, duration_sec, last_step) "
                    "VALUES (?, ?, ?, ?)",
                    (record["finished_at"], result, record["duration_sec"], last_step),
                )
                conn.commit()
            finally:
                conn.close()
        except Exception as e:
            self._node.get_logger().warn(f"[history] 제조 이력 DB 저장 실패 (화면 표시는 유지): {e}")

    def _poll_extra_state(self):
        """그리퍼 DO / TCP 힘 / 속도 모드 값을 주기적으로 다시 물어본다 (타이머 콜백).

        joint_states처럼 토픽으로 밀려오는 값이 아니라 서비스라, 여기서 직접
        call_async를 쏘고 응답은 콜백(_tool_force_done_cb 등)에서 캐시에 반영한다.
        서비스가 아직 안 떠 있으면(로봇 드라이버가 I/O 서비스를 안 켰거나 시뮬레이션
        환경 등) 조용히 건너뛴다 — 다음 주기에 다시 시도.
        """
        if self._get_tool_force_client.service_is_ready():
            request = GetToolForce.Request()
            request.ref = TOOL_FORCE_REF_BASE
            self._get_tool_force_client.call_async(request).add_done_callback(self._tool_force_done_cb)

        if self._get_digital_output_client.service_is_ready():
            for index in (GRIPPER_DO_GRIP_INDEX, GRIPPER_DO_RELEASE_INDEX, GRIPPER_DO_RESERVED_INDEX):
                request = GetCtrlBoxDigitalOutput.Request()
                request.index = index
                self._get_digital_output_client.call_async(request).add_done_callback(
                    lambda future, i=index: self._digital_output_done_cb(i, future)
                )

        if self._get_speed_mode_client.service_is_ready():
            self._get_speed_mode_client.call_async(GetRobotSpeedMode.Request()).add_done_callback(
                self._speed_mode_done_cb
            )

    """ TCP 힘/토크 조회 응답 콜백 """
    def _tool_force_done_cb(self, future): 
        try:
            result = future.result()
        except Exception:
            return  # 타임아웃/연결 끊김 등 — 다음 폴링 주기에 재시도되므로 조용히 무시
        if result is None or not result.success:
            return
        with self._status_lock:
            self._robot_status["tool_force"] = [round(v, 2) for v in result.tool_force]

    """ 컨트롤박스 디지털 출력 조회 응답 콜백 """
    def _digital_output_done_cb(self, index, future): 
        try:
            result = future.result()
        except Exception:
            return
        if result is None or not result.success:
            return
        with self._status_lock:
            if index == GRIPPER_DO_GRIP_INDEX:
                self._robot_status["do1"] = result.value
            elif index == GRIPPER_DO_RELEASE_INDEX:
                self._robot_status["do2"] = result.value
            elif index == GRIPPER_DO_RESERVED_INDEX:
                self._robot_status["do3"] = result.value

    """ 로봇 속도 모드 조회 응답 콜백 """
    def _speed_mode_done_cb(self, future):
        try:
            result = future.result()
        except Exception:
            return
        if result is None or not result.success:
            return
        with self._status_lock:
            self._robot_status["speed_mode"] = result.speed_mode

    # ==============================================================
    # 상태 조회 (Flask 요청 스레드에서 호출 — 캐시 스냅샷만 읽는다)
    # ==============================================================

    def get_status(self):
        """관리자 화면이 1초마다 폴링하는 상태 스냅샷.

        연결 여부는 joint_states 스트림이 최근에 들어왔는지로 판단한다.
        (연결 끊김 '이벤트' 토픽만으로는 재연결 시점을 알 수 없기 때문).
        두산 공식 기능은 RobotDisconnection (연결 끊김)만 제공하고, 재연결 시점은 알 수 없다.
        RobotDisconnection은 로그에 기록해서 관리자가 볼 수 있는 용으로 활용.

        제조 현황(making)은 cobot_control이 발행하는 /perfume_status(Int32) 공정
        단계 코드로 채운다 — 코드↔이름 매핑은 STATUS_NAMES 규약(모듈 상단) 참고.
        단계가 바뀔 때만 발행되는 이벤트성 토픽이라, 경과 시간은 시작 전이 시각
        (making_started_at)부터 여기서 매번 계산한다. 향료별 샷 계획(plan)은 이
        토픽에 없어서 표시하지 않는다 — kiosk→cobot_control의 Order.srv에만 실린다.

        그리퍼 상태/TCP 힘은 로봇 드라이버 서비스를 직접 조회해 얻으므로 cobot_control과 무관.

        이 메서드는 ROS2를 새로 호출하지 않고 _robot_status/_error_log(구독
        콜백/폴링 콜백들이 이미 채워둔 값)를 그대로 스냅샷 떠서 반환할 뿐이다.
        admin 화면이 1초마다 폴링해도 가벼운 이유가 이것 — 매번 로봇에 물어보는
        게 아니라 그냥 최근에 도착한 값을 읽기만 한다.
        """
        now = time.time()
        with self._status_lock:
            last = self._robot_status["last_joint_time"]
            making_code = self._robot_status["making_code"]
            making_active = making_code not in MAKING_INACTIVE_CODES
            making_started = self._robot_status["making_started_at"]
            status = {
                "robot": {
                    # 연결 여부 판단: joint_states 마지막 수신 시각이 최근이면 연결됨으로 간주
                    "connected": (now - last) < JOINT_STALE_SEC if last else False,
                    "joints_deg": list(self._robot_status["joints_deg"]),
                    "gripper": _gripper_label(self._robot_status["do1"], self._robot_status["do2"]),
                    "tool_force": list(self._robot_status["tool_force"]),
                    "speed_mode": self._robot_status["speed_mode"],
                },
                "making": {
                    "active": making_active,
                    "status_code": making_code,
                    "status_name": STATUS_NAMES.get(making_code, str(making_code)),
                    "elapsed_sec": int(now - making_started) if making_active and making_started else 0,
                    # 최근 제조 이력 (최신이 앞, SQLite 영속화라 재시작에도 유지)
                    "history": list(self._making_history),
                },
                "errors": list(self._error_log),
            }
        return status

    def publish_kiosk_lock(self, locked):
        """키오스크 잠금 상태 발행 — kiosk가 KIOSK_LOCK_TOPIC(래치)으로 받아간다.

        잠금 값 자체는 app.py의 kiosk_locked 전역이 단일 소스이고, 여기서는
        그 값이 바뀔 때(admin_lock 라우트)와 시작 시(생성자) 발행만 한다.
        """
        msg = Bool()
        msg.data = bool(locked)
        self._kiosk_lock_pub.publish(msg)
        self._node.get_logger().info(
            f"[admin] 키오스크 {'잠금' if msg.data else '잠금 해제'} — {KIOSK_LOCK_TOPIC} 발행")

    def clear_errors(self):
        """관리자 화면의 로그 비우기 (화면 관리용 — 로봇에는 아무 영향 없음)."""
        with self._status_lock:
            self._error_log.clear()
        return {"status": "success", "message": "로그를 지웠습니다."}

    # ==============================================================
    # 제어 (Flask 요청 스레드에서 호출 — 두산 서비스 응답까지 블로킹)
    # ==============================================================

    def stop_robot(self):
        """로봇 정지 — /stop_perfume(true) 발행. 실제 정지는 cobot_control이 수행한다.

        정지 로직은 cobot_control 한 곳에 모여 있다: 이 신호를 받으면 그쪽이
        move_stop(진행 중 모션 정지) + 시퀀스 중단(KeyboardInterrupt) +
        perfume_done=false 발행 후 종료까지 처리한다 (검증된 참고 구현:
        main_thread.py의 stop_callback — 새 메인 파일에도 이 구독이 반드시
        있어야 정지가 성립한다). HMI가 move_stop을 직접 부르지 않는 이유는
        STOP_PERFUME_TOPIC 주석 참고.

        발행은 구독자가 없어도 조용히 성공하므로, 구독자 수를 확인해서
        cobot_control이 안 듣고 있으면 관리자에게 에러로 알린다 — 이때 로봇은
        이 버튼으로 멈출 수 없다는 뜻이다 (물리 비상정지가 최후 수단).

        주의 1: 이건 이더넷(ROS2) 경유의 일반 소프트웨어 정지라 안전
        정지(STO/SS1/SS2)나 비상 정지가 아니다 — 물리 비상정지 버튼을 대체할
        수 없다 (모듈 주석 참고).

        2: 정지하면 cobot_control 프로세스가 종료되므로, 새 주문을 받으려면
        cobot_control을 다시 실행해야 한다.
        """
        stop_msg = Bool()
        stop_msg.data = True
        self._stop_perfume_pub.publish(stop_msg)

        listeners = self._stop_perfume_pub.get_subscription_count()
        if listeners == 0:
            self._node.get_logger().error(
                "[admin] 정지 신호(/stop_perfume) 발행 — 구독자 없음! cobot_control 미실행/미구현 의심")
            return {"status": "error",
                    "message": "정지 신호를 발행했지만 받는 노드가 없습니다 — "
                               "cobot_control이 실행 중인지(그리고 /stop_perfume를 구독하는지) 확인하세요. "
                               "지금은 이 버튼으로 로봇을 멈출 수 없습니다."}

        self._node.get_logger().warn(f"[admin] 로봇 정지 — /stop_perfume 발행 (구독자 {listeners}개)")
        return {"status": "success",
                "message": "정지 신호 전송 완료 — cobot_control이 모션 정지와 시퀀스 중단을 수행합니다. "
                           "새 주문을 받으려면 현장 확인 후 cobot_control을 다시 실행하세요."}

    def set_gripper(self, action):
        """그리퍼 수동 개폐 — cobot_control의 grip()/release()와 같은 DO 시퀀스를 그대로 재현한다.

        팔 모션 없이 컨트롤박스 디지털 출력만 바꾸므로 홈 복귀 등과 달리 저위험.
        action: "grip"(닫기/파지) 또는 "release"(열기).
        cobot_control과 동일하게 1~3번을 전부 OFF로 리셋한 뒤 목표 조합을 쓴다 —
        3번(GRIPPER_DO_RESERVED_INDEX)은 아직 의미가 없지만, cobot_control이 매번
        방어적으로 리셋하는 핀이라 여기서도 같이 초기화해 둔다(향후 의미가 생겨도
        수동 버튼이 이전 상태를 남겨두지 않도록).
        """
        if action == "grip":
            target = {GRIPPER_DO_GRIP_INDEX: DO_ON, GRIPPER_DO_RELEASE_INDEX: DO_OFF,
                      GRIPPER_DO_RESERVED_INDEX: DO_OFF}
        elif action == "release":
            target = {GRIPPER_DO_GRIP_INDEX: DO_OFF, GRIPPER_DO_RELEASE_INDEX: DO_ON,
                      GRIPPER_DO_RESERVED_INDEX: DO_OFF}
        else:
            return {"status": "error", "message": f"알 수 없는 그리퍼 동작: {action}"}

        if not self._set_digital_output_client.wait_for_service(timeout_sec=2.0):
            return {"status": "error", "message": "I/O 서비스에 연결할 수 없습니다."}

        def _write_do(index, value):
            request = SetCtrlBoxDigitalOutput.Request()
            request.index = index
            request.value = value
            return _wait_future(self._set_digital_output_client.call_async(request), IO_TIMEOUT_SEC)

        # 1단계: 1~3번 전부 OFF로 리셋 (cobot_control의 grip()/release()와 동일한 순서)
        for index in (GRIPPER_DO_GRIP_INDEX, GRIPPER_DO_RELEASE_INDEX, GRIPPER_DO_RESERVED_INDEX):
            response = _write_do(index, DO_OFF)
            if response is None or not response.success:
                return {"status": "error", "message": f"그리퍼 DO{index} 리셋에 실패했습니다."}

        # 2단계: 목표 조합 설정
        for index, value in target.items():
            response = _write_do(index, value)
            if response is None or not response.success:
                return {"status": "error", "message": f"그리퍼 DO{index} 설정에 실패했습니다."}

        # 다음 폴링을 기다리지 않고 캐시를 즉시 갱신해서 UI 배지가 바로 바뀌게 한다.
        with self._status_lock:
            self._robot_status["do1"] = target[GRIPPER_DO_GRIP_INDEX]
            self._robot_status["do2"] = target[GRIPPER_DO_RELEASE_INDEX]
            self._robot_status["do3"] = target[GRIPPER_DO_RESERVED_INDEX]

        label = "닫기(파지)" if action == "grip" else "열기"
        return {"status": "success", "message": f"그리퍼 {label} 완료."}
    
    
    def set_speed_mode(self, mode):
        """로봇 속도 모드 전환 — SPEED_MODE_NORMAL(0) / SPEED_MODE_REDUCED(1).

        두산 안전 속도 모드라 가속도 별도 설정은 없다 (드라이버가 vel/acc 전역
        오버라이드 서비스를 제공하지 않음 — 모션별 vel/acc는 cobot_control 소관).
        """
        if mode not in (SPEED_MODE_NORMAL, SPEED_MODE_REDUCED):
            return {"status": "error", "message": f"알 수 없는 속도 모드: {mode}"}

        if not self._set_speed_mode_client.wait_for_service(timeout_sec=2.0):
            return {"status": "error", "message": "속도 모드 서비스에 연결할 수 없습니다."}

        request = SetRobotSpeedMode.Request()
        request.speed_mode = mode
        response = _wait_future(self._set_speed_mode_client.call_async(request), IO_TIMEOUT_SEC)
        if response is None or not response.success:
            return {"status": "error", "message": "속도 모드 전환에 실패했습니다."}

        with self._status_lock:
            self._robot_status["speed_mode"] = mode

        label = "감속" if mode == SPEED_MODE_REDUCED else "일반"
        return {"status": "success", "message": f"{label} 모드로 전환했습니다."}


    # 홈 복귀 — move_joint 서비스 호출 (SYNC 모션)
    def move_home(self):
        """홈 자세(HOME_POSJ)로 복귀. SYNC 모드라 이동이 끝나야 응답이 온다.

        주의: 로봇 제어부 패키지가 별도 프로세스로 동시에 제조 모션을 보낼 수
        있는데, 이 프로세스의 robot_lock은 그 프로세스를 알지 못한다 — 홈 복귀와
        제조가 겹치지 않게 하려면 로봇 제어부 쪽(또는 로봇 컨트롤러 자체)에서도
        막아줘야 한다 (cross-process 동시성은 이 모듈 책임 밖).
        """
        if not self._move_joint_client.wait_for_service(timeout_sec=2.0):
            return {"status": "error", "message": "모션 서비스에 연결할 수 없습니다."}

        request = MoveJoint.Request()
        request.pos = HOME_POSJ
        request.vel = HOME_VEL
        request.acc = HOME_ACC
        request.sync_type = 0  # SYNC: 이동 완료 후 응답

        self._node.get_logger().info(f"[admin] 홈 복귀 시작: {HOME_POSJ}")
        response = _wait_future(self._move_joint_client.call_async(request), timeout_sec=MOVE_HOME_TIMEOUT_SEC)
        if response is None or not response.success:
            return {"status": "error", "message": "홈 복귀에 실패했습니다."}
        return {"status": "success", "message": "홈 복귀 완료."}
