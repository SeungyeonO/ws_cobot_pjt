"""두산 협동로봇 M0609 HMI-로봇 브릿지 (ROS2, 모니터링 + 비상정지/홈 복귀 전용).

이 모듈은 "제조(dispense)"를 다루지 않는다. perfume_hmi는 순수 관리자
모니터링/대시보드 패키지로, 로봇 연결 상태·조인트 각도·에러 로그를 보여주고
비상 정지·홈 복귀만 수행한다. 손님 주문(조향 시작) 요청은 이 워크스페이스가
아니라 로봇팔 제어부 쪽에서 별도로 개발 중인 패키지가 직접 받는다 — 그 쪽이
완성되면 launch file로 perfume_hmi와 함께 묶어 로봇 제어 PC에서 같이 띄울
예정이다 (perfume_order_srv/Order.srv 클라이언트도 그 패키지가 가져간다).

[역할]
- ROS2 토픽 구독으로 조인트 각도/에러/연결 끊김 이벤트를 모아 admin 화면에
  보여줄 상태 스냅샷(get_status())을 만든다.
- 두산 표준 서비스로 비상 정지(estop)와 홈 복귀(move_home)를 수행한다.

[스레드 구조]
Flask(werkzeug) 개발 서버는 동기식이라 ROS2 이벤트 루프(spin)를 요청
스레드에서 직접 돌릴 수 없다. 그래서:
- init_ros()에서 ROS2 노드를 만들고, 전용 백그라운드 스레드에서 계속
  spin 시켜 서비스 응답 콜백이 처리되게 한다.
- estop()/move_home()은 call_async로 요청만 보내고, future.done()이
  True가 될 때까지 짧게 sleep하며 폴링한다.
"""
import math
import threading
import time
from collections import deque

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState

from dsr_msgs2.msg import RobotError, RobotDisconnection
from dsr_msgs2.srv import MoveJoint, MoveStop

# ---- 로봇 쪽 노드와 반드시 일치해야 하는 상수들 ----
ROBOT_NAMESPACE = "dsr01"                # 두산 로봇 네임스페이스

# 관리자 HMI용 두산 표준 토픽/서비스 (시뮬레이션·실기 공통)
# 조인트 상태는 환경에 따라 나오는 토픽이 달라서 둘 다 구독한다:
# - 실제 로봇(dsr_bringup2 real): /dsr01/joint_states 에 실측값 발행
# - 시뮬레이션(m0609_rg2_bringup): /joint_states 만 발행 (시각화용이라 값은
#   0으로 고정될 수 있음 — 이 경우 각도 표시는 안 되지만 연결 감지는 동작)
# 콜백은 공용이며, 이름으로 로봇 6축(joint_1~6)만 골라낸다 (그리퍼 조인트 제외).
JOINT_STATES_TOPICS = [f"/{ROBOT_NAMESPACE}/joint_states", "/joint_states"]
ROBOT_JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
ROBOT_ERROR_TOPIC = f"/{ROBOT_NAMESPACE}/error"
ROBOT_DISCONNECTION_TOPIC = f"/{ROBOT_NAMESPACE}/robot_disconnection"
MOVE_STOP_SERVICE = f"/{ROBOT_NAMESPACE}/motion/move_stop"
MOVE_JOINT_SERVICE = f"/{ROBOT_NAMESPACE}/motion/move_joint"

POLL_INTERVAL_SEC = 0.1          # 응답 도착 여부를 확인하는 폴링 주기(초)

JOINT_STALE_SEC = 2.0            # joint_states가 이 시간 이상 안 오면 "연결 끊김"으로 판단
ERROR_LOG_MAX = 20               # 관리자 화면에 보여줄 최근 에러 보관 개수
HOME_POSJ = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]  # 프로젝트 공용 홈 자세 (rokey 코드와 동일)
HOME_VEL, HOME_ACC = 30.0, 30.0               # 홈 복귀 속도/가속 (안전하게 낮게)
MOVE_HOME_TIMEOUT_SEC = 60.0     # 홈 복귀 완료 응답 대기 한도(초)

# 모듈 전역 상태 — init_ros()에서 한 번만 채워진다.
_node = None
_executor = None
_spin_thread = None
_estop_client = None
_move_joint_client = None

# ---- 관리자 HMI용 상태 저장소 ----
# ROS2 콜백 스레드와 Flask 요청 스레드가 동시에 접근하므로 락으로 보호한다.
_status_lock = threading.Lock()
_robot_status = {
    "joints_deg": [0.0] * 6,   # 현재 조인트 각도 [deg]
    "last_joint_time": 0.0,    # joint_states 마지막 수신 시각 (연결 판단용)
}
_error_log = deque(maxlen=ERROR_LOG_MAX)  # 최근 에러 목록 (최신이 앞)


def init_ros():
    """Flask 앱 시작 시 딱 한 번 호출한다.

    ROS2를 초기화하고, 서비스 클라이언트를 만들고, 백그라운드 스레드에서
    spin을 시작한다. 이미 초기화되어 있으면 아무 것도 하지 않는다.
    """
    global _node, _executor, _spin_thread, _estop_client, _move_joint_client
    if _node is not None:
        return  # 중복 초기화 방지 (Flask 리로더 등으로 두 번 불려도 안전)

    rclpy.init()
    _node = Node("perfume_hmi_client")

    # 관리자 HMI용 제어 클라이언트 (비상 정지 / 홈 복귀)
    _estop_client = _node.create_client(MoveStop, MOVE_STOP_SERVICE)
    _move_joint_client = _node.create_client(MoveJoint, MOVE_JOINT_SERVICE)

    # 관리자 HMI용 모니터링 구독 (조인트 상태 / 에러 / 연결 끊김 이벤트)
    for topic in JOINT_STATES_TOPICS:
        _node.create_subscription(JointState, topic, _joint_states_callback, 10)
    _node.create_subscription(RobotError, ROBOT_ERROR_TOPIC, _robot_error_callback, 10)
    _node.create_subscription(
        RobotDisconnection, ROBOT_DISCONNECTION_TOPIC, _robot_disconnection_callback, 10
    )

    # MultiThreadedExecutor + daemon thread: 메인(Flask) 스레드를 막지 않고
    # ROS2 콜백(서비스 응답 등)을 백그라운드에서 계속 처리하기 위함.
    # executor.spin()은 블로킹 호출이라 메인 스레드에서 그냥 부르면 Flask가
    # 멈춘다 — 그래서 별도 스레드에 맡기고 이 함수는 바로 리턴한다. 이 스레드가
    # 계속 돌면서 위에서 등록한 구독 콜백(_joint_states_callback 등)과
    # estop()/move_home()이 보낸 서비스 요청의 응답을 처리한다.
    _executor = MultiThreadedExecutor()
    _executor.add_node(_node)
    _spin_thread = threading.Thread(target=_executor.spin, daemon=True)
    _spin_thread.start()

    _node.get_logger().info("[perfume_hmi] ROS2 준비 완료 (모니터링 + estop/home)")


def shutdown_ros():

    """Flask 앱 종료 시 ROS2 자원을 정리한다."""

    global _node, _executor, _spin_thread
    if _executor is not None:
        _executor.shutdown()
    if _spin_thread is not None:
        # spin 스레드가 완전히 끝난 뒤에 rclpy.shutdown()을 불러야 한다.
        # 순서를 지키지 않으면 인터프리터 종료 시 세그폴트가 날 수 있다.
        _spin_thread.join(timeout=2.0)
    if _node is not None:
        _node.destroy_node()
    if _node is not None or _executor is not None:
        rclpy.shutdown()
    _node = _executor = _spin_thread = None


# ==============================================================
# 관리자 HMI: 모니터링 콜백 + 상태 조회 + 제어 (E-stop / 홈 복귀)
# ==============================================================

def _joint_states_callback(msg):
    """로봇 조인트 상태 수신 — 각도 저장 + 수신 시각 기록(연결 판단 근거).

    /joint_states에는 그리퍼 조인트도 섞여 오므로, 이름으로 로봇 6축만 골라낸다.
    """
    pos_by_name = dict(zip(msg.name, msg.position))
    if not all(n in pos_by_name for n in ROBOT_JOINT_NAMES):
        return  # 로봇 6축이 없는 메시지(그리퍼 단독 등)는 무시
    with _status_lock:
        _robot_status["joints_deg"] = [
            round(math.degrees(pos_by_name[n]), 1) for n in ROBOT_JOINT_NAMES
        ]
        _robot_status["last_joint_time"] = time.time()


def _robot_error_callback(msg):
    """로봇 에러/경고 발생 시 최근 에러 목록에 추가 (최신이 앞)."""
    level_str = {1: "INFO", 2: "WARN", 3: "ERROR"}.get(msg.level, str(msg.level))
    with _status_lock:
        _error_log.appendleft({
            "time": time.strftime("%H:%M:%S"),
            "level": level_str,
            "code": msg.code,
            "message": msg.msg1 or f"group={msg.group} code={msg.code}",
        })


def _robot_disconnection_callback(msg):
    """로봇 연결 끊김 이벤트 — 에러 목록에 함께 기록해서 관리자가 볼 수 있게."""
    with _status_lock:
        _error_log.appendleft({
            "time": time.strftime("%H:%M:%S"),
            "level": "ERROR",
            "code": 0,
            "message": "로봇 연결이 끊어졌습니다 (robot_disconnection)",
        })


def get_status():
    """관리자 화면이 1초마다 폴링하는 상태 스냅샷.

    연결 여부는 joint_states 스트림이 최근에 들어왔는지로 판단한다.
    (연결 끊김 '이벤트' 토픽만으로는 재연결 시점을 알 수 없기 때문)

    주의: '제조 현황(making)'은 더 이상 여기서 알 수 없다 — 실제 제조는
    별도 로봇 제어부 패키지가 수행하기 때문. 그 패키지가 제조 상태를
    ROS2 토픽 등으로 공개하면 여기서 구독해 채워 넣을 수 있다 (TODO).

    이 함수는 ROS2를 새로 호출하지 않고 _robot_status/_error_log(구독
    콜백들이 이미 채워둔 값)를 그대로 스냅샷 떠서 반환할 뿐이다. admin
    화면이 1초마다 폴링해도 가벼운 이유가 이것 — 매번 로봇에 물어보는 게
    아니라 그냥 최근에 도착한 값을 읽기만 한다.
    """
    now = time.time()
    with _status_lock:
        last = _robot_status["last_joint_time"]
        status = {
            "robot": {
                "connected": (now - last) < JOINT_STALE_SEC if last else False,
                "joints_deg": list(_robot_status["joints_deg"]),
            },
            "errors": list(_error_log),
        }
    return status


def _wait_future(future, timeout_sec):
    """call_async future가 끝날 때까지 폴링 대기. 시간 초과 시 None 반환.

    call_async()는 요청만 보내고 즉시 future를 돌려주는 논블로킹 호출이라,
    실제 완료는 future.done()이 True가 될 때까지 기다려야 한다. asyncio 같은
    진짜 비동기 대신 짧게(POLL_INTERVAL_SEC) sleep하며 폴링하는 이유는, 이걸
    부르는 Flask 요청 스레드 입장에서는 "끝날 때까지 단순 블로킹"하는 형태로
    맞추는 게 가장 간단하기 때문이다 — estop()/move_home() 둘 다 이 함수의
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


def estop():
    """비상 정지 — 두산 공식 move_stop 서비스로 현재 모션을 즉시 중단시킨다.

    주의: 이건 '로봇 팔의 모션'을 멈추는 것이고, 제조 요청 자체를 취소하는
    게 아니다 — 제조는 별도 로봇 제어부 패키지가 처리하므로, 그쪽에서 실패
    처리를 하도록 되어 있어야 한다.
    """
    if _estop_client is None:
        return {"status": "error", "message": "ROS2가 초기화되지 않았습니다."}
    if not _estop_client.wait_for_service(timeout_sec=2.0):
        return {"status": "error", "message": "비상 정지 서비스에 연결할 수 없습니다."}

    request = MoveStop.Request()
    request.stop_mode = 1  # DR_QSTOP(1): Quick stop (Stop Category 2)

    _node.get_logger().warn("[admin] 비상 정지(move_stop) 호출!")
    response = _wait_future(_estop_client.call_async(request), timeout_sec=3.0)
    if response is None or not response.success:
        return {"status": "error", "message": "비상 정지 명령이 실패했습니다."}
    return {"status": "success", "message": "비상 정지 완료. 로봇 상태를 확인해주세요."}


def move_home():
    """홈 자세(HOME_POSJ)로 복귀. SYNC 모드라 이동이 끝나야 응답이 온다.

    주의: 로봇 제어부 패키지가 별도 프로세스로 동시에 제조 모션을 보낼 수
    있는데, 이 프로세스의 robot_lock은 그 프로세스를 알지 못한다 — 홈 복귀와
    제조가 겹치지 않게 하려면 로봇 제어부 쪽(또는 로봇 컨트롤러 자체)에서도
    막아줘야 한다 (cross-process 동시성은 이 모듈 책임 밖).
    """
    if _move_joint_client is None:
        return {"status": "error", "message": "ROS2가 초기화되지 않았습니다."}
    if not _move_joint_client.wait_for_service(timeout_sec=2.0):
        return {"status": "error", "message": "모션 서비스에 연결할 수 없습니다."}

    request = MoveJoint.Request()
    request.pos = HOME_POSJ
    request.vel = HOME_VEL
    request.acc = HOME_ACC
    request.sync_type = 0  # SYNC: 이동 완료 후 응답

    _node.get_logger().info(f"[admin] 홈 복귀 시작: {HOME_POSJ}")
    response = _wait_future(_move_joint_client.call_async(request), timeout_sec=MOVE_HOME_TIMEOUT_SEC)
    if response is None or not response.success:
        return {"status": "error", "message": "홈 복귀에 실패했습니다."}
    return {"status": "success", "message": "홈 복귀 완료."}
