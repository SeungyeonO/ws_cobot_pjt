"""두산 협동로봇 M0609 제어 모듈 (ROS2 서비스 클라이언트).

[역할 분담]
- 향료병 위치로 이동 / 뚜껑 열고 닫기 / 실제 토출 등 로봇 동작 자체는
  로봇 제어 쪽(별도 ROS2 노드, 예: ws_dsr/src/rokey 계열 노드)에서 구현한다.
- 이 모듈(Flask 백엔드)이 담당하는 건 딱 하나:
  "어떤 향료를 몇 샷 넣을지"를 ROS2 서비스로 요청하고, 로봇이 실제로
  제조를 끝낼 때까지 기다렸다가 그 결과를 반환하는 것.

[서비스 계약(contract)]
- 서비스 타입: perfume_order_srv/srv/Order
  (정의 위치: ws_cobot1/src/perfume_order_srv/srv/Order.srv)
- 요청 필드가 scent1~scent6(int8) 6칸 고정이라, scent 이름 → 슬롯 매핑을
  SCENT_ORDER 상수(아래)로 고정해뒀다. 로봇 쪽 향료병 물리 배치 순서와
  반드시 일치해야 하니, 순서가 다르면 로봇 쪽과 맞출 것.
- 응답은 success(bool) 하나뿐이라 실패 사유(message)나 실제 토출량 검증
  (total_shots)을 로봇 쪽에서 받아올 수 없다. total_shots는 요청 시점에
  로컬에서 합산한 값을 그대로 되돌려준다.
- 서비스 이름: ORDER_SERVICE 상수 (아래) — 로봇 제어 쪽 노드가 이
  이름으로 서비스 서버를 열어야 서로 통신이 된다. 이름이 안 맞으면
  wait_for_service()가 계속 실패하니 로봇 쪽 코드와 반드시 맞출 것.
- 중요한 약속: 로봇 제어 쪽은 "요청을 접수했다"는 의미로 바로 응답하면 안 되고,
  실제로 로봇팔이 제조를 전부 마친 뒤에만 응답을 보내야 한다.
  왜냐하면 이 서비스 호출(call_async → 응답 대기)이 곧 "제조중 화면을
  얼마나 오래 띄울지"를 결정하기 때문이다. 즉:
    제조하기 버튼 클릭
    → /api/make_perfume 요청 (Flask)
    → dispense() 호출 → ROS2 서비스 요청 전송
    → [블로킹] 로봇이 실제로 다 만들 때까지 대기 (= 프론트는 '제조중' 화면 유지)
    → 로봇이 응답을 보내야 비로소 여기서 리턴 → Flask 응답 → 프론트 '완료' 화면

[스레드 구조]
Flask(werkzeug) 개발 서버는 동기식이라 ROS2 이벤트 루프(spin)를 요청
스레드에서 직접 돌릴 수 없다. 그래서:
- init_ros()에서 ROS2 노드를 만들고, 전용 백그라운드 스레드에서 계속
  spin 시켜 서비스 응답 콜백이 처리되게 한다.
- Flask 요청 스레드(dispense 함수)는 call_async로 요청만 보내고,
  future.done()이 True가 될 때까지 짧게 sleep하며 폴링한다.
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

from perfume_order_srv.srv import Order

# ---- 로봇 쪽 노드와 반드시 일치해야 하는 상수들 ----
ROBOT_NAMESPACE = "dsr01"                                    # 두산 로봇 네임스페이스
ORDER_SERVICE = f"/{ROBOT_NAMESPACE}/perfume/order"          # 제조 요청 서비스 이름
# scent 이름 → Order.srv의 scent1~scent6 슬롯 순서.
# frontend/app.js SCENTS, backend/app.py VALID_SCENTS와 동일한 순서(top→middle→base)를
# 그대로 사용한 것으로, 로봇 쪽 향료병 배치 순서와 반드시 일치해야 한다.
SCENT_ORDER = ["Citrus", "Green", "Floral", "Woody", "Musk", "Amber"]

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

SERVICE_WAIT_TIMEOUT_SEC = 5.0   # 로봇 제어 노드가 떠 있는지 확인하는 대기 시간(초)
DISPENSE_TIMEOUT_SEC = 300.0     # 제조 완료 응답을 기다리는 최대 시간(초) — 로봇 응답 없을 때 안전장치
POLL_INTERVAL_SEC = 0.1          # 응답 도착 여부를 확인하는 폴링 주기(초)

JOINT_STALE_SEC = 2.0            # joint_states가 이 시간 이상 안 오면 "연결 끊김"으로 판단
ERROR_LOG_MAX = 20               # 관리자 화면에 보여줄 최근 에러 보관 개수
HOME_POSJ = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]  # 프로젝트 공용 홈 자세 (rokey 코드와 동일)
HOME_VEL, HOME_ACC = 30.0, 30.0               # 홈 복귀 속도/가속 (안전하게 낮게)
MOVE_HOME_TIMEOUT_SEC = 60.0     # 홈 복귀 완료 응답 대기 한도(초)

# 모듈 전역 상태 — init_ros()에서 한 번만 채워지고 이후 dispense()에서 재사용한다.
_node = None
_client = None
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
_making_status = {
    "active": False,       # 지금 제조 중인지
    "recipe_name": None,
    "plan": None,
    "started_at": None,
    "last": None,          # 직전 제조 결과 요약
}


def init_ros():
    """Flask 앱 시작 시 딱 한 번 호출한다.

    ROS2를 초기화하고, 서비스 클라이언트를 만들고, 백그라운드 스레드에서
    spin을 시작한다. 이미 초기화되어 있으면 아무 것도 하지 않는다.
    """
    global _node, _client, _executor, _spin_thread, _estop_client, _move_joint_client
    if _node is not None:
        return  # 중복 초기화 방지 (Flask 리로더 등으로 두 번 불려도 안전)

    rclpy.init()
    _node = Node("perfume_backend_client")
    _client = _node.create_client(Order, ORDER_SERVICE)

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
    _executor = MultiThreadedExecutor()
    _executor.add_node(_node)
    _spin_thread = threading.Thread(target=_executor.spin, daemon=True)
    _spin_thread.start()

    _node.get_logger().info(f"[perfume] ROS2 준비 완료 (서비스: {ORDER_SERVICE})")


def shutdown_ros():

    """Flask 앱 종료 시 ROS2 자원을 정리한다."""

    global _node, _client, _executor, _spin_thread
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
    _node = _client = _executor = _spin_thread = None


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
    """
    now = time.time()
    with _status_lock:
        last = _robot_status["last_joint_time"]
        making = dict(_making_status)
        status = {
            "robot": {
                "connected": (now - last) < JOINT_STALE_SEC if last else False,
                "joints_deg": list(_robot_status["joints_deg"]),
            },
            "errors": list(_error_log),
        }
    if making["active"]:
        making["elapsed_sec"] = round(now - making["started_at"], 1)
    status["making"] = making
    return status


def _wait_future(future, timeout_sec):
    """call_async future가 끝날 때까지 폴링 대기. 시간 초과 시 None 반환."""
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

    주의: 이건 '로봇 팔의 모션'을 멈추는 것이고, 제조 요청(dispense) 자체를
    취소하는 게 아니다. 제조 중 E-stop을 누르면 로봇 제어 노드 쪽에서 실패
    응답을 보내거나 dispense 타임아웃으로 정리된다.
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

    제조 중 호출 금지 — 호출하는 쪽(app.py)에서 robot_lock으로 막는다.
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


def dispense(recipe_name, dispense_plan):

    """향료 토출을 ROS2로 요청하고, 로봇이 실제로 제조를 끝낼 때까지 대기.

    Args:
        recipe_name (str): 조합명 (로그/응답 표시용).
        dispense_plan (list[dict]): [{"scent": str, "shots": int}, ...]

    Returns:
        dict: {"status": "success", "recipe_name": str, "total_shots": int}
              또는 {"status": "error", "message": str}
    """

    if _client is None:
        # init_ros()를 안 부르고 dispense()부터 호출한 설정 실수 — 안전하게 에러 처리
        return {"status": "error", "message": "ROS2가 초기화되지 않았습니다. (init_ros 누락)"}

    total_shots = sum(item["shots"] for item in dispense_plan)


    # 로봇 제어 노드가 서비스를 아직 안 열었거나(꺼져 있거나) 하면 여기서 바로 실패 처리.
    # 이게 없으면 로봇이 꺼진 상태에서 DISPENSE_TIMEOUT_SEC(5분)까지 대기.
    if not _client.wait_for_service(timeout_sec=SERVICE_WAIT_TIMEOUT_SEC):
        return {
            "status": "error",
            "message": "로봇 제어 서비스에 연결할 수 없습니다. 로봇 전원/노드 상태를 확인해주세요.",
        }

    shots_by_scent = {item["scent"]: int(item["shots"]) for item in dispense_plan}
    request = Order.Request()
    (
        request.scent1, request.scent2, request.scent3,
        request.scent4, request.scent5, request.scent6,
    ) = [shots_by_scent.get(name, 0) for name in SCENT_ORDER]

    _node.get_logger().info(
        f"[perfume] 제조 요청 전송: {recipe_name} (총 {total_shots}샷, {len(dispense_plan)}개 향료)"
    )

    # 관리자 화면 '제조 현황' 표시용 — 시작 기록 후 finally에서 종료 기록
    with _status_lock:
        _making_status.update(
            active=True, recipe_name=recipe_name, plan=dispense_plan,
            started_at=time.time(),
        )
    result = {"status": "error", "message": "알 수 없는 오류"}
    try:
        # call_async: 요청만 보내고 즉시 future를 돌려받는다 (블로킹 아님).
        # 실제 완료 대기는 아래 while 루프에서 future.done()을 폴링하며 수행한다.
        # → 이 루프가 끝나는 시점 = 로봇 응답 도착 시점 = 실제 제조 완료 시점.
        future = _client.call_async(request)

        waited = 0.0
        while not future.done():
            time.sleep(POLL_INTERVAL_SEC)
            waited += POLL_INTERVAL_SEC
            if waited >= DISPENSE_TIMEOUT_SEC:
                future.cancel()
                result = {
                    "status": "error",
                    "message": "로봇 응답 시간이 초과되었습니다. 로봇이 정상 종료됐는지 확인해주세요.",
                }
                return result

        response = future.result()
        if response is None or not response.success:
            # Order.Response에는 실패 사유 필드가 없어 구체적인 메시지를 받을 수 없다.
            result = {"status": "error", "message": "제조에 실패했습니다."}
            return result

        _node.get_logger().info(f"[perfume] 제조 완료 응답 수신: {recipe_name}")
        result = {
            "status": "success",
            "recipe_name": recipe_name,
            "total_shots": total_shots,
        }
        return result
    finally:
        with _status_lock:
            _making_status.update(active=False, recipe_name=None, plan=None, started_at=None)
            _making_status["last"] = {
                "recipe_name": recipe_name,
                "status": result["status"],
                "finished_at": time.strftime("%H:%M:%S"),
            }
