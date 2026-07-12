"""조향 자동화 솔루션 HMI 백엔드 — 관리자 모니터링/대시보드 전용 (로봇 제어 PC에서 실행).

이 패키지는 "제조(dispense)"를 다루지 않는다. 손님 주문(조향 시작) 요청은
로봇팔 제어부 쪽에서 별도로 개발 중인 패키지가 직접 받는다 — 그 패키지가
완성되면 launch file로 이 perfume_hmi와 함께 로봇 제어 PC에서 묶어 띄울
예정이다. perfume_hmi가 맡는 건 관리자 화면(로봇 연결/조인트/그리퍼/TCP 힘/
로그 모니터링, 로봇 정지·서보 복구, 홈 복귀, 그리퍼 수동 개폐, 속도 모드
전환, 키오스크 잠금 토글, 로그 비우기)뿐이다.
(일시정지/재개는 두산 드라이버가 모션 진행 중에 응답을 못 하는 구조라 제거 — robot_bridge 주석 참고)

로봇과 랜선으로 직접 연결된 PC에서 robot_bridge.py(ROS2)와 같은 프로세스로
띄운다. 키오스크 백엔드(perfume_kiosk, 다른 PC)는 이 서버의
GET /internal/lock_status만 서버-to-서버로 호출해 잠금 상태를 읽는다 —
kiosk_locked의 단일 소스는 이 서버다. (제조 요청은 kiosk가 로봇 제어부
패키지에 직접 보낸다 — perfume_kiosk 쪽 문서 참고)

- /admin                    관리자 HMI (PIN 로그인 필요 — 미로그인 시 로그인 페이지 표시)
  - POST /api/admin/login    PIN 로그인 {"password": "..."} → 세션 발급 (30분 유효)
  - POST /api/admin/logout   로그아웃
  - GET  /api/admin/status   로봇 연결/조인트/그리퍼/TCP 힘/속도 모드/제조 현황/로그/잠금 상태 (1초 폴링용)
  - POST /api/admin/stop     로봇 정지 (두산 move_stop — 감속 정지 후 서보 차단.
                             소프트웨어 정지라 안전 정지/비상 정지 아님 — robot_bridge 주석 참고)
  - POST /api/admin/servo_on 정지 후 서보 복구 (두산 set_robot_control)
  - POST /api/admin/home     홈 자세 복귀
  - POST /api/admin/lock     키오스크 잠금/해제 {"locked": true|false}
  - POST /api/admin/gripper  그리퍼 수동 개폐 {"action": "grip"|"release"}
  - POST /api/admin/speed_mode  속도 모드 전환 {"mode": "normal"|"reduced"}
  - POST /api/admin/clear_errors  로그 비우기
  - 관리자 PIN: 환경변수 ADMIN_PASSWORD (기본값 "0609")
- /internal/*               키오스크 백엔드 전용 (브라우저 대상 아님)
  - GET  /internal/lock_status  {"locked": bool}
  - 인증: 헤더 X-HMI-Api-Key == 환경변수 HMI_API_KEY (기본값 "perfume-internal-key")

[실행] ament_python 패키지로 설치되므로 colcon build 후 다음처럼 실행한다.
(env.sh가 아래 source 3줄을 대신 해준다.)
  source /opt/ros/humble/setup.bash
  source <ws_dsr>/install/setup.bash
  source <ws_cobot1>/install/setup.bash
  ros2 run perfume_hmi perfume_hmi
"""
import atexit
import os
import secrets
import threading
import time

from ament_index_python.packages import get_package_share_directory
from flask import Flask, jsonify, redirect, request, send_from_directory, session

from .robot_bridge import RobotBridge, SPEED_MODE_NORMAL, SPEED_MODE_REDUCED

PACKAGE_SHARE_DIR = get_package_share_directory("perfume_hmi")
FRONTEND_DIR = os.path.join(PACKAGE_SHARE_DIR, "frontend")

# 앱 전체에서 하나만 쓰는 로봇 브릿지(ROS2) — main()에서 생성된다
# (perfume_kiosk의 _robot 패턴과 동일). 라우트는 전부 이 인스턴스를 통해
# 로봇과 통신한다.
bridge = None

#======================================================================
# 홈 복귀 버튼 연타 등으로 모션 명령이 겹치는 것만 막는다. 로봇 제어부(별도
# 프로세스)가 보내는 제조 모션과의 충돌 방지는 이 락으로 커버되지 않는다 —
# bridge.move_home() 주석 참고.
robot_lock = threading.Lock()
#======================================================================

# 관리자 키오스크 잠금 — True면 손님 주문을 받지 않는다 (점검/향료 교체용).
# 키오스크 PC는 GET /internal/lock_status로 이 값을 읽어간다.
kiosk_locked = False

# ---- 관리자 인증 (브라우저 세션) ----
ADMIN_PASSWORD = os.environ.get("ADMIN_PASSWORD", "0609")  # 기본 PIN = 로봇 모델명
ADMIN_SESSION_MAX_AGE_SEC = 1800  # 로그인 유지 시간 30분

# ---- 키오스크 백엔드 인증 (서버-to-서버) ----
HMI_API_KEY = os.environ.get("HMI_API_KEY", "perfume-internal-key")

app = Flask(__name__, static_folder=None)
# 세션 서명 키 — 서버 재시작 시마다 새로 생성되므로 재시작하면 재로그인 필요 (의도된 동작)
app.secret_key = secrets.token_hex(16)


def _admin_authed():
    """관리자 세션이 유효한지 (로그인했고 30분이 지나지 않았는지) 확인."""
    logged_in_at = session.get("admin_at")
    return logged_in_at is not None and (time.time() - logged_in_at) < ADMIN_SESSION_MAX_AGE_SEC


@app.before_request
def _guard_requests():
    """모든 /api/admin/*(login 제외)와 /internal/* 요청을 한 곳에서 인증 검사.

    인증 방식이 두 종류라 여기서 경로로 분기한다:
    - /api/admin/*: 브라우저 세션 쿠키 기반 (admin_login이 session["admin_at"]을
      채우고, _admin_authed()가 30분 이내인지 매 요청마다 검사).
    - /internal/*: 브라우저가 아니라 키오스크 백엔드가 서버-to-서버로 호출하는
      경로라 세션이 없다. 대신 헤더 X-HMI-Api-Key == HMI_API_KEY 여부만 본다.
    @app.before_request로 등록돼 있어서 모든 라우트 함수보다 먼저 실행되므로,
    각 라우트 안에는 인증 코드가 따로 없다.
    """
    path = request.path
    if path.startswith("/api/admin/") and path != "/api/admin/login":
        if not _admin_authed():
            return jsonify({"status": "error", "message": "로그인이 필요합니다."}), 401
    elif path.startswith("/internal/"):
        if request.headers.get("X-HMI-Api-Key") != HMI_API_KEY:
            return jsonify({"status": "error", "message": "인증되지 않은 요청입니다."}), 401


# ---------- 프론트엔드 서빙 (관리자 HMI만) ----------
@app.route("/")
def index():
    return redirect("/admin")


@app.route("/<path:filename>")
def frontend_files(filename):
    return send_from_directory(FRONTEND_DIR, filename)


# ---------- 관리자 HMI ----------
@app.route("/admin")
def admin_page():
    """로그인 상태면 HMI, 아니면 로그인 페이지."""
    if not _admin_authed():
        return send_from_directory(FRONTEND_DIR, "admin_login.html")
    return send_from_directory(FRONTEND_DIR, "admin.html")


@app.route("/api/admin/login", methods=["POST"])
def admin_login():
    data = request.get_json(silent=True) or {}
    if data.get("password") != ADMIN_PASSWORD:
        time.sleep(1)  # 무차별 대입 속도 늦추기
        return jsonify({"status": "error", "message": "PIN이 올바르지 않습니다."}), 401
    session["admin_at"] = time.time()
    return jsonify({"status": "success"})


@app.route("/api/admin/logout", methods=["POST"])
def admin_logout():
    session.clear()
    return jsonify({"status": "success"})


@app.route("/api/admin/status", methods=["GET"])
def admin_status():
    """관리자 화면이 1초마다 폴링하는 통합 상태.

    이 라우트는 ROS2를 직접 호출하지 않는다 — bridge.get_status()가
    돌려주는 건 백그라운드 스레드가 토픽 구독으로 이미 쌓아둔 값의 스냅샷일
    뿐이라 가볍다 (자세한 원리는 robot_bridge.py의 RobotBridge 주석 참고).
    """
    status = bridge.get_status()
    status["kiosk_locked"] = kiosk_locked
    return jsonify(status)


@app.route("/api/admin/stop", methods=["POST"])
def admin_stop():
    """로봇 정지 — 제조 중이든 아니든 즉시 실행한다. 정지 후 서보가 차단된다.

    소프트웨어 정지(move_stop)라 안전 정지/비상 정지가 아니다 — robot_bridge 주석 참고.
    """
    result = bridge.stop_robot()
    return jsonify(result), 200 if result["status"] == "success" else 502


@app.route("/api/admin/servo_on", methods=["POST"])
def admin_servo_on():
    """정지(Safe-Off) 후 서보 복구 — 티칭펜던트 'Servo On'과 동일."""
    result = bridge.servo_on()
    return jsonify(result), 200 if result["status"] == "success" else 502


@app.route("/api/admin/home", methods=["POST"])
def admin_home():
    """홈 자세 복귀. robot_lock은 버튼 연타로 인한 중복 요청만 막는다."""
    if not robot_lock.acquire(blocking=False):
        return jsonify({"status": "error",
                        "message": "이미 홈 복귀가 진행 중입니다."}), 409
    try:
        result = bridge.move_home()
    finally:
        robot_lock.release()
    return jsonify(result), 200 if result["status"] == "success" else 502


@app.route("/api/admin/lock", methods=["POST"])
def admin_lock():
    """키오스크 잠금/해제 토글."""
    global kiosk_locked
    data = request.get_json(silent=True) or {}
    kiosk_locked = bool(data.get("locked"))
    return jsonify({"locked": kiosk_locked})


@app.route("/api/admin/gripper", methods=["POST"])
def admin_gripper():
    """그리퍼 수동 열기/닫기 — 팔 모션 없이 I/O만 바꾸므로 robot_lock 불필요."""
    data = request.get_json(silent=True) or {}
    result = bridge.set_gripper(data.get("action"))
    return jsonify(result), 200 if result["status"] == "success" else 502


@app.route("/api/admin/speed_mode", methods=["POST"])
def admin_speed_mode():
    """로봇 속도 모드 전환 {"mode": "normal"|"reduced"}."""
    data = request.get_json(silent=True) or {}
    mode_map = {"normal": SPEED_MODE_NORMAL,
                "reduced": SPEED_MODE_REDUCED}
    mode = mode_map.get(data.get("mode"))
    if mode is None:
        return jsonify({"status": "error", "message": "mode는 normal 또는 reduced여야 합니다."}), 400
    result = bridge.set_speed_mode(mode)
    return jsonify(result), 200 if result["status"] == "success" else 502


@app.route("/api/admin/clear_errors", methods=["POST"])
def admin_clear_errors():
    """로그 비우기 (화면 관리용)."""
    return jsonify(bridge.clear_errors())


# ---------- 키오스크 백엔드 전용 내부 API ----------
@app.route("/internal/lock_status", methods=["GET"])
def internal_lock_status():
    """키오스크 백엔드가 시작 화면/점검 폴링 때마다 확인하는 잠금 상태."""
    return jsonify({"locked": kiosk_locked})


def main():
    # 서버가 켜져 있는 동안 계속 쓸 로봇 브릿지(ROS2)를 한 번만 만들고,
    # 프로세스가 종료될 때(atexit) 정리한다. Flask 요청을 처리하는 이
    # 메인 스레드와 별개로, RobotBridge 생성자 안에서 ROS2 spin용 데몬
    # 스레드가 하나 더 뜬다 (robot_bridge.py의 RobotBridge 주석 참고).
    global bridge
    bridge = RobotBridge()
    atexit.register(bridge.shutdown)

    debug = os.environ.get("FLASK_DEBUG", "0") == "1"
    app.run(host="0.0.0.0", port=5000, debug=debug, threaded=True)


if __name__ == "__main__":
    main()
