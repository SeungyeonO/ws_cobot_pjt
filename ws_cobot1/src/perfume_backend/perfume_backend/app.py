"""조향 자동화 솔루션 Flask 백엔드.

- GET  /api/recipes        추천 조합 목록 (SQLite3 동적 로드)
- POST /api/make_perfume   제조 요청 (배율/횟수 계산 후 실제 로봇(M0609)에 ROS2로 제조 요청)
- GET  /api/kiosk_status   키오스크 잠금 여부 (손님 화면이 점검 중 표시에 사용)
- /                        프론트엔드 서빙
- /admin                   관리자 HMI (PIN 로그인 필요 — 미로그인 시 로그인 페이지 표시)
  - POST /api/admin/login   PIN 로그인 {"password": "..."} → 세션 발급 (30분 유효)
  - POST /api/admin/logout  로그아웃
  - GET  /api/admin/status  로봇 연결/조인트/에러 로그/제조 현황/잠금 상태 (1초 폴링용)
  - POST /api/admin/estop   비상 정지 (두산 move_stop)
  - POST /api/admin/home    홈 자세 복귀 (제조 중에는 거절)
  - POST /api/admin/lock    키오스크 잠금/해제 {"locked": true|false}
  - 관리자 PIN: 환경변수 ADMIN_PASSWORD (기본값 "0609")
- DB 초기화: 서버 시작 시 DB_PATH(~/.ros/perfume/perfume.db)가 없으면
  schema.sql + 시드로 자동 생성 (재초기화하려면 그 파일 삭제 후 재시작)

[제조 흐름 요약] (자세한 설명은 robot_control.py 상단 주석 참고)
프론트 "제조하기" 클릭 → POST /api/make_perfume → robot_control.dispense()가
ROS2로 로봇에 제조를 요청하고 실제로 다 만들 때까지 블로킹 → 그 응답을 그대로
JSON으로 돌려준다. 즉 이 HTTP 요청이 응답을 받는 순간이 곧 "실제 제조 완료" 시점이라,
프론트는 이 fetch가 끝날 때까지 '제조중' 화면을 유지하기만 하면 된다(프론트 수정 불필요).

[실행] ament_python 패키지로 설치되므로 colcon build 후 다음처럼 실행한다.
(perfume_backend/env.sh가 아래 source 3줄을 대신 해준다.)
  source /opt/ros/humble/setup.bash
  source <ws_dsr>/install/setup.bash
  source <ws_cobot1>/install/setup.bash
  ros2 run perfume_backend perfume_backend
"""
import atexit
import os
import secrets
import sqlite3
import threading
import time

from ament_index_python.packages import get_package_share_directory
from flask import Flask, jsonify, request, send_from_directory, session

from . import robot_control

PACKAGE_SHARE_DIR = get_package_share_directory("perfume_backend")
SCHEMA_PATH = os.path.join(PACKAGE_SHARE_DIR, "schema.sql")
FRONTEND_DIR = os.path.join(PACKAGE_SHARE_DIR, "frontend")

# DB는 설치된 패키지 디렉터리(재빌드 시 갱신됨) 대신 홈 아래 별도 위치에 둔다.
DB_DIR = os.path.expanduser("~/.ros/perfume")
DB_PATH = os.path.join(DB_DIR, "perfume.db")

# 계산 상수 — 토출 단위는 '샷' (1샷 = 밸브 1회 토출)
RECOMMEND_TOTAL_SHOTS = 10   # 추천 조합 총 샷 수 (배율 %를 샷으로 환산)
CUSTOM_SHOTS_PER_SCENT = 2   # '나만의 조합' 선택 향료당 고정 샷 수
FREE_MAX_SHOTS = 6           # '내맘대로' 총 샷 제한

# 로봇에 장착된 향료 목록 (frontend/app.js SCENTS와 동일하게 유지)
VALID_SCENTS = {
    "top": ["Citrus", "Green"],
    "middle": ["Floral", "Woody"],
    "base": ["Musk", "Amber"],
}
ALL_SCENTS = {s for layer in VALID_SCENTS.values() for s in layer}

# 로봇은 한 번에 하나의 제조만 수행 가능
robot_lock = threading.Lock()

# 관리자 키오스크 잠금 — True면 손님 주문을 받지 않는다 (점검/향료 교체용)
kiosk_locked = False

# ---- 관리자 인증 ----
ADMIN_PASSWORD = os.environ.get("ADMIN_PASSWORD", "0609")  # 기본 PIN = 로봇 모델명
ADMIN_SESSION_MAX_AGE_SEC = 1800  # 로그인 유지 시간 30분 (키오스크에 열어두고 방치 대비)

app = Flask(__name__, static_folder=None)
# 세션 서명 키 — 서버 재시작 시마다 새로 생성되므로 재시작하면 재로그인 필요 (의도된 동작)
app.secret_key = secrets.token_hex(16)


def _admin_authed():
    """관리자 세션이 유효한지 (로그인했고 30분이 지나지 않았는지) 확인."""
    logged_in_at = session.get("admin_at")
    return logged_in_at is not None and (time.time() - logged_in_at) < ADMIN_SESSION_MAX_AGE_SEC


@app.before_request
def _guard_admin_api():
    """모든 /api/admin/* 요청을 한 곳에서 인증 검사 (login만 예외)."""
    if request.path.startswith("/api/admin/") and request.path != "/api/admin/login":
        if not _admin_authed():
            return jsonify({"status": "error", "message": "로그인이 필요합니다."}), 401


def get_db():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn


# ---------- DB 초기화 ----------
# 추천 카드 3종 시드 (top/mid/base 배율 합 = 100)
# 향료 이름은 VALID_SCENTS에 있는 실제 향료여야 한다 — 제조 시 이 이름 그대로
# robot_control.SCENT_ORDER 슬롯(Order.srv scent1~6)에 매핑되기 때문.
SEED_RECIPES = [
    ("상쾌한 아침", "맑은 날 · 시트러스 계열의 산뜻한 조합",
     "Citrus", "Floral", "Musk", 50, 30, 20),
    ("포근한 오후", "흐린 날 · 플로럴 중심의 부드러운 조합",
     "Green", "Floral", "Amber", 20, 50, 30),
    ("깊은 밤", "비 오는 날 · 우디/머스크의 묵직한 조합",
     "Green", "Woody", "Musk", 20, 30, 50),
]


def init_db():
    """schema.sql 실행 + 추천 레시피 시드."""
    os.makedirs(DB_DIR, exist_ok=True)
    with open(SCHEMA_PATH, "r", encoding="utf-8") as f:
        schema = f.read()

    conn = sqlite3.connect(DB_PATH)
    try:
        conn.executescript(schema)
        conn.executemany(
            "INSERT INTO recommend_recipes "
            "(recipe_name, description, top_scent, mid_scent, base_scent, "
            " top_ratio, mid_ratio, base_ratio) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
            SEED_RECIPES,
        )
        conn.commit()
        print(f"[init_db] DB 생성 완료: {DB_PATH}")
        print(f"[init_db] 추천 레시피 {len(SEED_RECIPES)}건 시드 완료")
    finally:
        conn.close()


# ---------- 프론트엔드 서빙 ----------
@app.route("/")
def index():
    return send_from_directory(FRONTEND_DIR, "index.html")


@app.route("/<path:filename>")
def frontend_files(filename):
    return send_from_directory(FRONTEND_DIR, filename)


# ---------- API ----------
@app.route("/api/recipes", methods=["GET"])
def get_recipes():
    conn = get_db()
    try:
        rows = conn.execute(
            "SELECT id, recipe_name, description, top_scent, mid_scent, base_scent, "
            "top_ratio, mid_ratio, base_ratio "
            "FROM recommend_recipes ORDER BY id"
        ).fetchall()
    finally:
        conn.close()
    return jsonify([dict(r) for r in rows])


def _plan_from_recommend(recipe_id):
    """추천 조합: DB 배율(%)을 총 샷 수로 환산."""
    conn = get_db()
    try:
        row = conn.execute(
            "SELECT recipe_name, top_scent, mid_scent, base_scent, "
            "top_ratio, mid_ratio, base_ratio "
            "FROM recommend_recipes WHERE id = ?",
            (recipe_id,),
        ).fetchone()
    finally:
        conn.close()

    if row is None:
        return None, None

    plan = [
        {"scent": row["top_scent"],  "shots": round(row["top_ratio"]  / 100 * RECOMMEND_TOTAL_SHOTS)},
        {"scent": row["mid_scent"],  "shots": round(row["mid_ratio"]  / 100 * RECOMMEND_TOTAL_SHOTS)},
        {"scent": row["base_scent"], "shots": round(row["base_ratio"] / 100 * RECOMMEND_TOTAL_SHOTS)},
    ]
    plan = [p for p in plan if p["shots"] > 0]
    return row["recipe_name"], plan


def _plan_from_custom(selections):
    """나만의 조합: 선택된 향료마다 고정 샷."""
    if not isinstance(selections, dict):
        return None, None
    plan = []
    for layer in ("top", "middle", "base"):
        scents = selections.get(layer, [])
        if not isinstance(scents, list):
            return None, None
        for scent in scents:
            if scent not in ALL_SCENTS:
                return None, None
            plan.append({"scent": scent, "shots": CUSTOM_SHOTS_PER_SCENT})
    return "나만의 조합", plan


def _plan_from_free(slots):
    """내맘대로: 슬롯 값 그대로, 합계 검증."""
    if not isinstance(slots, list):
        return None, None
    plan = []
    for slot in slots:
        if not isinstance(slot, dict):
            return None, None
        scent = slot.get("scent")
        try:
            shots = int(slot.get("shots", 0))
        except (TypeError, ValueError):
            return None, None
        if scent and scent not in ALL_SCENTS:
            return None, None
        if scent and shots > 0:
            plan.append({"scent": scent, "shots": shots})
    return "내맘대로 조합", plan


@app.route("/api/kiosk_status", methods=["GET"])
def kiosk_status():
    """손님 화면이 시작 시점에 잠금 여부를 확인하는 용도."""
    return jsonify({"locked": kiosk_locked})


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
    """관리자 화면이 1초마다 폴링하는 통합 상태."""
    status = robot_control.get_status()
    status["kiosk_locked"] = kiosk_locked
    return jsonify(status)


@app.route("/api/admin/estop", methods=["POST"])
def admin_estop():
    """비상 정지 — 제조 중이든 아니든 즉시 실행한다."""
    result = robot_control.estop()
    return jsonify(result), 200 if result["status"] == "success" else 502


@app.route("/api/admin/home", methods=["POST"])
def admin_home():
    """홈 자세 복귀. 제조와 모션이 겹치면 위험하므로 제조 중에는 거절."""
    if not robot_lock.acquire(blocking=False):
        return jsonify({"status": "error",
                        "message": "제조 진행 중에는 홈 복귀를 할 수 없습니다."}), 409
    try:
        result = robot_control.move_home()
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


@app.route("/api/make_perfume", methods=["POST"])
def make_perfume():
    if kiosk_locked:
        return jsonify({"status": "error",
                        "message": "키오스크가 점검 중입니다. 잠시 후 이용해주세요."}), 503

    data = request.get_json(silent=True)
    if not data or "mode" not in data:
        return jsonify({"status": "error", "message": "잘못된 요청 형식입니다."}), 400

    mode = data["mode"]

    if mode == "recommend":
        recipe_id = data.get("recipe_id")
        if recipe_id is None:
            return jsonify({"status": "error", "message": "recipe_id가 없습니다."}), 400
        recipe_name, plan = _plan_from_recommend(recipe_id)
        if plan is None:
            return jsonify({"status": "error", "message": "존재하지 않는 레시피입니다."}), 404

    elif mode == "custom":
        recipe_name, plan = _plan_from_custom(data.get("selections", {}))
        if plan is None:
            return jsonify({"status": "error", "message": "잘못된 향료 선택입니다."}), 400

    elif mode == "free":
        recipe_name, plan = _plan_from_free(data.get("slots", []))
        if plan is None:
            return jsonify({"status": "error", "message": "잘못된 향료 선택입니다."}), 400

    else:
        return jsonify({"status": "error", "message": f"알 수 없는 모드: {mode}"}), 400

    if not plan:
        return jsonify({"status": "error", "message": "선택된 향료가 없습니다."}), 400

    total = sum(p["shots"] for p in plan)
    if total > FREE_MAX_SHOTS and mode == "free":
        return jsonify({"status": "error",
                        "message": f"총 토출 횟수는 최대 {FREE_MAX_SHOTS}샷까지 가능합니다."}), 400

    # 로봇은 한 번에 하나만 제조 가능 — 제조 중이면 새 요청 거절.
    # 아래 dispense() 호출은 ROS2로 실제 로봇에 제조를 요청하고, 로봇이
    # 제조를 전부 마쳐서 응답을 보낼 때까지 이 자리에서 대기(블로킹)한다.
    if not robot_lock.acquire(blocking=False):
        return jsonify({"status": "error",
                        "message": "이미 제조가 진행 중입니다. 잠시 후 다시 시도해주세요."}), 409
    try:
        result = robot_control.dispense(recipe_name, plan)
    finally:
        robot_lock.release()

    if result["status"] != "success":
        # 로봇 미연결/제조 실패 등 — 상태 코드만 다르게, 형식은 다른 에러 응답과 동일
        return jsonify(result), 502

    result["plan"] = plan
    return jsonify(result), 200


def main():
    if not os.path.exists(DB_PATH):
        init_db()

    # 서버가 켜져 있는 동안 계속 쓸 ROS2 노드를 한 번만 초기화하고,
    # 프로세스가 종료될 때(atexit) 정리한다.
    robot_control.init_ros()
    atexit.register(robot_control.shutdown_ros)

    debug = os.environ.get("FLASK_DEBUG", "0") == "1"
    # threaded=True: 제조 요청 하나가 오래 블로킹되는 동안에도(로봇이 제조하는 동안)
    # 다른 요청(정적 파일, 헬스체크 등)을 함께 처리할 수 있도록 함.
    app.run(host="0.0.0.0", port=5000, debug=debug, threaded=True)


if __name__ == "__main__":
    main()
