"""조향 자동화 솔루션 키오스크 백엔드 (키오스크 PC에서 실행).

주문 접수(레시피 계산)를 담당하고, 아래 두 채널로 로봇 제어부와 통신한다.
브라우저는 항상 이 서버하고만 통신하므로(상대 경로 fetch 그대로) 프론트엔드
코드 변경이 필요 없다.

- HMI 백엔드(perfume_hmi, 로봇 제어 PC): 잠금 상태(kiosk_locked) 조회만
  HTTP로. admin이 키오스크를 잠그면 여기로 반영된다.
- 로봇 제어부 주문 서비스: 실제 제조(dispense) 요청은 ROS2 서비스로 직접
  보낸다. 로봇 네임스페이스 없이 서비스 이름 "/order_perfume", 타입
  perfume_order_srv/srv/Order (scent1~scent6 샷 수 요청 → success 응답).
  Flask(werkzeug) 개발 서버는 동기식이라 ROS2 spin은 전용 백그라운드
  스레드에서 돌리고(init_ros), order_perfume()은 call_async 후
  future.done()을 폴링해 블로킹 호출처럼 동작하게 만든다.

- GET  /api/recipes        추천 조합 목록 (SQLite3 동적 로드)
- POST /api/make_perfume   제조 요청 (배율/횟수 계산 후 /order_perfume 호출)
- GET  /api/kiosk_status   키오스크 잠금 여부 (HMI 백엔드에서 조회, 손님 화면이 점검 중 표시에 사용)
- /                        프론트엔드 서빙
- DB 초기화: 서버 시작 시 DB_PATH(~/.perfume/kiosk.db)가 없으면
  schema.sql + 시드로 자동 생성 (재초기화하려면 그 파일 삭제 후 재시작)
- 설정(환경변수):
  - HMI_BASE_URL, HMI_API_KEY: perfume_hmi 쪽 값과 반드시 동일해야 함

[제조 흐름 요약]
프론트 "제조하기" 클릭 → POST /api/make_perfume → order_perfume()이
/order_perfume ROS2 서비스를 호출해 로봇이 실제로 다 만들 때까지 블로킹 →
그 결과를 JSON으로 돌려준다. 프론트는 이 fetch가 끝날 때까지 '제조중' 화면을
유지하기만 하면 된다.

[실행] pip로 설치하지만 /order_perfume 호출을 위해 ROS2(rclpy)와
perfume_order_srv 인터페이스가 필요하다 — 먼저 perfume_order_srv를
colcon build 한 뒤 아래 순서로 source하고 실행한다:
  source /opt/ros/humble/setup.bash
  source <ws_cobot1>/install/setup.bash
  pip install -e .
  HMI_BASE_URL=http://<HMI-PC-IP>:5000 HMI_API_KEY=<키> perfume_kiosk
"""
import atexit
import os
import sqlite3
import threading
import time
from pathlib import Path

import requests
import rclpy
from flask import Flask, jsonify, request, send_from_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from perfume_order_srv.srv import Order

# __file__ = 이 app.py 자신의 경로. perfume_hmi처럼 ament_index_python으로
# "설치된 위치"를 찾을 ROS2 빌드시스템이 없으므로(순수 pip 패키지), __file__
# 기준 상대경로로 직접 찾는다 — pip install -e .(editable install)면 실행
# 위치(cwd)가 어디든 항상 소스 트리 안의 frontend/를 정확히 가리킨다.
# .parent        = perfume_kiosk/perfume_kiosk/ (이 파일이 있는 패키지 폴더)
# .parent.parent = perfume_kiosk/                (패키지 루트) → 그 아래 frontend/
FRONTEND_DIR = Path(__file__).resolve().parent.parent / "frontend"
# schema.sql은 app.py와 같은 폴더(perfume_kiosk/perfume_kiosk/)에 있으므로 한 단계만 올라간다.
SCHEMA_PATH = Path(__file__).resolve().parent / "schema.sql"

# DB는 소스 트리 밖, 홈 디렉터리 아래 별도 위치에 둔다 (ROS2 워크스페이스와 무관).
# 코드(소스 트리)와 데이터(실행 중 쌓이는 상태)를 분리해서, pip install -e .를
# 다시 하거나 소스를 갱신해도 DB가 지워지지 않고 git에도 안 들어가게 한다.
DB_DIR = os.path.expanduser("~/.perfume")
DB_PATH = os.path.join(DB_DIR, "kiosk.db")

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

# ---- HMI 백엔드 (perfume_hmi) — 잠금 상태 조회 전용 ----
HMI_BASE_URL = os.environ.get("HMI_BASE_URL", "http://localhost:5000")
HMI_API_KEY = os.environ.get("HMI_API_KEY", "perfume-internal-key")  # perfume_hmi와 동일해야 함


# ==============================================================
# 로봇 제어부 주문 서비스 (ROS2) — /order_perfume 클라이언트
# ==============================================================

# 로봇 네임스페이스 없이 통신
ORDER_SERVICE_NAME = "/order_perfume"

# 로봇 물리 슬롯(밸브) 순서 — scent1~scent6과 1:1 고정 매핑.
# 위 VALID_SCENTS(top/middle/base)와 항상 동일하게 유지할 것.
SCENT_SLOT_ORDER = ["Citrus", "Green", "Floral", "Woody", "Musk", "Amber"]

POLL_INTERVAL_SEC = 0.1        # 응답 도착 여부를 확인하는 폴링 주기(초)
ORDER_TIMEOUT_SEC = 310.0      # 실제 제조 시간(최대 5분 가정)보다 여유 있게

# 모듈 전역 상태 — init_ros()에서 한 번만 채워진다.
_node = None
_executor = None
_spin_thread = None
_order_client = None


def init_ros():
    """Flask 앱 시작 시 딱 한 번 호출한다. 이미 초기화되어 있으면 아무 것도 하지 않는다."""
    global _node, _executor, _spin_thread, _order_client
    if _node is not None:
        return

    rclpy.init()
    _node = Node("perfume_kiosk_client")
    _order_client = _node.create_client(Order, ORDER_SERVICE_NAME)

    _executor = MultiThreadedExecutor()
    _executor.add_node(_node)
    # 서브 스레드에서 spin()을 돌려 Flask 개발 서버 블로킹될 시 ROS2 서비스 응답 가능하게
    _spin_thread = threading.Thread(target=_executor.spin, daemon=True)
    _spin_thread.start()

    _node.get_logger().info(f"[perfume_kiosk] ROS2 준비 완료 ({ORDER_SERVICE_NAME} 클라이언트)")


def shutdown_ros():
    """Flask 앱 종료 시 ROS2 자원을 정리한다."""
    global _node, _executor, _spin_thread, _order_client
    if _executor is not None:
        _executor.shutdown()
    if _spin_thread is not None:
        _spin_thread.join(timeout=2.0)
    if _node is not None:
        _node.destroy_node()
    if _node is not None or _executor is not None:
        rclpy.shutdown()
    _node = _executor = _spin_thread = _order_client = None


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


def order_perfume(plan):
    """plan([{"scent": str, "shots": int}, ...])을 /order_perfume 요청으로 보낸다.

    반환: {"success": bool, "message": str}
    """
    if _order_client is None:
        return {"success": False, "message": "ROS2가 초기화되지 않았습니다."}
    if not _order_client.wait_for_service(timeout_sec=2.0):
        return {"success": False,
                "message": f"로봇 제어부 주문 서비스({ORDER_SERVICE_NAME})에 연결할 수 없습니다."}

    shots_by_scent = {p["scent"]: p["shots"] for p in plan}
    req = Order.Request()
    for i, scent in enumerate(SCENT_SLOT_ORDER, start=1):
        setattr(req, f"scent{i}", shots_by_scent.get(scent, 0))

    _node.get_logger().info(f"[order] {ORDER_SERVICE_NAME} 호출: {plan}")
    response = _wait_future(_order_client.call_async(req), ORDER_TIMEOUT_SEC)
    if response is None:
        return {"success": False, "message": "제조 요청이 시간 초과되었습니다."}
    if not response.success:
        return {"success": False, "message": "로봇 제어부가 제조에 실패했습니다."}
    return {"success": True, "message": "제조 완료"}


# ==============================================================
# Flask 앱
# ==============================================================

app = Flask(__name__, static_folder=None)


def get_db():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn


# ---------- DB 초기화 ----------
# 추천 카드 3종 시드 (top/mid/base 배율 합 = 100)
# 향료 이름은 VALID_SCENTS에 있는 실제 향료여야 한다 — 제조 시 이 이름 그대로
# 로봇 제어부의 향료 슬롯에 매핑되기 때문.
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
    """손님 화면이 시작/점검중 화면에서 잠금 여부를 확인하는 용도.

    HMI 백엔드가 kiosk_locked의 단일 소스이므로 매번 그쪽에 물어본다.
    """
    try:
        res = requests.get(
            f"{HMI_BASE_URL}/internal/lock_status",
            headers={"X-HMI-Api-Key": HMI_API_KEY},
            timeout=3.0,
        )
        res.raise_for_status()
    except requests.RequestException:
        return jsonify({"status": "error", "message": "HMI 서버와 통신할 수 없습니다."}), 502
    return jsonify({"locked": bool(res.json().get("locked"))})


@app.route("/api/make_perfume", methods=["POST"])
def make_perfume():
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

    # 실제 제조는 로봇 제어부의 ROS2 서비스(/order_perfume)에 위임 — 응답이
    # 올 때까지 블로킹된다 (order_perfume() 참고).
    result = order_perfume(plan)
    if not result["success"]:
        return jsonify({"status": "error", "message": result["message"]}), 502

    return jsonify({
        "status": "success",
        "recipe_name": recipe_name,
        "plan": plan,
        "total_shots": total,
    })


def main():
    if not os.path.exists(DB_PATH):
        init_db()

    # 서버가 켜져 있는 동안 계속 쓸 ROS2 노드를 한 번만 초기화하고,
    # 프로세스가 종료될 때(atexit) 정리한다.
    init_ros()
    atexit.register(shutdown_ros)

    debug = os.environ.get("FLASK_DEBUG", "0") == "1"
    app.run(host="0.0.0.0", port=5000, debug=debug, threaded=True)


if __name__ == "__main__":
    main()
