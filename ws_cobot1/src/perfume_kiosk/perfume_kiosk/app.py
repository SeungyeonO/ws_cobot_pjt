"""조향 자동화 솔루션 키오스크 백엔드 (키오스크 PC에서 실행).

주문 접수(레시피 계산)를 담당하고, 아래 두 채널로 로봇 제어부와 통신한다.
브라우저는 항상 이 서버하고만 통신하므로(상대 경로 fetch 그대로) 프론트엔드
코드 변경이 필요 없다.

- HMI 백엔드(perfume_hmi, 로봇 제어 PC): 잠금 상태(kiosk_locked) 조회만
  HTTP로. admin이 키오스크를 잠그면 여기로 반영된다.
- 로봇 제어부 주문 서비스: 실제 제조(dispense) 요청은 ROS2 서비스로 직접
  보낸다. 로봇 네임스페이스 없이 서비스 이름 "/order_perfume", 타입
  perfume_order_srv/srv/Order (scent1~scent6 샷 수 요청).
  제조 완료 여부는 서비스 응답(success)이 아니라 별도 토픽
  ORDER_DONE_TOPIC(std_msgs/Bool)으로 true가 오면 끝난 것으로 판정한다.
  ROS2 통신(노드·서비스 클라이언트·완료 토픽 구독·spin 스레드)은
  RobotOrderClient 클래스 하나에 묶여 있다. Flask(werkzeug) 개발 서버는
  동기식이라 spin은 전용 백그라운드 스레드에서 돌리고, order()는 서비스로
  주문만 전달한 뒤 완료 토픽 이벤트를 기다려 블로킹 호출처럼 동작한다.

- GET  /api/recipes        추천 조합 목록 (SQLite3 동적 로드)
- POST /api/make_perfume   제조 요청 (배율/횟수 계산 후 /order_perfume 호출)
- GET  /api/kiosk_status   키오스크 잠금 여부 (HMI 백엔드에서 조회, 손님 화면이 점검 중 표시에 사용)
- /                        프론트엔드 서빙
- DB 초기화: 서버 시작 시 DB_PATH(~/.perfume/kiosk.db)가 없으면
  schema.sql + 시드로 자동 생성 (재초기화하려면 그 파일 삭제 후 재시작)
- 설정(환경변수):
  - HMI_BASE_URL, HMI_API_KEY: perfume_hmi 쪽 값과 반드시 동일해야 함

[제조 흐름 요약]
프론트 "제조하기" 클릭 → POST /api/make_perfume → RobotOrderClient.order()가
/order_perfume ROS2 서비스로 주문을 전달 → 로봇이 제조를 마치고
ORDER_DONE_TOPIC으로 true를 publish할 때까지 블로킹 → 그 결과를 JSON으로
돌려준다. 프론트는 이 fetch가 끝날 때까지 '제조중' 화면을 유지하기만 하면 된다.

[실행] perfume_hmi와 동일한 ament_python 패키지 — colcon build 후 ros2 run으로 실행한다:
  source /opt/ros/humble/setup.bash
  cd <ws_cobot1> && colcon build --packages-select perfume_order_srv perfume_kiosk
  source <ws_cobot1>/install/setup.bash
  HMI_BASE_URL=http://<HMI-PC-IP>:5000 HMI_API_KEY=<키> ros2 run perfume_kiosk perfume_kiosk
"""
import atexit
import os
import sqlite3
import threading
from pathlib import Path

import requests
import rclpy
from ament_index_python.packages import get_package_share_directory
from flask import Flask, jsonify, request, send_from_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

from perfume_order_srv.srv import Order

# perfume_hmi와 동일하게 ament_index_python으로 "설치된 위치"의 share 폴더를
# 찾는다 (setup.py data_files가 frontend/를 share/perfume_kiosk/frontend에 설치).
PACKAGE_SHARE_DIR = get_package_share_directory("perfume_kiosk")
FRONTEND_DIR = os.path.join(PACKAGE_SHARE_DIR, "frontend")
# schema.sql은 setup.py package_data로 app.py와 같은 폴더에 설치되므로 __file__ 기준 그대로.
SCHEMA_PATH = Path(__file__).resolve().parent / "schema.sql"

# DB는 소스 트리 밖, 홈 디렉터리 아래 별도 위치에 둔다 (ROS2 워크스페이스와 무관).
# 코드(소스 트리)와 데이터(실행 중 쌓이는 상태)를 분리해서, colcon build를
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
# HMI 실행 pc ip 주소와 perfume_hmi에서 설정한 api key를 반드시 동일하게 통일
HMI_BASE_URL = os.environ.get("HMI_BASE_URL", "http://172.23.0.195:5000")
HMI_API_KEY = os.environ.get("HMI_API_KEY", "perfume-internal-key")  # perfume_hmi와 동일해야 함


# ==============================================================
# 로봇 제어부 주문 서비스 (ROS2) — /order_perfume 클라이언트
# ==============================================================

# 로봇 네임스페이스 없이 통신
ORDER_SERVICE_NAME = "/order_perfume"

# 제조 완료 신호 토픽 (std_msgs/Bool) — 로봇 제어부가 제조를 마치면 true를
# publish한다. 로봇 쪽 퍼블리셔 토픽 이름과 반드시 동일해야 함.
ORDER_DONE_TOPIC = "/perfume_done"

# 로봇 물리 슬롯(밸브) 순서 — scent1~scent6과 1:1 고정 매핑.
# 위 VALID_SCENTS(top/middle/base)와 항상 동일하게 유지할 것.
SCENT_SLOT_ORDER = ["Citrus", "Green", "Floral", "Woody", "Musk", "Amber"]

ORDER_TIMEOUT_SEC = 310.0      # 실제 제조 시간(최대 5분 가정)보다 여유 있게

class RobotOrderClient:
    """로봇 제어부와의 ROS2 통신을 한 덩어리로 묶은 클라이언트.

    - /order_perfume 서비스: 주문 전달 (응답 success는 완료 판정에 쓰지 않음)
    - ORDER_DONE_TOPIC 토픽: 제조 완료 신호(true) 수신
    Flask 앱 시작 시 하나만 만들어(main() 참고) 서버가 켜져 있는 동안 재사용한다.
    """

    def __init__(self): 
        
        # 클래스 내에서 ROS2 초기화, 노드 생성, 서비스 클라이언트 생성, 토픽 구독, spin 스레드 시작
        
        rclpy.init()
        # 노드 생성
        self._node = Node("perfume_kiosk_client")
        # 서비스 클라이언트 생성
        self._order_client = self._node.create_client(Order, ORDER_SERVICE_NAME)
        # 제조 완료 신호 구독 — 콜백은 spin 스레드에서 실행된다.
        self._done_sub = self._node.create_subscription(
            Bool, ORDER_DONE_TOPIC, self._on_done, 10)
        # 완료 신호 수신 이벤트 — 콜백(spin 스레드)이 set하고, order()(Flask 요청 스레드)가 wait한다.
        # _done_result는 마지막으로 받은 신호 값 (true=제조 성공, false=제조 실패).
        self._done_event = threading.Event()
        self._done_result = False

        # 동기화 / 병렬처리 세팅
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        # 서브 스레드에서 spin()을 돌려 Flask 개발 서버 블로킹될 시 ROS2 콜백 처리 가능하게
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()
        # 문제 없을 시 ROS2 준비 완료 로그
        self._node.get_logger().info(
            f"[perfume_kiosk] ROS2 준비 완료 ({ORDER_SERVICE_NAME} 클라이언트, {ORDER_DONE_TOPIC} 구독)")

    def _on_done(self, msg):
        """ORDER_DONE_TOPIC 콜백 — true면 제조 성공, false면 제조 실패.

        어느 쪽이든 신호가 온 것이므로 결과를 기록하고 wait 중인 스레드를 깨운다.
        """
        self._done_result = bool(msg.data)
        self._done_event.set()

    def order(self, plan):
        """향료별 샷 수 계획(plan)을 서비스로 전달하고, 제조 결과는
        ORDER_DONE_TOPIC으로 true(성공)/false(실패)가 publish될 때까지 기다려 판정한다.

        성공하면 {"success": True, "message": "제조 완료"}, 실패하면 {"success": False, "message": "..."}
        (로봇이 false를 보낸 실패는 "robot_failed": True가 함께 담긴다.)
        """
        if not self._order_client.wait_for_service(timeout_sec=2.0):
            return {"success": False,
                    "message": f"로봇 제어부 주문 서비스({ORDER_SERVICE_NAME})에 연결할 수 없습니다."}

        shots_by_scent = {p["scent"]: p["shots"] for p in plan}
        req = Order.Request()
        for i, scent in enumerate(SCENT_SLOT_ORDER, start=1):
            setattr(req, f"scent{i}", shots_by_scent.get(scent, 0))

        # 이전 주문의 완료 신호가 남아 있지 않도록 반드시 주문 전에 초기화한다.
        self._done_event.clear()

        # 서비스는 주문 전달 용도로만 호출한다 — 응답(success)은 완료 판정에 쓰지 않는다.
        self._node.get_logger().info(f"[order] {ORDER_SERVICE_NAME} 호출: {plan}")
        self._order_client.call_async(req)

        # 주문 후 제조 완료 신호가 올 때까지 블로킹 — Flask 요청 스레드에서 wait()한다.
        if not self._done_event.wait(timeout=ORDER_TIMEOUT_SEC):
            return {"success": False,
                    "message": f"제조 완료 신호({ORDER_DONE_TOPIC})가 시간 초과되었습니다."}
        if not self._done_result:
            # 로봇이 false를 publish — 제조 도중 실패 (프론트가 실패 화면을 띄운다)
            return {"success": False, "robot_failed": True,
                    "message": "로봇이 제조에 실패했습니다. 잠시 후 다시 시도해주세요."}
        return {"success": True, "message": "제조 완료"}

    def shutdown(self):
        """Flask 앱 종료 시 ROS2 자원을 정리한다."""
        self._executor.shutdown()
        self._spin_thread.join(timeout=2.0)
        self._node.destroy_node()
        rclpy.shutdown()


# 앱 전체에서 하나만 쓰는 로봇 클라이언트 — main()에서 생성된다.
_robot = None


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


# ---------- DB 초기화 ----------
def init_db():
    """schema.sql 실행 + 추천 레시피 시드."""
    os.makedirs(DB_DIR, exist_ok=True)
    with open(SCHEMA_PATH, "r", encoding="utf-8") as f:
        schema = f.read()

    conn = sqlite3.connect(DB_PATH)
    # schema.sql 실행 후 추천 레시피 시드
    try:
        conn.executescript(schema)
        conn.executemany(
            "INSERT INTO recommend_recipes "
            "(recipe_name, description, top_scent, mid_scent, base_scent, "
            " top_ratio, mid_ratio, base_ratio) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
            SEED_RECIPES,
        )
        # DB 변경 사항 커밋
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
    # DB 조회
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
    
    # DB 배율(%)을 총 샷 수로 환산 — 소수점 반올림, 0샷이면 제외
    plan = [
        {"scent": row["top_scent"],  "shots": round(row["top_ratio"]  / 100 * RECOMMEND_TOTAL_SHOTS)},
        {"scent": row["mid_scent"],  "shots": round(row["mid_ratio"]  / 100 * RECOMMEND_TOTAL_SHOTS)},
        {"scent": row["base_scent"], "shots": round(row["base_ratio"] / 100 * RECOMMEND_TOTAL_SHOTS)},
    ]
    # 0샷 향료는 제외 (DB 배율이 0%인 경우)
    plan = [p for p in plan if p["shots"] > 0]
    return row["recipe_name"], plan


def _plan_from_custom(selections):
    """나만의 조합: Top/Middle/Base 레이어마다 향료 정확히 1개씩, 향료당 고정 샷.

    노트 (탑, 미들, 베이스) 당 1개 제한은 프론트엔드(라디오 버튼)도 걸지만, 총 샷 수가 여기서
    결정되므로(3개 × CUSTOM_SHOTS_PER_SCENT) 서버에서도 검증한다.
    """
    if not isinstance(selections, dict):
        return None, None
    plan = []
    for layer in ("top", "middle", "base"):
        scents = selections.get(layer, [])
        # 프론트엔드에서 라디오 버튼으로 1개만 선택하도록 했지만, 혹시라도 여러 개가 들어오면 무효 처리
        if not isinstance(scents, list) or len(scents) != 1:
            return None, None
        # 선택된 향료가 실제 장착된 향료인지 검증
        scent = scents[0] # 프론트엔드에서 라디오 버튼으로 1개만 선택하도록 했지만, 혹시라도 여러 개가 들어오면 무효 처리
        if scent not in VALID_SCENTS[layer]:
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
    
    # 요청 형식 검증
    if not data or "mode" not in data:
        return jsonify({"status": "error", "message": "잘못된 요청 형식입니다."}), 400

    mode = data["mode"]

    # 사용자 모드 선택에 따라 제조 계획(plan)을 계산한다.
    if mode == "recommend": # 오늘의 추천 조합
        recipe_id = data.get("recipe_id")
        # DB에서 레시피 정보 조회 실패 시 404, recipe_id 누락 시 400
        if recipe_id is None:
            return jsonify({"status": "error", "message": "recipe_id가 없습니다."}), 400
        recipe_name, plan = _plan_from_recommend(recipe_id)
        if plan is None:
            return jsonify({"status": "error", "message": "존재하지 않는 레시피입니다."}), 404

    elif mode == "custom": # 나만의 조합
        recipe_name, plan = _plan_from_custom(data.get("selections", {}))
        if plan is None:
            return jsonify({"status": "error", "message": "잘못된 향료 선택입니다."}), 400

    elif mode == "free":  # 내맘대로 조합
        recipe_name, plan = _plan_from_free(data.get("slots", []))
        if plan is None:
            return jsonify({"status": "error", "message": "잘못된 향료 선택입니다."}), 400

    else:
        return jsonify({"status": "error", "message": f"알 수 없는 모드: {mode}"}), 400
    

    # 선택 향료 없을 시
    if not plan:
        return jsonify({"status": "error", "message": "선택된 향료가 없습니다."}), 400


    # 총 샷 수 검증 — '내맘대로' 모드만 제한, 나머지는 DB/고정값
    total = sum(p["shots"] for p in plan)
    if total > FREE_MAX_SHOTS and mode == "free":
        return jsonify({"status": "error",
                        "message": f"총 토출 횟수는 최대 {FREE_MAX_SHOTS}샷까지 가능합니다."}), 400

    # 실제 제조는 로봇 제어부의 ROS2 서비스(/order_perfume)에 위임 — 완료 신호
    # 토픽(ORDER_DONE_TOPIC)으로 true가 올 때까지 블로킹된다 (RobotOrderClient.order() 참고).
    if _robot is None:
        return jsonify({"status": "error", "message": "ROS2가 초기화되지 않았습니다."}), 502
    result = _robot.order(plan)
    if not result["success"]:
        return jsonify({"status": "error", "message": result["message"],
                        "robot_failed": result.get("robot_failed", False)}), 502

    return jsonify({
        "status": "success",
        "recipe_name": recipe_name,
        "plan": plan,
        "total_shots": total,
    })


def main():
    # Flask 앱 실행 전에 DB 초기화 + 로봇 클라이언트 생성
    global _robot
    if not os.path.exists(DB_PATH):
        init_db()

    # 서버가 켜져 있는 동안 계속 쓸 로봇 클라이언트를 한 번만 만들고,
    # 프로세스가 종료될 때(atexit) 정리한다.
    _robot = RobotOrderClient()
    atexit.register(_robot.shutdown)

    debug = os.environ.get("FLASK_DEBUG", "0") == "1"
    app.run(host="0.0.0.0", port=5000, debug=debug, threaded=True)


if __name__ == "__main__":
    main()
