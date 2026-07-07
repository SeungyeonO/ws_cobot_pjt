# ws_cobot1

두산 협동로봇 M0609 기반 **조향(향수) 자동화 솔루션** ROS2 워크스페이스.
손님이 키오스크 화면에서 향료 조합을 고르면, Flask 백엔드가 ROS2 서비스로
로봇 제어 노드에 제조를 요청하고, 로봇이 실제로 향료를 토출해 완료한다.

## 워크스페이스 구조

```
ws_cobot1/
├── doc/                     # 문서 (이 파일)
├── env.sh                   # 개발 환경 source 스크립트
└── src/
    ├── perfume_order_srv/   # ROS2 인터페이스 패키지 (ament_cmake)
    │   └── srv/Order.srv    # 제조 요청 서비스: scent1~scent6(int8) → bool success
    └── perfume_backend/     # Flask 백엔드 + 프론트엔드 (ament_python, ROS2 노드)
        ├── perfume_backend/ # app.py(Flask), robot_control.py(ROS2 클라이언트), schema.sql
        └── frontend/        # 손님 키오스크 + 관리자 HMI (html/css/js)
```

## 의존 워크스페이스: `ws_dsr`

`perfume_backend`는 두산 로봇 메시지/서비스 정의(`dsr_msgs2`, joint state, error 토픽 등)를
위해 별도 워크스페이스 **`ws_dsr`**(두산 공식 `doosan-robot2` 드라이버 스택 등)에 의존한다.
`dsr_msgs2`는 `doosan-robot2` 저장소의 일부이고 그 안의 다른 패키지(`dsr_hardware2`,
`dsr_controller2`, `dsr_bringup2` 등)도 직접 의존하므로 `ws_cobot1`으로 옮기지 않는다.
드라이버(`ws_dsr`)를 underlay로, 애플리케이션(`ws_cobot1`)을 overlay로 두는 표준적인
ROS2 다중 워크스페이스 구성이다.

## 빌드

```bash
source /opt/ros/humble/setup.bash
cd ws_cobot1
colcon build
```

## 실행

워크스페이스 루트에서, 매 터미널마다 환경을 한 번 불러온 뒤 실행한다.

```bash
cd ws_cobot1
source env.sh   # humble + ws_dsr + ws_cobot1 순서로 source
ros2 run perfume_backend perfume_backend
```

- 손님 키오스크: `http://localhost:5000/`
- 관리자 HMI: `http://localhost:5000/admin` (PIN, 환경변수 `ADMIN_PASSWORD` 기본값 `0609`)
- DB: `~/.ros/perfume/perfume.db` (없으면 최초 실행 시 `schema.sql` + 추천 레시피 3종으로 자동 생성)

## 로봇 제어부와의 계약

`perfume_backend`는 `perfume_order_srv/srv/Order` 서비스(`/dsr01/perfume/order`)의
**클라이언트**다. 실제로 향료병을 옮기고 토출하는 동작은 이 워크스페이스가 아니라
별도 로봇 제어 노드(예: `ws_dsr/src/rokey`)가 그 서비스의 **서버**로 구현해야 한다.

- 요청 필드(`scent1~scent6`)는 이름이 없는 정수 6개라, 향료 이름 ↔ 슬롯 번호 매핑은
  코드 밖에서 양쪽이 합의해야 하는 계약이다. `perfume_backend`는
  `robot_control.SCENT_ORDER = ["Citrus", "Green", "Floral", "Woody", "Musk", "Amber"]`
  순서를 쓰므로, 로봇 제어 노드의 향료병 물리 배치도 이 순서와 일치해야 한다.
- 응답은 `bool success` 하나뿐이라 실패 사유를 전달할 방법이 없다. 로봇 제어 노드는
  "요청 접수" 시점이 아니라 **실제 제조를 전부 마친 뒤에만** 응답해야 한다 — 이 응답
  도착 시점이 곧 프론트엔드의 '제조중' 화면이 끝나는 시점이기 때문이다.

## 참고

- 향료 6종(`Citrus`/`Green`/`Floral`/`Woody`/`Musk`/`Amber`)과 레이어(top/middle/base)
  구성은 `perfume_backend/app.py`의 `VALID_SCENTS`가 기준이며, 프론트엔드
  `frontend/app.js`의 `SCENTS`와 항상 동일하게 유지해야 한다.
