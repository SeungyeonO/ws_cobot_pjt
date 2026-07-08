# ws_cobot1

두산 협동로봇 M0609 기반 **조향(향수) 자동화 솔루션** ROS2 워크스페이스.
손님이 키오스크 화면에서 향료 조합을 고르면, 키오스크 백엔드가 로봇 제어부
주문 서비스에 제조를 요청하고, 로봇이 실제로 향료를 토출해 완료한다.

## 배포 구조 (3-way)

로봇팔과 랜선으로 직접 연결되는 PC는 한 대뿐이다. 그 PC(로봇 제어 PC)에서
서로 다른 두 팀이 만드는 두 개의 독립된 프로세스가 함께 돈다.

- **로봇 제어 PC**
  - `perfume_hmi` (이 워크스페이스, 관리자 모니터링/대시보드 전용): 로봇
    연결 상태·조인트 각도·에러 로그를 보여주고, 비상 정지·홈 복귀·키오스크
    잠금 토글을 수행한다. **제조(dispense)는 다루지 않는다.**
  - 로봇 제어부 주문 서비스 (별도 팀이 개발 중, 이 저장소 밖): ROS2 서비스
    `/order_perfume`(로봇 네임스페이스 없음, 타입 `perfume_order_srv/srv/Order`)의
    서버 역할을 맡아 실제 로봇 모션을 수행한다. 완성되면 launch file로
    `perfume_hmi`와 함께 로봇 제어 PC에서 묶어 띄울 예정이다.
- **키오스크 PC**: `perfume_kiosk`(손님 화면)를 실행한다. pip 패키지지만
  `/order_perfume` 호출을 위해 ROS2(rclpy)와 `perfume_order_srv` 인터페이스가
  필요하다 — `ws_dsr`(두산 드라이버)까지는 필요 없다.

키오스크 백엔드는 두 채널로 로봇 제어부와 통신한다(브라우저는 항상 자기 PC의
백엔드하고만 통신 — CORS 불필요):
- 잠금 상태(`kiosk_locked`) 조회 → `perfume_hmi`의 `GET /internal/lock_status` (HTTP)
- 실제 제조 요청 → 로봇 제어부 주문 서비스의 `/order_perfume` (ROS2 서비스)

> **주의**: 로봇 제어부 주문 서비스(서버 쪽)는 아직 만들어지지 않았다.
> `perfume_kiosk`는 `/order_perfume` **클라이언트**만 구현되어 있다
> (`perfume_kiosk/app.py`의 `order_perfume()`) — 서버가 뜨기 전까지는
> `wait_for_service` 단계에서 연결 실패로 응답한다.

## 워크스페이스 구조

```
ws_cobot1/
├── doc/                     # 문서 (이 파일)
├── env.sh                   # 로봇 제어 PC 전용 개발 환경 source 스크립트
└── src/
    ├── perfume_order_srv/   # ROS2 인터페이스 패키지 (ament_cmake)
    │   └── srv/Order.srv    # 제조 요청 서비스 정의: scent1~scent6(int8) → bool success
    │                        # /order_perfume 서비스 이름으로 씀 (perfume_kiosk가 클라이언트,
    │                        #  로봇 제어부 주문 서비스가 서버)
    ├── perfume_hmi/         # HMI 백엔드 (ament_python, ROS2 노드) — 로봇 제어 PC용, 모니터링 전용
    │   ├── perfume_hmi/     # app.py(Flask, admin+internal API), robot_bridge.py(ROS2 클라이언트)
    │   └── frontend/        # 관리자 HMI (admin.html/js/css, admin_login.html)
    └── perfume_kiosk/       # 키오스크 백엔드 (pip 패키지) — 키오스크 PC용
        ├── perfume_kiosk/   # app.py(Flask + recipes + HMI 조회 + /order_perfume ROS2 클라이언트), schema.sql
        └── frontend/        # 손님 키오스크 (index.html/app.js/style.css)
```

## 의존 워크스페이스: `ws_dsr` (로봇 제어 PC만 해당)

`perfume_hmi`는 두산 로봇 메시지/서비스 정의(`dsr_msgs2`, joint state, error 토픽 등)를
위해 별도 워크스페이스 **`ws_dsr`**(두산 공식 `doosan-robot2` 드라이버 스택 등)에 의존한다.
`dsr_msgs2`는 `doosan-robot2` 저장소의 일부이고 그 안의 다른 패키지(`dsr_hardware2`,
`dsr_controller2`, `dsr_bringup2` 등)도 직접 의존하므로 `ws_cobot1`으로 옮기지 않는다.
드라이버(`ws_dsr`)를 underlay로, 애플리케이션(`ws_cobot1`)을 overlay로 두는 표준적인
ROS2 다중 워크스페이스 구성이다. 키오스크 PC는 `ws_dsr`은 필요 없지만, `/order_perfume`
호출을 위해 ROS2(rclpy)와 `ws_cobot1`의 `perfume_order_srv`는 필요하다.

## 빌드 & 실행

### 로봇 제어 PC (`perfume_hmi`)

```bash
source /opt/ros/humble/setup.bash
cd ws_cobot1
colcon build
source env.sh   # humble + ws_dsr + ws_cobot1 순서로 source
ADMIN_PASSWORD=<PIN> HMI_API_KEY=<키> ros2 run perfume_hmi perfume_hmi
```

- 관리자 HMI: `http://<이 PC의 IP>:5000/admin` (PIN, 환경변수 `ADMIN_PASSWORD` 기본값 `0609`)
- `HMI_API_KEY`: 키오스크 백엔드가 `/internal/lock_status` 호출 시 보내야 하는 공유 키
  (기본값 `perfume-internal-key` — 배포 시 반드시 바꿀 것, 키오스크 쪽과 동일해야 함)
- 로봇 제어부 주문 서비스는 별도 프로세스로 같은 PC에서 실행한다 (포트가
  5000과 겹치지 않게 할 것). 완성되면 이 실행 절차도 launch file로 통합 예정.

### 키오스크 PC (`perfume_kiosk`)

pip 패키지이지만 `/order_perfume` 호출을 위해 ROS2(rclpy)와
`perfume_order_srv` 인터페이스가 필요하다 (`ws_dsr`은 필요 없음).
`perfume_order_srv`를 colcon build 한 뒤 source하고 실행한다.

```bash
source /opt/ros/humble/setup.bash
cd ws_cobot1
colcon build --packages-select perfume_order_srv
source install/setup.bash

cd src/perfume_kiosk
pip install -e .
HMI_BASE_URL=http://<로봇 제어 PC의 IP>:5000 HMI_API_KEY=<키> perfume_kiosk
```

- 손님 키오스크: `http://<이 PC의 IP>:5000/`
- DB: `~/.perfume/kiosk.db` (없으면 최초 실행 시 `schema.sql` + 추천 레시피 3종으로 자동 생성)

### 배포 전 확인할 것

- `perfume_kiosk/frontend/app.js`의 `ADMIN_URL`을 로봇 제어 PC의 실제 주소로 변경
- `perfume_hmi/frontend/admin.html`, `admin_login.html`의 "키오스크로 →" 링크를
  키오스크 PC의 실제 주소로 변경
- 로봇 제어 PC 방화벽에서 5000 포트(HMI) 허용
- 로봇 제어부 주문 서비스가 `/order_perfume`(`perfume_order_srv/srv/Order`)
  서버를 로봇 네임스페이스 없이 떠 있어야 `perfume_kiosk`가 연결할 수 있다

## 참고

- 향료 6종(`Citrus`/`Green`/`Floral`/`Woody`/`Musk`/`Amber`)과 레이어(top/middle/base)
  구성은 `perfume_kiosk/app.py`의 `VALID_SCENTS`가 기준이며, 프론트엔드
  `frontend/app.js`의 `SCENTS`와 항상 동일하게 유지해야 한다. `/order_perfume`
  요청의 `scent1`~`scent6` 슬롯 순서는 `perfume_kiosk/app.py`의
  `SCENT_SLOT_ORDER`가 기준이며, 로봇 제어부 주문 서비스도 이 순서(밸브
  배치)를 따라야 한다.
- `perfume_hmi`의 "제조 현황" 표시는 현재 비어 있다 — 실제 제조를 별도
  프로세스가 수행하므로 이 패키지가 알 방법이 없기 때문이다. 로봇 제어부
  주문 서비스가 제조 상태를 ROS2 토픽 등으로 공개하면 `robot_bridge.py`에서
  구독해 채워 넣을 수 있다 (아직 TODO).
