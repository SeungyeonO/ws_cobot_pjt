# **협동로봇 기반 맞춤형 향수 제조 시스템**

> **한 줄 소개:** 무인 체험형 서비스(인생네컷 방식)를 향수 제조에 접목하여, 고객이 직접 선택한 향료를 협동로봇이 정밀하게 조향하는 맞춤형 향수 제조 시스템입니다.
>
> **조 이름:** 일랑일랑

## 📖 프로젝트 소개

본 프로젝트는 **무인 체험형 서비스 모델(인생네컷 방식)**을 향수 제조 분야에 적용한 맞춤형 향수 제작 시스템입니다. 고객은 매장에 비치된 시향 샘플을 통해 원하는 향을 확인한 후, 키오스크에서 노트별 향료를 직접 선택하거나 추천 조합을 선택하여 자신만의 향수를 제작할 수 있습니다.

주문이 접수되면 **두산 M0609 협동로봇**이 실제 조향사가 사용하는 **고무 벌브 스포이드**를 직접 조작하여 향료를 채취하고 조제합니다. 또한 **OnRobot RG2 그리퍼의 위치 제어와 힘 제어**를 활용하여 향료를 안정적으로 계량함으로써 높은 조향 정밀도를 확보하였습니다.

### 👥 팀원

| 이름 | 역할 | 담당 업무 |
|------|------|-----------|
| **오승연** | 개발자 | 시스템 코드 전체 개발, 로봇 제어 및 키오스크 연동 구현 |
| **유정완** | 개발자 | 키오스크 및 관리자 HMI 개발, 사용자 화면 및 운영 기능 구현 |
| **노혜은** | 개발자 | 로봇 제어 코드 개발, 로봇 이동 함수 구현, 문서 및 발표 자료 작성 |
| **서정민** | PM | 프로젝트 관리, 개발 환경 구축, 하드웨어 제작, 문서 및 보고서 작성 |

---

# 1. 🎨 시스템 설계 및 플로우 차트

프로젝트의 전체 시스템 구성과 소프트웨어 동작 흐름을 나타낸 그림입니다.

## 1-1. 시스템 설계도 (System Architecture)

<!-- 시스템 설계도 이미지 삽입 -->

**설명**
- 키오스크, 협동로봇 제어 PC, 두산 로봇 컨트롤러, 협동로봇, 관리자 HMI로 구성된 전체 시스템 구조를 나타냅니다.
- 각 구성 요소는 ROS 2 기반의 서비스와 토픽을 통해 주문, 제어 및 상태 정보를 주고받으며 향수 제조 공정을 수행합니다.

---

## 1-2. 플로우 차트 (Flow Chart)

<!-- 플로우 차트 이미지 삽입 -->

**설명**
- 고객의 향료 선택부터 주문 접수, 협동로봇의 조향 작업, 제조 완료까지의 전체 공정 흐름을 나타냅니다.
- 주문 처리 과정에서 각 단계의 제어 흐름과 작업 순서를 한눈에 확인할 수 있습니다.


---

## 2. 🖥️ 운영체제 환경 (OS Environment)

키오스크 PC와 협동로봇 제어 PC는 동일한 개발 환경에서 구성 및 테스트되었습니다.

| 항목 | 환경 |
|------|------|
| **운영체제 (OS)** | Ubuntu 22.04 LTS |
| **ROS Version** | ROS 2 Humble Hawksbill |
| **Python** | Python 3.10.12 |
| **IDE** | Visual Studio Code (VS Code) |


---

## 3. 🛠️ 사용 장비 목록 (Hardware List)

프로젝트에 사용된 주요 하드웨어 장비입니다.

| 장비명 | 모델 | 수량 | 비고 |
|--------|------|:---:|------|
| 협동로봇 | Doosan Robotics M0609 | 1 | 향료 채취 및 조향 작업 수행 |
| 전동 그리퍼 | OnRobot RG2 | 1 | 고무 벌브 스포이드 파지 및 위치·힘 제어 |
| 3D 프린터 | bambu lab p1s | 1 | 환경 구성 장비 제작 |


---

## 4. 📦 의존성 (Dependencies)

프로젝트 실행을 위해 추가로 사용된 Python 라이브러리입니다.

| 라이브러리 | 버전 | 용도 |
|-----------|:---:|------|
| Flask | >= 2.0 | 키오스크 GUI와 협동로봇 제어 프로그램 간의 HTTP 기반 통신 |
| Requests | >= 2.25 | Flask 서버와의 HTTP 요청 및 응답 처리 |
| SQLite3 | Python 기본 포함 | 향료 정보 및 추천 조합 데이터 저장 및 관리 |

### 설치

```bash
pip install "flask>=2.0" "requests>=2.25"
```

> `sqlite3`는 Python 표준 라이브러리에 포함되어 있으므로 별도의 설치가 필요하지 않습니다.


---

## 5. ▶️ 실행 순서 (Usage Guide)

실행 전 각 터미널에서 ROS 2 및 워크스페이스 환경을 설정합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
source ~/ws_cobot_pjt/ws_cobot1/install/setup.bash
```

### Step 1. 로봇 Bringup 실행

두산 M0609 협동로봇과 컨트롤러의 전원을 켠 후 Bringup을 실행합니다.

> **host**에는 로봇 컨트롤러의 IP 주소를 입력합니다.

### Step 2. 향수 제조 모션 제어 노드 실행

향수 제조 공정을 수행하는 메인 제어 노드를 실행합니다.

### Step 3. 비상정지 관리 노드 실행

비상정지 및 복구 기능을 담당하는 노드를 실행합니다.

### Step 4. 관리자 HMI 실행

관리자 HMI를 실행한 후 웹 브라우저에서 아래 주소로 접속합니다.

`http://localhost:5000`

### Step 5. 고객용 키오스크 실행

실행 전 `app.py`에서 아래 코드의 `HMI_BASE_URL`을 **제어 PC(HMI 서버)의 IP 주소**로 변경합니다.

```python
HMI_BASE_URL = os.environ.get(
    "HMI_BASE_URL",
    "http://172.23.0.195:5000"
)
```

설정이 완료되면 키오스크를 실행한 후 웹 브라우저에서 아래 주소로 접속합니다.

`http://localhost:5000`

### 비상정지 후 재시작

관리자 HMI에서 비상정지를 수행하면 `main` 노드가 종료됩니다. 복구가 완료된 후에는 아래 명령어를 다시 실행하여 메인 제어 노드를 재시작해야 합니다.

### 실행 명령어

```bash
# ROS2 및 워크스페이스 환경 설정
source /opt/ros/humble/setup.bash
source ~/ws_cobot_pjt/ws_dsr/install/setup.bash
source ~/ws_cobot_pjt/ws_cobot1/install/setup.bash

# 1. Bringup
ros2 launch m0609_rg2_bringup bringup.launch.py \
mode:=real \
host:=192.168.1.100 \
model:=m0609

# 2. 메인 제어 노드
ros2 run cobot_control main

# 3. 비상정지 관리 노드
ros2 run cobot_control stop_manager

# 4. 관리자 HMI
ros2 run perfume_hmi perfume_hmi

# 5. 고객용 키오스크
ros2 run perfume_kiosk perfume_kiosk

# 비상정지 후 메인 노드 재실행
ros2 run cobot_control main
```
