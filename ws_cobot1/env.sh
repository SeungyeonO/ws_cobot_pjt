#!/usr/bin/env bash
# ROS2 + ws_dsr + ws_cobot1 워크스페이스 환경을 한 번에 불러오기 위한 스크립트.
#
# 로봇 제어 PC(HMI 백엔드 + 로봇 제어 노드가 도는 PC) 전용이다. 키오스크 PC는
# perfume_kiosk(pip 패키지)가 /order_perfume 호출을 위해 ROS2(rclpy)와
# perfume_order_srv는 필요하지만, dsr_msgs2 등 두산 드라이버(ws_dsr)는 필요
# 없다 — 이 스크립트를 그대로 써도 되지만(불필요한 ws_dsr까지 같이 source될
# 뿐), 키오스크 PC에 ws_dsr이 없다면 아래 두 줄만 source하면 된다:
#   source /opt/ros/humble/setup.bash
#   source <ws_cobot1>/install/setup.bash
#
# 사용법 (워크스페이스 루트에서, 새 터미널을 열 때마다 한 번):
#   cd ws_cobot1
#   source env.sh
#
# 반드시 "source"로 실행해야 한다 (./env.sh 로 실행하면 하위 프로세스에서만
# 환경변수가 바뀌고 현재 셸에는 적용되지 않는다).
#
# 이걸 source하면:
# - ROS2 humble 기본 환경 (ros2 명령어, rclpy 등)
# - ws_dsr 워크스페이스 빌드 결과물 (dsr_msgs2, rokey 등)
# - ws_cobot1 워크스페이스 빌드 결과물 (perfume_order_srv, perfume_hmi)
# 셋 다 현재 셸에서 바로 쓸 수 있게 된다. (ros2 run perfume_hmi perfume_hmi 실행 전 필요)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DSR_DIR="$(cd "${SCRIPT_DIR}/../ws_dsr" && pwd)"
WS_COBOT1_DIR="${SCRIPT_DIR}"

source /opt/ros/humble/setup.bash
source "${WS_DSR_DIR}/install/setup.bash"
source "${WS_COBOT1_DIR}/install/setup.bash"

echo "[env.sh] ROS2 humble + ws_dsr + ws_cobot1 환경 로드 완료 (dsr_msgs2, perfume_order_srv, perfume_hmi 등 사용 가능)"
echo "[env.sh] 실행: ros2 run perfume_hmi perfume_hmi"
