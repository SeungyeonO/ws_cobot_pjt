"""perfume_kiosk — 키오스크 PC용 pip 패키지.

설치 (pip 패키지지만 /order_perfume 호출을 위해 rclpy·perfume_order_srv가
필요하므로, perfume_order_srv를 colcon build 한 뒤 ROS2 환경을 source하고
설치·실행한다):
  source /opt/ros/humble/setup.bash
  source <ws_cobot1>/install/setup.bash
  cd src/perfume_kiosk
  pip install -e .

실행:
  HMI_BASE_URL=http://<HMI-PC-IP>:5000 HMI_API_KEY=<키> perfume_kiosk
"""
from setuptools import find_packages, setup

setup(
    name="perfume_kiosk",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    install_requires=["flask", "requests"],
    description="조향 자동화 솔루션 키오스크 백엔드 (/order_perfume ROS2 서비스로 제조 요청)",
    maintainer="jeongwan-ryu",
    maintainer_email="jeowryu@gmail.com",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "perfume_kiosk = perfume_kiosk.app:main",
        ],
    },
)
