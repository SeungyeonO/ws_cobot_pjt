import os
from glob import glob

from setuptools import find_packages, setup

package_name = "perfume_hmi"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "frontend"), glob("frontend/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jeongwan-ryu",
    maintainer_email="jeowryu@gmail.com",
    description="조향 자동화 솔루션 HMI 백엔드 (관리자 모니터링/대시보드 — 로봇 연결 상태·정지·홈 복귀)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perfume_hmi = perfume_hmi.app:main",
        ],
    },
)
