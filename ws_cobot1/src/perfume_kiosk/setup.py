import os
from glob import glob

from setuptools import find_packages, setup

package_name = "perfume_kiosk"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "frontend"), glob("frontend/*")),
    ],
    # schema.sql은 app.py와 같은 폴더에 있어야 하므로 (SCHEMA_PATH가 __file__ 기준)
    # share가 아니라 파이썬 패키지 안에 함께 설치한다.
    package_data={package_name: ["schema.sql"]},
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jeongwan-ryu",
    maintainer_email="jeowryu@gmail.com",
    description="조향 자동화 솔루션 키오스크 백엔드 (/order_perfume ROS2 서비스로 제조 요청)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perfume_kiosk = perfume_kiosk.app:main",
        ],
    },
)
