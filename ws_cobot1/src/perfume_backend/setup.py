import os
from glob import glob

from setuptools import find_packages, setup

package_name = "perfume_backend"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, [os.path.join(package_name, "schema.sql")]),
        (os.path.join("share", package_name, "frontend"), glob("frontend/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jeongwan-ryu",
    maintainer_email="jeowryu@gmail.com",
    description="조향 자동화 솔루션 Flask 백엔드 (perfume_order_srv로 로봇팔 제어부와 ROS2 통신)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perfume_backend = perfume_backend.app:main",
        ],
    },
)
