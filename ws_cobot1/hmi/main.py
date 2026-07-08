# main.py

import sys

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

import rclpy

from cobot1_dash_board import RobotGUI
from ros2_node import RobotNode


def main(args=None):

    # ROS2 초기화
    rclpy.init(args=args)

    # ROS2 노드 생성
    ros_node = RobotNode()

    # Qt Application 생성
    app = QApplication(sys.argv)

    # GUI 생성
    window = RobotGUI(ros_node)

    # GUI를 ROS 노드에 등록
    ros_node.set_gui(window)

    window.show()

    # -----------------------------
    # ROS2 Spin
    # -----------------------------
    timer = QTimer()

    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.0))

    # 20ms = 50Hz
    timer.start(20)

    # -----------------------------
    # Qt 실행
    # -----------------------------
    exit_code = app.exec()

    # -----------------------------
    # 종료 처리
    # -----------------------------
    ros_node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
