# cobot1_dash_board.py

from PyQt5 import QtWidgets, uic
from PyQt5.QtCore import Qt

from trajectory_msgs.msg import JointTrajectoryPoint


class RobotGUI(QtWidgets.QMainWindow):

    def __init__(self, ros2_node):
        super().__init__()

        # Qt Designer ui 불러오기
        uic.loadUi("cobot1_dash_board.ui", self)

        self.node = ros2_node

        # -------------------------------
        # Button Signal
        # -------------------------------
        self.btn_start.clicked.connect(self.start_robot)
        self.btn_stop.clicked.connect(self.stop_robot)
        self.btn_shutdown.clicked.connect(self.shutdown_robot)
        self.btn_estop.clicked.connect(self.emergency_stop)

        # -------------------------------
        # Combo Box
        # -------------------------------
        self.combo_mode.currentTextChanged.connect(self.change_mode)

        # -------------------------------
        # Joint Slider
        # -------------------------------
        self.slider_j1.valueChanged.connect(lambda value: self.joint_changed(0, value))
        self.slider_j2.valueChanged.connect(lambda value: self.joint_changed(1, value))
        self.slider_j3.valueChanged.connect(lambda value: self.joint_changed(2, value))
        self.slider_j4.valueChanged.connect(lambda value: self.joint_changed(3, value))
        self.slider_j5.valueChanged.connect(lambda value: self.joint_changed(4, value))
        self.slider_j6.valueChanged.connect(lambda value: self.joint_changed(5, value))

        # Slider 설정
        self.initialize_slider()

    ####################################################
    # Slider 초기 설정
    ####################################################
    def initialize_slider(self):

        sliders = [
            self.slider_j1,
            self.slider_j2,
            self.slider_j3,
            self.slider_j4,
            self.slider_j5,
            self.slider_j6,
        ]

        for slider in sliders:
            slider.setOrientation(Qt.Horizontal)
            slider.setMinimum(-180)
            slider.setMaximum(180)
            slider.setValue(0)

    ####################################################
    # Button
    ####################################################
    def start_robot(self):
        self.node.publish_start()

        self.write_log("Start Button Clicked")

    def stop_robot(self):
        self.node.publish_stop()

        self.write_log("Stop Button Clicked")

    def shutdown_robot(self):
        self.node.publish_shutdown()

        self.write_log("Shutdown Button Clicked")

    def emergency_stop(self):
        self.node.publish_estop()

        self.write_log("!!! EMERGENCY STOP !!!")

    ####################################################
    # Mode
    ####################################################
    def change_mode(self, mode):
        self.node.publish_mode(mode)

        self.write_log(f"Mode : {mode}")

    ####################################################
    # Joint
    ####################################################
    def joint_changed(self, index, value):
        # self.node.publish_joint(index, value)
        self.node.move_joint(joint_position)

    ####################################################
    # Robot Info
    ####################################################
    def update_robot_status(self, status, mode, battery, voltage, cpu, temperature):

        self.lbl_status.setText(str(status))
        self.lbl_mode.setText(str(mode))
        self.lbl_battery.setText(f"{battery:.1f} %")
        self.lbl_voltage.setText(f"{voltage:.2f} V")
        self.lbl_cpu.setText(f"{cpu:.1f} %")
        self.lbl_temp.setText(f"{temperature:.1f} ℃")

    ####################################################
    # IMU
    ####################################################
    def update_imu(self, roll, pitch, yaw, ax, ay, az, gx, gy, gz):
        
        self.lbl_roll.setText(f"{roll:.2f}")
        self.lbl_pitch.setText(f"{pitch:.2f}")
        self.lbl_yaw.setText(f"{yaw:.2f}")

        self.lbl_ax.setText(f"{ax:.2f}")
        self.lbl_ay.setText(f"{ay:.2f}")
        self.lbl_az.setText(f"{az:.2f}")

        self.lbl_gx.setText(f"{gx:.2f}")
        self.lbl_gy.setText(f"{gy:.2f}")
        self.lbl_gz.setText(f"{gz:.2f}")

    ####################################################
    # System Log
    ####################################################
    def write_log(self, text):
        self.text_log.append(text)

    ####################################################
    # ROS Callback
    ####################################################
    def robot_callback(self, msg):
        """
        ros_node.py에서 호출
        """
        self.update_robot_status(
            msg.status, msg.mode, msg.battery, msg.voltage, msg.cpu, msg.temperature
        )

    def imu_callback(self, roll, pitch, yaw, ax, ay, az, gx, gy, gz):
        self.update_imu(roll, pitch, yaw, ax, ay, az, gx, gy, gz)

    ####################################################
    # Close
    ####################################################
    def closeEvent(self, event):
        self.node.shutdown()

        event.accept()
