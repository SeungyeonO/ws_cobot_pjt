# ros2_node.py
"""
전제:

ROS2 Humble
Doosan Robotics ROS2 패키지 (dsr_msgs2, dsr_bringup2)
Robot namespace: /dsr01
로봇: M0609
GUI 연동 (cobot1_dash_board.py에서 호출)
사용 인터페이스:
/joint_states → sensor_msgs/msg/JointState
/dsr01/state → dsr_msgs2/msg/RobotState
/dsr01/motion/move_joint → dsr_msgs2/srv/MoveJoint
/dsr01/stop → std_srvs/srv/Trigger
/robot/start → std_msgs/msg/Bool
/robot/stop → std_msgs/msg/Bool
/robot/estop → std_msgs/msg/Bool

참고: Doosan ROS2 버전에 따라 서비스 이름은 motion/move_joint, motion/move_line 등의
namespace가 달라질 수 있습니다. 아래 코드는 /dsr01 namespace 기준입니다.
"""

import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState

from trajectory_msgs.msg import JointTrajectory

from std_srvs.srv import Trigger

# Doosan message
from dsr_msgs2.msg import RobotState
from dsr_msgs2.srv import MoveJoint


class RobotNode(Node):

    def __init__(self):

        super().__init__("m0609_gui_node")
        self.gui = None
        qos = QoSProfile(depth=10)
        ####################################
        # Publisher
        ####################################

        self.start_pub = self.create_publisher(Bool, "/robot/start", qos)
        self.stop_pub = self.create_publisher(Bool, "/robot/stop", qos)
        self.estop_pub = self.create_publisher(Bool, "/robot/estop", qos)
        self.mode_pub = self.create_publisher(String, "/robot/mode", qos)

        ####################################
        # Joint trajectory
        ####################################

        self.joint_pub = self.create_publisher(
            JointTrajectory, "/dsr01/joint_trajectory", qos
        )

        ####################################
        # Subscriber
        ####################################

        self.joint_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_callback, qos
        )

        self.state_sub = self.create_subscription(
            RobotState, "/dsr01/state", self.state_callback, qos
        )

        ####################################
        # Doosan Service
        ####################################

        self.move_joint_client = self.create_client(
            MoveJoint, "/dsr01/motion/move_joint"
        )

        self.stop_client = self.create_client(Trigger, "/dsr01/stop")

        self.current_joint = [0.0] * 6

        self.get_logger().info("Doosan M0609 ROS2 Node Started")

    ########################################
    # GUI 연결
    ########################################

    def set_gui(self, gui):

        self.gui = gui

    ########################################
    # Robot Control
    ########################################

    def publish_start(self):

        msg = Bool()
        msg.data = True

        self.start_pub.publish(msg)

        self.log("Robot Start")

    def publish_stop(self):

        msg = Bool()
        msg.data = True

        self.stop_pub.publish(msg)
        self.log("Robot Stop")

    def publish_estop(self):

        msg = Bool()
        msg.data = True

        self.estop_pub.publish(msg)
        self.log("Emergency Stop")

    def publish_shutdown(self):

        if self.stop_client.wait_for_service(timeout_sec=1.0):

            req = Trigger.Request()

            self.stop_client.call_async(req)

            self.log("Shutdown Service Called")

    ########################################
    # Mode
    ########################################

    def publish_mode(self, mode):

        msg = String()
        msg.data = mode

        self.mode_pub.publish(msg)

    ########################################
    # Joint Slider
    ########################################

    def publish_joint(self, index, value):

        self.current_joint[index] = value * 3.141592 / 180.0

        traj = JointTrajectory()
        traj.joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]

        point = JointTrajectoryPoint()

        point.positions = self.current_joint
        point.time_from_start.sec = 1

        traj.points.append(point)

        self.joint_pub.publish(traj)

    ########################################
    # Doosan MoveJoint
    ########################################

    def move_joint(self, position, velocity=20.0, acceleration=20.0):

        if not self.move_joint_client.wait_for_service(timeout_sec=1.0):

            self.get_logger().warn("MoveJoint service unavailable")

            return

        req = MoveJoint.Request()
        req.pos = position
        req.vel = velocity
        req.acc = acceleration

        self.move_joint_client.call_async(req)

    ########################################
    # Callback
    ########################################

    def joint_callback(self, msg):
        self.current_joint = list(msg.position[:6])

    def state_callback(self, msg):
        if self.gui is None:
            return

        self.gui.update_robot_status(msg.robot_state, msg.robot_state, 0, 0, 0, 0)

    ########################################
    # Log
    ########################################

    def log(self, text):
        self.get_logger().info(text)

        if self.gui:

            self.gui.write_log(text)

    ########################################
    # 종료
    ########################################

    def shutdown(self):

        self.get_logger().info("ROS shutdown")
