import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0
DR_BASE = 0
DR_MV_MOD_REL = 1
CYCLE = 4

VELOCITY = 30
ACC = 30


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("force_rotate_j6", namespace=ROBOT_ID)
    DR_init.__dsr__node = node


    try:
        from DSR_ROBOT2 import (
            set_ref_coord,
            set_tool,
            set_tcp,
            set_digital_output,
            task_compliance_ctrl,
            set_desired_force,
            release_force,
            release_compliance_ctrl,
            movel,
            wait,
            DR_TOOL,
            DR_MV_MOD_REL,
        )

        from DR_common2 import posj, posx

    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return

    set_tool("Tool Weight_1")
    set_tcp("GripperDA_v1")
    set_ref_coord(DR_TOOL)

    # =============================  함수  ===============================
    
    def grip():
        node.get_logger().info("set for digital output 1 0 for grip")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)

    def release():
        node.get_logger().info("set for digital output 0 1 for release")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(0.5)


    # ====================================================================

    try:
        for _ in range(CYCLE):
            grip()
            print("grip")

            movel(
                [0, 0, 0, 0, 0, 120],
                vel=100,
                acc=80,
                ref=DR_TOOL,
                mod=DR_MV_MOD_REL
            )

            release()
            print("ungrip")

            movel(
                [0, 0, 0, 0, 0, -120],
                vel=100,
                acc=80,
                ref=DR_TOOL,
                mod=DR_MV_MOD_REL
            )

    finally:
        # 4. 힘제어 해제
        try:
            pass
        except Exception as e:
            print(f"release error: {e}")

        rclpy.shutdown()


if __name__ == "__main__":
    main()