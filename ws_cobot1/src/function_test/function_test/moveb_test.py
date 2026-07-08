import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

VELOCITY, ACC = 50, 100
# 매뉴얼 속도는 아래 값
# VELOCITY, ACC = 150, 250

DR_BASE = 0
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1

# moveb segment type
DR_LINE = 0
DR_CIRCLE = 1

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("rokey_moveb", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movej,
            movel,
            moveb,
            wait,
        )

        from DR_common2 import (
            posx,
            posj,
            posb,
        )

    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return

    print("Starting Manual MoveB Program")

    set_tool("Tool Weight1")
    set_tcp("GripperDA_v1")

    # =========================
    # Init Pose
    # =========================
    print("set_tool")
    Jx1 = posj([45, 0, 90, 0, 90, 45])
    X0 = posx(370, 420, 650, 0, 180, 0)
    print("init_pose")
    # =========================
    # Absolute Goal Poses
    # =========================
    X1 = posx(370, 670, 650, 0, 180, 0)

    X1a = posx(370, 670, 400, 0, 180, 0)
    X1a2 = posx(370, 545, 400, 0, 180, 0)

    X1b2 = posx(370, 670, 400, 0, 180, 0)

    X1c = posx(370, 420, 150, 0, 180, 0)
    X1c2 = posx(370, 545, 150, 0, 180, 0)

    X1d = posx(370, 670, 275, 0, 180, 0)
    X1d2 = posx(370, 795, 150, 0, 180, 0)
    print("absolute goal pose")
    # =========================
    # moveb Segment List
    # =========================
    seg11 = posb(DR_LINE, X1, radius=20)
    seg12 = posb(DR_CIRCLE, X1a, X1a2, radius=20)
    seg14 = posb(DR_LINE, X1b2, radius=20)
    seg15 = posb(DR_CIRCLE, X1c, X1c2, radius=20)
    seg16 = posb(DR_CIRCLE, X1d, X1d2, radius=20)

    b_list1 = [seg11, seg12, seg14, seg15, seg16]

    try:
        node.get_logger().info("Moving to initial joint position Jx1")
        movej(Jx1, vel=30, acc=60, mod=DR_MV_MOD_ABS)

        wait(1)

        node.get_logger().info("Moving to initial task position X0")
        movel(
            X0,
            vel=VELOCITY,
            acc=ACC,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )

        wait(1)

        node.get_logger().info("Starting moveb trajectory")
        moveb(
            b_list1,
            vel=VELOCITY,
            acc=ACC,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )

        node.get_logger().info("MoveB trajectory finished")

    except KeyboardInterrupt:
        node.get_logger().info("Program Stopped")

    except Exception as e:
        node.get_logger().info(f"Robot Error: {e}")

    finally:
        node.get_logger().info("Shutdown node")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()