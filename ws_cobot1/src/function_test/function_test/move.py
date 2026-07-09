# pick and place in 1 method. from pos1 to pos2 @20241104
import rclpy
import DR_init

# for single robot
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model= ROBOT_MODEL

ON, OFF = 1, 0


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_grip_simple", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_ref_coord,
            set_tool,
            set_tcp,
            movej,
            movel,
            DR_TOOL,
            DR_BASE,
            DR_MV_MOD_REL,
        )

        from DR_common2 import posj, posx

    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return

    set_tool("Tool Weight_1")
    set_tcp("GripperDA_v1")
    set_ref_coord(DR_BASE)


     
    homej = posj([0, 0, 90, 0, 90, 0])
    SCENT1_POSE = posx([336.37,-65.62,300,0,180,90])

    node.get_logger().info(f"Moving to joint position: {homej}")
    movej(homej, vel=VELOCITY, acc=ACC)

    try:
        movel(SCENT1_POSE, vel=VELOCITY, acc=ACC)


        node.get_logger().info("open_lid Test Complete")

    except KeyboardInterrupt:
        node.get_logger().info("Program Stopped")
    except TimeoutError as e:
        node.get_logger().error(str(e))

    finally:
        try:
            movej(homej, vel=VELOCITY, acc=ACC)
        except Exception as e:
            node.get_logger().error(f"Failed to move home: {e}")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


