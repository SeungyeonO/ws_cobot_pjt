# pick and place in 1 method. from pos1 to pos2 @20241104
import rclpy
import DR_init

# for single robot
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m0609"
VELOCITY, ACC = 300, 200

DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model= ROBOT_MODEL

ON, OFF = 1, 0
CYCLE = 3
CHECK_INTERVAL = 0.1


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("rokey_grip_simple", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_ref_coord,
            set_tool,
            set_tcp,
            set_digital_output,
            movej,
            movel,
            wait,
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
     
    homej = posj([0, 0, 90, 0, 90, 0])
    rotate_cw = posj([0, 0, 0, 0, 0, -180])
    rotate_ccw = posj([0, 0, 0, 0, 0, 180])

    node.get_logger().info(f"Moving to joint position: {homej}")
    movej(homej, vel=VELOCITY, acc=ACC)

    try:
        for i in range(CYCLE):
            node.get_logger().info(f"=== Cycle {i+1}/{CYCLE} ===")
            # movel(posx(0,0,-60,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=400, acc=300)
            grip()

            movej(rotate_cw, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)

            release()

            # movel(posx(0,0, 60,0,0,0), ref=DR_BASE, mod=DR_MV_MOD_REL, vel=400, acc=300)
            movej(rotate_ccw, vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL)


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


