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
            move_periodic,
            movej,
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
    set_ref_coord(DR_TOOL)  # 툴좌표계로 설정(태스크 모션 시 적용)

    # =============================  함수  ===============================
    
    def grip():
        node.get_logger().info("set for digital output 1 0 for grip")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.3)

    def release():
        node.get_logger().info("set for digital output 0 1 for release")
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(0.3)


    # ====================================================================
     
    try:
        grip()

        # 원운동하며 혼합
        move_periodic(
            amp=[20, 20, 0, 0, 0, 0],
            period=1.0,
            atime=0.2,
            repeat=8,
            ref=DR_TOOL
        )

        wait(0.5)
        
        release()

        node.get_logger().info("mix_drawing_circle Test Complete")

    except KeyboardInterrupt:
        node.get_logger().info("Program Stopped")
    except TimeoutError as e:
        node.get_logger().error(str(e))

    finally:
        try:
            pass
        except Exception as e:
            node.get_logger().error(f"Failed to move home: {e}")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


