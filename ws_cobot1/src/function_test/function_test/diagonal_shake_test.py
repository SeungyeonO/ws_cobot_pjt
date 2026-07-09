import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("test_shake_perfume2", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tcp,
            set_tool,
            movej,
            movel,
            move_periodic,
            wait,
            DR_TOOL,
            DR_MV_MOD_REL,
        )
        from DR_common2 import posj

    except ImportError as e:
        node.get_logger().error(f"Import error: {e}")
        return

    def move_home():
        homej = posj([0, 0, 90, 0, 90, 0])
        node.get_logger().info("Moving home")
        movej(homej, vel=100, acc=60)

    def shake_perfume2_test(
        tilt_angle=60,
        amp=30,
        period=0.6,
        repeat=2,
    ):
        node.get_logger().info("🚀 Start shake_perfume2 unit test")

        # 한쪽 60도 기울이기
        node.get_logger().info("Tilt +60 deg")
        movel(
            [0, 0, 0, 0, tilt_angle, 0],
            vel=80,
            acc=80,
            ref=DR_TOOL,
            mod=DR_MV_MOD_REL,
        )

        wait(0.5)

        # 기울어진 자세에서 사선 흔들기
        node.get_logger().info("Diagonal shake at +60 deg")
        move_periodic(
            amp=[0, 0, amp, 0, 0, 0],
            period=period,
            atime=0.2,
            repeat=repeat,
            ref=DR_TOOL,
        )

        wait(0.5)

        # 원위치 복귀
        node.get_logger().info("Return from +60 deg")
        movel(
            [0, 0, 0, 0, -tilt_angle, 0],
            vel=80,
            acc=80,
            ref=DR_TOOL,
            mod=DR_MV_MOD_REL,
        )

        wait(0.5)

        # 반대쪽 60도 기울이기
        node.get_logger().info("Tilt -60 deg")
        movel(
            [0, 0, 0, 0, -tilt_angle, 0],
            vel=80,
            acc=80,
            ref=DR_TOOL,
            mod=DR_MV_MOD_REL,
        )

        wait(0.5)

        # 반대 자세에서 사선 흔들기
        node.get_logger().info("Diagonal shake at -60 deg")
        move_periodic(
            amp=[0, 0, amp, 0, 0, 0],
            period=period,
            atime=0.2,
            repeat=repeat,
            ref=DR_TOOL,
        )

        wait(0.5)

        # 최종 원위치 복귀
        node.get_logger().info("Return from -60 deg")
        movel(
            [0, 0, 0, 0, tilt_angle, 0],
            vel=80,
            acc=80,
            ref=DR_TOOL,
            mod=DR_MV_MOD_REL,
        )

        node.get_logger().info("🙏 Finished shake_perfume2 unit test")

    try:
        set_tcp("GripperDA_v1")
        set_tool("Tool Weight1")

        move_home()
        wait(1.0)

        # 충돌 위험 있으면 먼저 z 방향으로 올림
        # movel(
        #     [0, 0, 100, 0, 0, 0],
        #     vel=80,
        #     acc=80,
        #     ref=DR_TOOL,
        #     mod=DR_MV_MOD_REL,
        # )

        shake_perfume2_test(
            tilt_angle=30,
            amp=20,
            period=0.5,
            repeat=2,
        )

        move_home()

    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")

    except Exception as e:
        node.get_logger().error(f"Test failed: {e}")
        try:
            move_home()
        except Exception as home_error:
            node.get_logger().error(f"Failed to move home: {home_error}")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()