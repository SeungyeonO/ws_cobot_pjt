import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("di_monitor", namespace=ROBOT_ID)

    # 중요: DSR import 전에 node 등록
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import get_digital_input, wait

    except ImportError as e:
        node.get_logger().error(f"DSR_ROBOT2 import failed: {e}")
        return

    node.get_logger().info("Digital Input Monitor Started")

    try:
        prev = None

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            di1 = get_digital_input(1)
            di2 = get_digital_input(2)
            di3 = get_digital_input(3)

            current = (di1, di2, di3)

            if current != prev:
                node.get_logger().info(
                    f"DI1={di1}, DI2={di2}, DI3={di3}"
                )
                prev = current

            wait(0.1)

    except KeyboardInterrupt:
        node.get_logger().info("Stopped by user")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()