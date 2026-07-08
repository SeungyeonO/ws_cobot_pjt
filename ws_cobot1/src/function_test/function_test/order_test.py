import rclpy
import DR_init
from perfume_order_srv.srv import Order

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node("order_test", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            movel,
            wait,
            DR_BASE,
            DR_MV_MOD_REL,
        )

    except ImportError as e:
        node.get_logger().error(f"Import error: {e}")
        return

    set_tool("Tool Weight_1")
    set_tcp("GripperDA_v1")

    order_requested = False

    def order_callback(request, response):
        nonlocal order_requested

        node.get_logger().info("Order request received")
        order_requested = True

        response.success = True
        return response

    node.create_service(
        Order,
        "order_perfume",
        order_callback,
    )

    node.get_logger().info("Order test service ready")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            if order_requested:
                order_requested = False

                node.get_logger().info("Move up 100mm start")

                movel(
                    [0, 0, 100, 0, 0, 0],
                    vel=100,
                    acc=80,
                    ref=DR_BASE,
                    mod=DR_MV_MOD_REL,
                )

                node.get_logger().info("Move up 100mm done")

                wait(0.5)

                node.get_logger().info("Move down 100mm start")

                movel(
                    [0, 0, -100, 0, 0, 0],
                    vel=100,
                    acc=80,
                    ref=DR_BASE,
                    mod=DR_MV_MOD_REL,
                )

                node.get_logger().info("Move down 100mm done")

    except KeyboardInterrupt:
        node.get_logger().info("Program stopped")

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()