import rclpy
import DR_init
import threading

from perfume_order_srv.srv import Order
from std_msgs.msg import Bool


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 300, 200

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("main", namespace=ROBOT_ID)

    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            set_tool,
            set_tcp,
            task_compliance_ctrl,
            set_desired_force,
            release_force,
            release_compliance_ctrl,
            get_tool_force,
            movej,
            wait,
            DR_MV_MOD_REL,
        )

        from DR_common2 import posj

        from cobot_control.control_functions import (
            RobotController,
            PERFUME_POSE,
            PICKUP_POSE,
            PERFUME_LID_POSE,
            scent_positions,
        )

    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return

    set_tcp("GripperDA_v1")
    set_tool("Tool Weight1")

    controller = RobotController(node)
    controller.move_to_home()

    done_pub = node.create_publisher(Bool, "/perfume_done", 10)

    order = None
    is_busy = False

    order_event = threading.Event()
    busy_lock = threading.Lock()

    def publish_done(result: bool):
        done_msg = Bool()
        done_msg.data = result
        done_pub.publish(done_msg)

    def order_callback(request, response):
        nonlocal order, is_busy

        new_order = [
            request.scent1,
            request.scent2,
            request.scent3,
            request.scent4,
            request.scent5,
            request.scent6,
        ]

        node.get_logger().warn(f"🔥 CALLBACK CALLED / current is_busy = {is_busy}")

        with busy_lock:
            if is_busy:
                node.get_logger().warn("⚠️ Robot is already processing an order")
                response.success = False
                return response

            is_busy = True
            order = new_order

        node.get_logger().info(f"📦 Order received: {order}")

        order_event.set()

        response.success = True
        return response

    node.create_service(
        Order,
        "/order_perfume",
        order_callback,
    )

    def process_order(order):
        node.get_logger().info(f"🔥 Start processing order: {order}")
        wait(1.0)

        controller.move_to_pose(
            PERFUME_POSE,
            from_home=True,
            down=73,
            velocity=30,
            acceleration=30,
        )

        if not controller.check_perfume_bottle():
            node.get_logger().info("❌ 향수병이 없습니다.")

            controller.move_to_home()
            publish_done(False)
            return

        controller.open_lid(cycle=3)

        node.get_logger().info("🔥 향수병 뚜껑 거치")
        wait(1.0)

        controller.grip()
        controller.move_to_pose(goal_pose=PERFUME_LID_POSE, up=73, down=50)
        controller.release()

        for i in range(6):
            scent_pose = scent_positions[i]
            drops = order[i]
            first = True

            for drop in range(drops):
                node.get_logger().info(f"💧 scent{i+1}: {drop+1}/{drops} drops")

                if first:
                    node.get_logger().info(f"📍➔📍 scent{i+1} 향료병으로 이동")
                    wait(1.0)
                    controller.move_to_pose(
                        goal_pose=scent_pose,
                        up=50,
                        down=54,
                    )

                node.get_logger().info("🧪 향료 추출 시작")
                wait(1.0)

                controller.grip()
                controller.release()

                node.get_logger().info("🧪 향료 추출 완료")

                node.get_logger().info("🦾 뚜껑 잡기 시작")
                wait(1.0)

                controller.move_down(z=18.7)
                controller.grip()

                movej(
                    posj([0, 0, 0, 0, 0, -45]),
                    vel=50,
                    acc=50,
                    mod=DR_MV_MOD_REL,
                )

                controller.move_up(z=18.7)

                movej(
                    posj([0, 0, 0, 0, 0, 45]),
                    vel=50,
                    acc=50,
                    mod=DR_MV_MOD_REL,
                )

                node.get_logger().info("🦾 뚜껑 잡기 완료")

                node.get_logger().info("📍➔📍 향수병으로 이동")
                controller.move_to_pose(
                    goal_pose=PERFUME_POSE,
                    up=54,
                    down=54,
                )
                wait(1.0)

                controller.release()

                node.get_logger().info("🧪 향료 투출 시작")
                wait(1.0)

                controller.grip()
                controller.release()

                node.get_logger().info("🧪 향료 투출 완료")

                node.get_logger().info("🦾 뚜껑 잡기 시작")
                wait(1.0)

                controller.move_down(z=18.7)
                controller.grip()

                movej(
                    posj([0, 0, 0, 0, 0, -45]),
                    vel=50,
                    acc=50,
                    mod=DR_MV_MOD_REL,
                )

                controller.move_up(z=18.7)

                movej(
                    posj([0, 0, 0, 0, 0, 45]),
                    vel=50,
                    acc=50,
                    mod=DR_MV_MOD_REL,
                )

                node.get_logger().info("🦾 뚜껑 잡기 완료")

                node.get_logger().info(f"📍➔📍 scent{i+1} 향료병으로 이동")
                wait(1.0)

                controller.move_to_pose(
                    goal_pose=scent_pose,
                    up=54,
                    down=54,
                )

                node.get_logger().info("✋ 뚜껑 놓기")
                wait(1.0)

                controller.release()

                first = False

        node.get_logger().info("📍➔📍 향수병 뚜껑 거치대 위치로 이동")
        wait(1.0)

        controller.move_to_pose(PERFUME_LID_POSE, up=54, down=55)
        controller.grip()

        node.get_logger().info("📍➔📍 향수병 위치로 이동")
        wait(1.0)

        controller.move_to_pose(PERFUME_POSE, up=65, down=60)
        controller.close_lid(cycle=3)

        node.get_logger().info("🦾 향수병 뚜껑 잡기 시작")
        wait(1.0)

        controller.grip()

        node.get_logger().info("🦾 향수병 뚜껑 잡기 완료")

        controller.move_to_home()
        wait(1.0)

        controller.shake_perfume(cycle=5)
        controller.shake_perfume2(tilt_angle=30, amp=20)

        node.get_logger().info("📍➔📍 픽업장소으로 이동")
        wait(1.0)

        controller.move_to_pose(
            PICKUP_POSE,
            from_home=True,
            up=30,
            down=40,
        )

        task_compliance_ctrl([300, 300, 300, 200, 200, 200], 0)
        wait(0.5)

        set_desired_force(
            [0, 0, -30, 0, 0, 0],
            [0, 0, 1, 0, 0, 0],
        )

        while rclpy.ok():
            var_force = get_tool_force()

            if var_force[2] > 10:
                release_force()
                release_compliance_ctrl()
                break

        controller.release()

        controller.move_to_home()

        node.get_logger().info("🫙 perfume is ready")

        publish_done(True)

        node.get_logger().info("✅ Published perfume_done: True")

    def spin_worker():
        node.get_logger().info("🧵 ROS spin thread started")
        rclpy.spin(node)

    spin_thread = threading.Thread(
        target=spin_worker,
        daemon=True,
    )
    spin_thread.start()

    node.get_logger().info("Order perfume service server ready")

    try:
        while rclpy.ok():
            order_event.wait(timeout=0.1)

            if not order_event.is_set():
                continue

            with busy_lock:
                current_order = order
                order = None

            try:
                process_order(current_order)

            except Exception as e:
                node.get_logger().error(f"❌ Motion error: {e}")

                try:
                    release_force()
                    release_compliance_ctrl()
                except Exception:
                    pass

                try:
                    controller.move_to_home()
                except Exception as home_error:
                    node.get_logger().error(f"Failed to move home: {home_error}")

                publish_done(False)

            finally:
                order_event.clear()

                with busy_lock:
                    is_busy = False

                node.get_logger().info("✅ Motion finished, robot is ready")

    except KeyboardInterrupt:
        node.get_logger().info("Program Stopped")

    finally:
        try:
            release_force()
            release_compliance_ctrl()
        except Exception:
            pass

        try:
            controller.move_to_home()
        except Exception as e:
            node.get_logger().error(f"Failed to move home: {e}")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()