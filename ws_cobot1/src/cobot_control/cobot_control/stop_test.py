import rclpy
import DR_init

from perfume_order_srv.srv import Order
from std_msgs.msg import Bool, Int32


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 300, 200

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# ============================================================
# HMI 공정 상태 코드
# ============================================================

STATUS_IDLE = 0
STATUS_ORDER_RECEIVED = 10
STATUS_PROCESS_START = 20
STATUS_MOVE_TO_PERFUME = 30
STATUS_CHECK_PERFUME = 40
STATUS_OPEN_PERFUME_LID = 50
STATUS_STORE_PERFUME_LID = 60

STATUS_SCENT_PROCESS_START = 100
STATUS_MOVE_TO_SCENT = 110
STATUS_EXTRACT_SCENT = 120
STATUS_GRIP_SCENT_LID = 130
STATUS_OPEN_SCENT_LID = 140
STATUS_MOVE_TO_MIX_BOTTLE = 150
STATUS_DISPENSE_SCENT = 160
STATUS_GRIP_SCENT_LID_RETURN = 170
STATUS_RETURN_TO_SCENT = 180
STATUS_CLOSE_SCENT_LID = 190
STATUS_SCENT_PROCESS_DONE = 200

STATUS_GET_PERFUME_LID = 210
STATUS_MOVE_LID_TO_PERFUME = 220
STATUS_CLOSE_PERFUME_LID = 230
STATUS_GRIP_FINISHED_PERFUME = 240
STATUS_MOVE_TO_HOME = 250
STATUS_SHAKE_PERFUME = 260
STATUS_TILT_MIX_PERFUME = 270
STATUS_MOVE_TO_PICKUP = 280
STATUS_PLACE_PERFUME = 290
STATUS_RELEASE_PERFUME = 300
STATUS_PROCESS_COMPLETE = 310
STATUS_RETURN_HOME = 320
STATUS_READY = 330


# 콘솔에서 상태 번호와 이름을 함께 확인하기 위한 용도
STATUS_NAMES = {
    STATUS_IDLE: "IDLE",
    STATUS_ORDER_RECEIVED: "ORDER_RECEIVED",
    STATUS_PROCESS_START: "PROCESS_START",
    STATUS_MOVE_TO_PERFUME: "MOVE_TO_PERFUME",
    STATUS_CHECK_PERFUME: "CHECK_PERFUME",
    STATUS_OPEN_PERFUME_LID: "OPEN_PERFUME_LID",
    STATUS_STORE_PERFUME_LID: "STORE_PERFUME_LID",

    STATUS_SCENT_PROCESS_START: "SCENT_PROCESS_START",
    STATUS_MOVE_TO_SCENT: "MOVE_TO_SCENT",
    STATUS_EXTRACT_SCENT: "EXTRACT_SCENT",
    STATUS_GRIP_SCENT_LID: "GRIP_SCENT_LID",
    STATUS_OPEN_SCENT_LID: "OPEN_SCENT_LID",
    STATUS_MOVE_TO_MIX_BOTTLE: "MOVE_TO_MIX_BOTTLE",
    STATUS_DISPENSE_SCENT: "DISPENSE_SCENT",
    STATUS_GRIP_SCENT_LID_RETURN: "GRIP_SCENT_LID_RETURN",
    STATUS_RETURN_TO_SCENT: "RETURN_TO_SCENT",
    STATUS_CLOSE_SCENT_LID: "CLOSE_SCENT_LID",
    STATUS_SCENT_PROCESS_DONE: "SCENT_PROCESS_DONE",

    STATUS_GET_PERFUME_LID: "GET_PERFUME_LID",
    STATUS_MOVE_LID_TO_PERFUME: "MOVE_LID_TO_PERFUME",
    STATUS_CLOSE_PERFUME_LID: "CLOSE_PERFUME_LID",
    STATUS_GRIP_FINISHED_PERFUME: "GRIP_FINISHED_PERFUME",
    STATUS_MOVE_TO_HOME: "MOVE_TO_HOME",
    STATUS_SHAKE_PERFUME: "SHAKE_PERFUME",
    STATUS_TILT_MIX_PERFUME: "TILT_MIX_PERFUME",
    STATUS_MOVE_TO_PICKUP: "MOVE_TO_PICKUP",
    STATUS_PLACE_PERFUME: "PLACE_PERFUME",
    STATUS_RELEASE_PERFUME: "RELEASE_PERFUME",
    STATUS_PROCESS_COMPLETE: "PROCESS_COMPLETE",
    STATUS_RETURN_HOME: "RETURN_HOME",
    STATUS_READY: "READY",
}


def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node(
        "main",
        namespace=ROBOT_ID,
    )

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
        node.get_logger().error(f"Error importing DSR_ROBOT2: {e}")
        node.destroy_node()
        rclpy.shutdown()
        return

    set_tcp("GripperDA_v1")
    set_tool("Tool Weight1")

    controller = RobotController(node)
    controller.move_to_home()

    # ========================================================
    # Publisher
    # ========================================================

    done_pub = node.create_publisher(
        Bool,
        "/perfume_done",
        10,
    )

    status_pub = node.create_publisher(
        Int32,
        "/perfume_status",
        10,
    )

    def publish_status(status_code):
        """
        HMI에 현재 공정 상태 번호를 전송한다.
        """

        msg = Int32()
        msg.data = int(status_code)
        status_pub.publish(msg)

        status_name = STATUS_NAMES.get(
            status_code,
            "UNKNOWN_STATUS",
        )

        node.get_logger().info(
            f"📡 HMI status: {status_code} ({status_name})"
        )

    def publish_done(result):
        """
        향수 제조 최종 성공 여부를 전송한다.
        """

        msg = Bool()
        msg.data = bool(result)
        done_pub.publish(msg)

        node.get_logger().info(
            f"📡 Published perfume_done: {result}"
        )

    # ========================================================
    # 주문 데이터
    # ========================================================

    order = [0, 0, 0, 0, 0, 0]
    order_received = False

    def order_callback(request, response):
        nonlocal order_received

        order[0] = request.scent1
        order[1] = request.scent2
        order[2] = request.scent3
        order[3] = request.scent4
        order[4] = request.scent5
        order[5] = request.scent6

        order_received = True

        # 10: 주문 접수
        publish_status(STATUS_ORDER_RECEIVED)

        node.get_logger().info(
            f"📦 Order received: {order}"
        )

        response.success = True
        return response

    node.create_service(
        Order,
        "/order_perfume",
        order_callback,
    )

    node.get_logger().info("Order perfume service server ready")

    # 0: 시스템 시작 후 주문 대기
    publish_status(STATUS_IDLE)

    try:
        while rclpy.ok():
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

            if not order_received:
                continue

            order_received = False

            # 20: 향수 제조 시작
            publish_status(STATUS_PROCESS_START)

            node.get_logger().info(f"🔥 Start processing order: {order}")

            wait(1.0)

            # =================================================
            # 향수병 위치로 이동
            # =================================================

            # 30: 향수병 위치로 이동 중
            publish_status(STATUS_MOVE_TO_PERFUME)

            controller.move_to_pose(
                PERFUME_POSE,
                from_home=True,
                down=103.58,
                velocity=100,
                acceleration=50,
            )

            # =================================================
            # 향수병 배치 확인
            # =================================================

            # 40: 향수병 배치 확인 중
            publish_status(STATUS_CHECK_PERFUME)

            if not controller.check_perfume_bottle():
                node.get_logger().error("❌ 향수병이 없습니다.")

                # 실패했지만 로봇은 홈으로 복귀
                publish_status(STATUS_RETURN_HOME)
                controller.move_to_home()

                publish_done(False)

                # 다시 주문 대기 상태
                publish_status(STATUS_READY)
                continue

            # =================================================
            # 향수병 뚜껑 열기
            # =================================================

            # 50: 향수병 뚜껑 열기
            publish_status(STATUS_OPEN_PERFUME_LID)

            controller.open_lid(cycle=3)

            # =================================================
            # 향수병 뚜껑 보관함에 거치
            # =================================================

            # 60: 향수병 뚜껑 거치
            publish_status(STATUS_STORE_PERFUME_LID)

            node.get_logger().info("🔥 향수병 뚜껑 거치")

            wait(1.0)

            # 열린 뚜껑 잡기
            controller.grip()

            controller.move_to_pose(
                goal_pose=PERFUME_LID_POSE,
                up=103.58,
                down=117.2,
            )

            controller.release()

            # =================================================
            # 향료 투입
            # =================================================

            # 100: 전체 향료 투입 공정 시작
            publish_status(STATUS_SCENT_PROCESS_START)

            for i in range(6):
                scent_pose = scent_positions[i]
                drops = order[i]
                first = True

                for drop in range(drops):
                    node.get_logger().info(f"💧 scent{i + 1}: {drop + 1}/{drops} drops")

                    if first:
                        # 110: 향료병 위치로 이동
                        publish_status(STATUS_MOVE_TO_SCENT)

                        node.get_logger().info(f"📍➔📍 scent{i + 1} 향료병으로 이동")

                        wait(0.5)

                        controller.move_to_pose(
                            goal_pose=scent_pose,
                            up=117.2,
                            down=81.23,
                        )

                    # ==========================================
                    # 스포이드 용액 추출
                    # ==========================================

                    # 120: 향료 추출
                    publish_status(STATUS_EXTRACT_SCENT)

                    node.get_logger().info("🧪 향료 추출 시작")

                    wait(0.5)

                    controller.grip()
                    controller.release()

                    node.get_logger().info("🧪 향료 추출 완료")

                    # ==========================================
                    # 향료병 뚜껑 잡기
                    # ==========================================

                    # 130: 향료병 뚜껑 잡기
                    publish_status(STATUS_GRIP_SCENT_LID)

                    node.get_logger().info("🦾 뚜껑 잡기 시작")

                    wait(0.5)

                    # 스포이드 위치에서 뚜껑 위치까지 하강
                    controller.move_down(z=18.16)
                    controller.grip()

                    # 140: 향료병 뚜껑 열기
                    publish_status(STATUS_OPEN_SCENT_LID)

                    movej(
                        posj([0, 0, 0, 0, 0, -45]),
                        vel=50,
                        acc=50,
                        mod=DR_MV_MOD_REL,
                    )

                    controller.move_up(z=18.16)


                    node.get_logger().info("🦾 뚜껑 잡기 완료")

                    # ==========================================
                    # 향수병으로 이동
                    # ==========================================

                    # 150: 향수병으로 이동
                    publish_status(STATUS_MOVE_TO_MIX_BOTTLE)

                    node.get_logger().info("📍➔📍 향수병으로 이동")

                    controller.move_to_pose(
                        goal_pose=PERFUME_POSE,
                        up=81.23,
                        down=94.58,
                    )

                    wait(0.5)

                    task_compliance_ctrl([300,300,300,200,200,200], 0)
                    wait(0.5)
                    set_desired_force([0,0,-13,0,0,0], [0,0,1,0,0,0])

                    while 1:
                        var_force = get_tool_force()
                        if var_force[2] > 13:
                            release_force()
                            release_compliance_ctrl()
                            break

                    # 향수병 위에 향료병 뚜껑 배치
                    controller.release()

                    # ==========================================
                    # 향료 투출
                    # ==========================================

                    controller.move_up(z=16.16)

                    # 160: 향료 투입
                    publish_status(STATUS_DISPENSE_SCENT)

                    node.get_logger().info("🧪 향료 투출 시작")

                    wait(0.5)

                    controller.grip()
                    controller.release()

                    node.get_logger().info("🧪 향료 투출 완료")

                    # ==========================================
                    # 향료병 뚜껑 다시 잡기
                    # ==========================================

                    # 170: 향료병 뚜껑 다시 잡기
                    publish_status(STATUS_GRIP_SCENT_LID_RETURN)

                    node.get_logger().info("🦾 뚜껑 잡기 시작")

                    wait(0.5)

                    controller.move_down(z=16.16)
                    controller.grip()

                    movej(posj([0,0,0,0,0,-45]), vel=80, acc=80, mod=DR_MV_MOD_REL)

                    controller.move_up(z=16.16)

                    node.get_logger().info("🦾 뚜껑 잡기 완료")

                    # ==========================================
                    # 향료병 위치로 복귀
                    # ==========================================

                    # 180: 향료병으로 복귀
                    publish_status(STATUS_RETURN_TO_SCENT)

                    node.get_logger().info(f"📍➔📍 scent{i + 1} 향료병으로 이동")

                    wait(0.5)

                    controller.move_to_pose(
                        goal_pose=scent_pose,
                        up=100.58,
                        down=99.39,
                    )

                    # ==========================================
                    # 향료병 뚜껑 닫기
                    # ==========================================

                    # 190: 향료병 뚜껑 닫기
                    publish_status(STATUS_CLOSE_SCENT_LID)

                    node.get_logger().info("✋ 뚜껑 놓기")

                    wait(0.5)
                    controller.release()

                    controller.move_up(z=18.16) # 내려간 만큼 올라가기

                    first = False

            # 200: 전체 향료 투입 완료
            publish_status(STATUS_SCENT_PROCESS_DONE)

            # =================================================
            # 향수병 뚜껑 가져오기
            # =================================================

            # 210: 향수병 뚜껑 가져오기
            publish_status(STATUS_GET_PERFUME_LID)

            node.get_logger().info("📍➔📍 향수병 뚜껑 거치대 위치로 이동")

            wait(0.5)

            controller.move_to_pose(
                PERFUME_LID_POSE,
                up=99.39,
                down=116.2,
            )

            controller.grip()

            # =================================================
            # 향수병 위치로 뚜껑 이동
            # =================================================

            # 220: 향수병 뚜껑 이동
            publish_status(STATUS_MOVE_LID_TO_PERFUME)

            node.get_logger().info("📍➔📍 향수병 위치로 이동")

            wait(0.5)

            controller.move_to_pose(
                PERFUME_POSE,
                up=116.2,
                down=100.58,
            )

            # =================================================
            # 향수병 뚜껑 닫기
            # =================================================

            # 230: 향수병 뚜껑 닫기
            publish_status(STATUS_CLOSE_PERFUME_LID)

            controller.close_lid(cycle=4)

            # =================================================
            # 완성된 향수병 잡기
            # =================================================

            # 240: 완성된 향수병 잡기
            publish_status(STATUS_GRIP_FINISHED_PERFUME)

            node.get_logger().info("🦾 향수병 뚜껑 잡기 시작")

            wait(0.5)

            controller.grip()

            node.get_logger().info("🦾 향수병 뚜껑 잡기 완료")

            controller.move_up(z=70)

            # =================================================
            # 홈 위치 이동
            # =================================================

            # 250: 홈 위치로 이동
            publish_status(STATUS_MOVE_TO_HOME)

            controller.move_to_home()
            wait(0.5)

            # =================================================
            # 향수 혼합
            # =================================================

            # 260: 기본 혼합
            publish_status(STATUS_SHAKE_PERFUME)

            controller.shake_perfume(cycle=5)

            # 270: 기울임 혼합
            publish_status(STATUS_TILT_MIX_PERFUME)

            controller.shake_perfume2(
                tilt_angle=30,
                amp=20,
            )

            # =================================================
            # 픽업 장소로 이동
            # =================================================

            # 280: 픽업 위치로 이동
            publish_status(STATUS_MOVE_TO_PICKUP)

            node.get_logger().info("📍➔📍 픽업장소로 이동")

            wait(0.5)

            controller.move_to_pose(
                PICKUP_POSE,
                from_home=True,
                down=80,
            )

            # =================================================
            # 힘 제어를 사용해 향수병 내려놓기
            # =================================================

            # 290: 향수병 내려놓기
            publish_status(STATUS_PLACE_PERFUME)

            task_compliance_ctrl([300, 300, 300, 200, 200, 200],0)

            wait(0.5)

            set_desired_force([0, 0, -30, 0, 0, 0],[0, 0, 1, 0, 0, 0])

            while rclpy.ok():
                var_force = get_tool_force()

                if var_force[2] > 10:
                    release_force()
                    release_compliance_ctrl()
                    break

            # =================================================
            # 향수병 놓기
            # =================================================

            # 300: 향수병 release
            publish_status(STATUS_RELEASE_PERFUME)

            controller.release()

            # =================================================
            # 제조 완료 후 홈 복귀
            # =================================================

            # 320: 초기 위치 복귀
            publish_status(STATUS_RETURN_HOME)

            controller.move_to_home()

            # =================================================
            # 제조 완료
            # =================================================

            # 310: 제조 완료
            publish_status(STATUS_PROCESS_COMPLETE)

            node.get_logger().info("🫙 perfume is ready")

            publish_done(True)

            # 330: 다음 주문 대기
            publish_status(STATUS_READY)

    except KeyboardInterrupt:
        node.get_logger().info(
            "Program Stopped"
        )

    except TimeoutError as e:
        node.get_logger().error(
            f"Timeout error: {e}"
        )

        publish_done(False)

    except Exception as e:
        node.get_logger().error(
            f"Unexpected error: {type(e).__name__}: {e}"
        )

        publish_done(False)

    finally:
        try:
            release_force()
            release_compliance_ctrl()
        except Exception:
            pass

        try:
            publish_status(STATUS_RETURN_HOME)
            controller.move_to_home()
        except Exception as e:
            node.get_logger().error(
                f"Failed to move home: {e}"
            )

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()