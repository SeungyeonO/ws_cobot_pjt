import rclpy
import DR_init
from perfume_order_srv.srv import Order
from std_msgs.msg import Bool


ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m0609"
VELOCITY, ACC = 300, 200

DR_init.__dsr__id   = ROBOT_ID
DR_init.__dsr__model= ROBOT_MODEL


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
            movel,
            wait,
            DR_MV_MOD_REL,
            DR_BASE,
            DR_TOOL
        )

        from DR_common2 import posj, posx

        from cobot_control.control_functions import (
            RobotController, 
            PERFUME_POSE, 
            PICKUP_POSE,
            PERFUME_LID_POSE, 
            scent_positions
        )

    except ImportError as e:
        node.get_logger().info(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    set_tcp("GripperDA_v1")
    set_tool("Tool Weight1")

    controller = RobotController(node)
    controller.move_to_home()

    
    order = [0,0,0,0,0,0]
    order_received = False

    def order_callback(request, response):

        order[0] = request.scent1
        order[1] = request.scent2
        order[2] = request.scent3
        order[3] = request.scent4
        order[4] = request.scent5
        order[5] = request.scent6

        nonlocal order_received
        order_received = True

        response.success = True
        return response

    node.create_service(
        Order,
        "/order_perfume",
        order_callback,
    )

    done_pub = node.create_publisher(
        Bool,
        "/perfume_done",
        10
    )

    node.get_logger().info("Order perfume service server ready")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            if order_received:
                order_received = False
                node.get_logger().info(f"🔥 Start processing order: {order}")
                wait(1.0)

                controller.move_to_pose(PERFUME_POSE, from_home=True, down=103.58, velocity=30, acceleration=30)    # 향수병 상단 - 향수병 뚜껑

                # ========== 향수병 배치 x 상황의 에러 처리 ============
                if not controller.check_perfume_bottle():
                    node.get_logger().info(f"❌ 향수병이 없습니다.")

                    controller.move_to_home()

                    done_msg = Bool()
                    done_msg.data = False
                    done_pub.publish(done_msg)

                    continue
                # =================================================
                
                controller.open_lid(cycle=3)

                # ======= 뚜껑 보관함에 거치 ==============
                node.get_logger().info("🔥 향수병 뚜껑 거치")
                wait(1.0)
                controller.grip()   # 열린 뚜껑 잡기
                controller.move_to_pose(goal_pose=PERFUME_LID_POSE, up=103.58, down=116.2) # 향수 뚜껑 보관함 상단 - 향수 뚜껑 놓는 위치
                controller.release()
                # ======================================

                for i in range(6):
                    scent_pose = scent_positions[i]
                    drops = order[i]
                    first = True

                    for drop in range(drops):
                        node.get_logger().info(f"💧 scent{i+1}: {drop+1}/{drops} drops")
                        if first:
                            node.get_logger().info(f"📍➔📍 scent{i+1} 향료병으로 이동")
                            wait(0.5)
                            controller.move_to_pose(goal_pose=scent_pose, up=116.2, down=81.23)   # scent 상단 - 스포이드 위치
                        
                        # ===== 스포이드 용액 추출 =======
                        node.get_logger().info("🧪 향료 추출 시작")
                        wait(0.5)
                        controller.grip()
                        controller.release()
                        node.get_logger().info("🧪 향료 추출 완료")
                        # =============================
                        
                        # ===== 뚜껑 부분 잡기 ==========
                        node.get_logger().info("🦾 뚜껑 잡기 시작")
                        wait(0.5)

                        controller.move_down(z=18.16) # 스포이드 - 뚜껑
                        controller.grip()

                        movej(posj([0,0,0,0,0,-45]), vel=50, acc=50, mod=DR_MV_MOD_REL)

                        controller.move_up(z=18.16) # 내려간 만큼 올라가기

                        movej(posj([0,0,0,0,0,45]), vel=50, acc=50, mod=DR_MV_MOD_REL)
                        
                        node.get_logger().info("🦾 뚜껑 잡기 완료")
                        # =============================

                        node.get_logger().info(f"📍➔📍 향수병으로 이동")
                        controller.move_to_pose(goal_pose=PERFUME_POSE, up=81.23, down=100.58) # 향수병 상단 - 뚜껑 놓을 위치
                        wait(0.5)

                        controller.release()
                        # ===== 스포이드 용액 투출 ========
                        controller.move_up(z=16.16)

                        node.get_logger().info("🧪 향료 투출 시작")
                        wait(0.5)

                        controller.grip()
                        controller.release()

                        node.get_logger().info("🧪 향료 투출 완료")

                        # =============================

                        # ===== 뚜껑 부분 잡기 ==========
                        node.get_logger().info("🦾 뚜껑 잡기 시작")
                        wait(0.5)

                        controller.move_down(z=16.16) # 뚜껑 위치로 이동
                        controller.grip()

                        # movej(posj([0,0,0,0,0,-45]), vel=50, acc=50, mod=DR_MV_MOD_REL)

                        controller.move_up(z=16.16) # 내려간 만큼 올라가기

                        # movej(posj([0,0,0,0,0,45]), vel=50, acc=50, mod=DR_MV_MOD_REL)
                        
                        node.get_logger().info("🦾 뚜껑 잡기 완료")
                        # =============================

                        node.get_logger().info(f"📍➔📍 scent{i+1} 향료병으로 이동")
                        wait(0.5)
                        controller.move_to_pose(goal_pose=scent_pose, up=100.58, down=99.39)  # 향료 상단 - 뚜껑 놓을 위치

                        node.get_logger().info(f"✋ 뚜껑 놓기")
                        wait(0.5)
                        controller.release()

                        first = False
                
                node.get_logger().info(f"📍➔📍 향수병 뚜껑 거치대 위치로 이동")
                wait(0.5)

                # ================== 수정 필요 =================
                controller.move_to_pose(PERFUME_LID_POSE, up=99.39, down=116.2)
                controller.grip()   
                # ============================================

                node.get_logger().info(f"📍➔📍 향수병 위치로 이동")
                wait(0.5)
                controller.move_to_pose(PERFUME_POSE, up=116.2, down=100.58)    # 향수병 - 향수병 뚜껑 놓을 위치
                controller.close_lid(cycle=4)

                # ===== 뚜껑 부분 잡기 ==========
                node.get_logger().info("🦾 향수병 뚜껑 잡기 시작")
                wait(0.5)

                # controller.move_down(z=18.7) # 뚜껑 위치로 이동
                controller.grip()
                

                node.get_logger().info("🦾 향수병 뚜껑 잡기 완료")
                # =============================
                controller.move_up(z=70) # home pose 이동 전에 위로 올리기
                controller.move_to_home()
                wait(0.5)
                
                controller.shake_perfume(cycle=5)
                
                controller.shake_perfume2(tilt_angle=30, amp=20)

                

                # ====== 픽업 공간에 pick&place =======
                node.get_logger().info(f"📍➔📍 픽업장소으로 이동")
                wait(0.5)
                controller.move_to_pose(PICKUP_POSE, from_home=True, down=80)    # pickup 상단 - 힘제어 시작점

                task_compliance_ctrl([300,300,300,200,200,200], 0)
                wait(0.5)
                set_desired_force([0,0,-30,0,0,0], [0,0,1,0,0,0])

                while 1:
                    var_force = get_tool_force()
                    if var_force[2] > 10:
                        release_force()
                        release_compliance_ctrl()
                        break
                # 바닥 위치 찾아야함
                # controller.move_down(z=20)

                controller.release()
                # ===================================

                controller.move_to_home()

                node.get_logger().info("🫙 perfume is ready")

                done_msg = Bool()
                done_msg.data = True
                done_pub.publish(done_msg)
                node.get_logger().info("✅ Published perfume_done: True")


    except KeyboardInterrupt:
        node.get_logger().info("Program Stopped")
    except TimeoutError as e:
        node.get_logger().error(str(e))

    finally:
        try:
            controller.move_to_home()
        except Exception as e:
            node.get_logger().error(f"Failed to move home: {e}")

        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()