import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

VELOCITY, ACC = 200, 150

DR_BASE = 0
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1

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

    # =========================
    # 기본 Pose 설정
    # =========================
    init_pose = posj([0, 0, 90, 0, 90, 0])

    # 향료병 위쪽 기준점
    X_essence_up = posx(200, 250, 400, 0, 180, 0)

    # 향수 공병 위쪽 기준점
    X_perfume_up = posx(500, 250, 400, 0, 180, 0)

    # 병 사이 이동 시 안전 경유점
    X_mid = posx(350, 250, 480, 0, 180, 0)

    # 병 안으로 내려가는 깊이
    Z_DOWN = 100

    # moveb 곡선 블렌딩 반경
    CURVE_RADIUS = 50

    # =========================
    # 상대좌표 Down / Up 함수
    # =========================
    def move_down(z=Z_DOWN):
        node.get_logger().info(f"Move down {z}mm")

        movel(
            posx(0, 0, -z, 0, 0, 0),
            vel=VELOCITY,
            acc=ACC,
            ref=DR_BASE,
            mod=DR_MV_MOD_REL,
        )

    def move_up(z=Z_DOWN):
        node.get_logger().info(f"Move up {z}mm")

        movel(
            posx(0, 0, z, 0, 0, 0),
            vel=VELOCITY,
            acc=ACC,
            ref=DR_BASE,
            mod=DR_MV_MOD_REL,
        )

    # =========================
    # 절대좌표 이동 함수
    # =========================
    def move_abs(target_pose):
        movel(
            target_pose,
            vel=VELOCITY,
            acc=ACC,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )

    # =========================
    # moveb 곡선 이동 함수
    # 현재 위치 → mid_pose → goal_pose
    #
    # 핵심:
    # - start_pose를 넣지 않는다.
    # - 현재 위치가 자동으로 시작점이다.
    # - X_mid에서 radius를 주어 부드럽게 꺾이도록 한다.
    # =========================
    def move_curve_to(goal_pose, mid_pose=X_mid, radius=CURVE_RADIUS):
        node.get_logger().info("Starting moveb curved trajectory")

        b_list = [
            # 현재 위치에서 X_mid로 이동하되,
            # X_mid에서 radius만큼 블렌딩
            posb(DR_LINE, mid_pose, radius=radius),

            # X_mid를 거쳐 goal_pose로 이동
            # 마지막 지점에서는 정확히 도착해야 하므로 radius=0
            posb(DR_LINE, goal_pose, radius=0),
        ]

        moveb(
            b_list,
            vel=VELOCITY,
            acc=ACC,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )

        node.get_logger().info("Moveb curved trajectory finished")

    try:
        print("Starting Move Only Program")

        set_tool("Tool Weight1")
        set_tcp("GripperDA_v1")

        # =========================
        # 1. 초기 Joint 위치 이동
        # =========================
        node.get_logger().info("1. Move to initial joint position")
        movej(
            init_pose,
            vel=VELOCITY,
            acc=ACC,
            mod=DR_MV_MOD_ABS,
        )
        wait(1)

        # =========================
        # 2. 이니셜 포즈 → 향료 기준좌표
        # =========================
        node.get_logger().info("2. Move to essence up pose")
        move_abs(X_essence_up)
        wait(1)

        # =========================
        # 3. 향료 기준좌표 → Down
        # =========================
        node.get_logger().info("3. Essence down")
        move_down()
        wait(1)

        # =========================
        # 4. 향료 기준좌표 → Up
        # =========================
        node.get_logger().info("4. Essence up")
        move_up()
        wait(1)

        # =========================
        # 5. 향료 기준좌표 → 향수 기준좌표
        # 곡선 이동: 현재 위치 X_essence_up → X_mid → X_perfume_up
        # =========================
        node.get_logger().info("5. Move essence up to perfume up")
        move_curve_to(X_perfume_up)
        wait(1)

        # =========================
        # 6. 향수 기준좌표 → Down
        # =========================
        node.get_logger().info("6. Perfume down")
        move_down()
        wait(1)

        # =========================
        # 7. 향수 기준좌표 → Up
        # =========================
        node.get_logger().info("7. Perfume up")
        move_up()
        wait(1)

        # =========================
        # 8. 향수 기준좌표 → 향료 기준좌표
        # 곡선 이동: 현재 위치 X_perfume_up → X_mid → X_essence_up
        # =========================
        node.get_logger().info("8. Move perfume up to essence up")
        move_curve_to(X_essence_up)
        wait(1)

        # =========================
        # 9. 향료 기준좌표 → Down
        # =========================
        node.get_logger().info("9. Essence down again")
        move_down()
        wait(1)

        node.get_logger().info("Program finished")

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