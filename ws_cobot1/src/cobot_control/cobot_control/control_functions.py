from DSR_ROBOT2 import (
        set_digital_output,
        task_compliance_ctrl,
        set_desired_force,
        release_force,
        release_compliance_ctrl,
        get_current_posx,
        movej,
        movel,
        moveb,
        wait,
        DR_MV_MOD_ABS,
        DR_MV_MOD_REL,
        DR_TOOL,
        DR_BASE,
        DR_CIRCLE,
        DR_LINE,
        OFF,
        ON,
    )

from DR_common2 import posj, posx, posb


SCENT1_POSE = posx([336.37,-65.62,135.19,0,180,0])
SCENT2_POSE = posx([370,7,94,0,180,0])
SCENT3_POSE = posx([431.11,-66.15,135.19,0,180,0])
SCENT4_POSE = posx([370,52,94,0,180,0])
SCENT5_POSE = posx([290,97,94,0,180,0])
SCENT6_POSE = posx([370,97,94,0,180,0])
PERFUME_POSE = posx([366.77,43.66,135.19,0,180,0])
PERFUME_LID_POSE = posx([311.46,50.85,115.19,0,180,0])
PICKUP_POSE = posx([525.12,50.86,135.10,0,180,0])

scent_positions = [SCENT1_POSE, SCENT2_POSE, SCENT3_POSE, SCENT4_POSE, SCENT5_POSE, SCENT6_POSE]


class RobotController:
    def __init__(self, node):
        self.node = node

        self.node.get_logger().info("RobotController initialized")

    def grip(self):
        self.node.get_logger().info("Grip: digital output 1 ON, 2 OFF")

        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        wait(0.5)

    def release(self):
        self.node.get_logger().info("Release: digital output 1 OFF, 2 ON")

        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(0.5)

    def move_to_home(self, velocity=200, acceleration=60):
        homej = posj([0, 0, 90, 0, 90, 0])
        self.node.get_logger().info(f"Moving to home position: {homej}")
        movej(homej, vel=velocity, acc=acceleration)


    def open_lid(self, velocity=300, acceleration=200, cycle=3):
        rotate_cw = posj([0, 0, 0, 0, 0, -180])
        rotate_ccw = posj([0, 0, 0, 0, 0, 180])

        self.node.get_logger().info("🚀 Start opening lid")

        for i in range(cycle):
            self.node.get_logger().info(f"Open lid cycle {i + 1}/{cycle}")

            self.grip()
            movej(rotate_cw, vel=velocity, acc=acceleration, mod=DR_MV_MOD_REL)
            self.release()
            movej(rotate_ccw, vel=velocity, acc=acceleration, mod=DR_MV_MOD_REL)

        self.node.get_logger().info("🙏 Finished opening lid")


    def close_lid(self, velocity=300, acceleration=200, cycle=4):
        rotate_cw = posj([0, 0, 0, 0, 0, -180])
        rotate_ccw = posj([0, 0, 0, 0, 0, 180])

        self.node.get_logger().info("🚀 Start Closing lid")

        # ===========  뚜껑 제대로 결착하기 위한 힘제어 + 회전 모션  ============
        print("순응제어 ON")
        task_compliance_ctrl([3000,3000,500,200,200,200], 0)
        wait(0.5)
        print("힘제어 ON")
        set_desired_force([0,0,-10,0,0,0], [0,0,1,0,0,0])
        print("set_desired_force 완료")
        wait(1.0)
        print("movel 하기 직전")
        movel(
            [0, 0, 0, 0, 0, 45],
            vel=100,
            acc=80,
            ref=DR_TOOL,
            mod=DR_MV_MOD_REL
        )

        print("movel 완료")

        release_force()
        release_compliance_ctrl()

        self.release()
        movel(
            [0, 0, 0, 0, 0, -45],
            vel=100,
            acc=80,
            ref=DR_TOOL,
            mod=DR_MV_MOD_REL
        )
        # ====================================================================

        # ============= 힘 제어 OFF + 4번 180도 회전 모션 ===================
        for i in range(cycle):
            self.node.get_logger().info(f"=== Cycle {i+1}/{cycle} ===")
            self.grip()
            movej(rotate_ccw, vel=velocity, acc=acceleration, mod=DR_MV_MOD_REL)
            self.release()
            movej(rotate_cw, vel=velocity, acc=acceleration, mod=DR_MV_MOD_REL)

        # ===============================================================

        self.node.get_logger().info("🙏 Finished closing lid")


    def shake_perfume(self, R=20, cycle=15):
        
        circle_path = []

        for _ in range(cycle):
            circle_path += [
                posb(DR_CIRCLE, posx(R, 0, 0, 0, 0, 0), posx(0, -R, 0, 0, 0, 0), radius=5),
                posb(DR_CIRCLE, posx(-R, 0, 0, 0, 0, 0), posx(0, R, 0, 0, 0, 0), radius=5),
            ]

        self.node.get_logger().info(f"🚀 Start Shaking Perfume: radius={R}, cycle={cycle}")

        moveb(circle_path, vel=120, acc=200, ref=DR_TOOL, mod=DR_MV_MOD_REL)

        self.node.get_logger().info("🙏 Finished Shaking Perfume")

    # ==============  move_to_pose에서 활용할 함수 =================

    def move_down(self, z=0, velocity=300, acceleration=200):
        self.node.get_logger().info(f"Move down {z}mm")

        movel(
            posx(0, 0, -z, 0, 0, 0),
            vel=velocity,
            acc=acceleration,
            ref=DR_BASE,
            mod=DR_MV_MOD_REL,
        )

    def move_up(self, z=0, velocity=300, acceleration=200):
        self.node.get_logger().info(f"Move up {z}mm")

        movel(
            posx(0, 0, z, 0, 0, 0),
            vel=velocity,
            acc=acceleration,
            ref=DR_BASE,
            mod=DR_MV_MOD_REL,
        )

    def move_abs(self, target_pose, velocity=300, acceleration=200):
        self.node.get_logger().info(f"Move to absolute target pose: {target_pose}")
        movel(
            target_pose,
            vel=velocity,
            acc=acceleration,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )

    
    def move_curve_to(self, goal_pose, mid_pose, velocity=300, acceleration=200, radius=50):
        self.node.get_logger().info("Starting moveb curved trajectory")

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
            vel=velocity,
            acc=acceleration,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )

        self.node.get_logger().info("Moveb curved trajectory finished")

    # ==========================================================

    # def move_to_pose(self, goal_pose, from_home=False, up=60, down=60, velocity=200, acceleration=60, radius=5):
    #     self.node.get_logger().info(f"Moving to pose: {goal_pose}")
    #     movel(goal_pose, vel=velocity, acc=acceleration, ref=DR_BASE, mod=DR_MV_MOD_REL)

    def move_to_pose(self, goal_pose, from_home=False, up=60, down=60, velocity=200, acceleration=60, radius=5):
        self.node.get_logger().info(f"Moving to pose: {goal_pose}")

        current_pose = get_current_posx()[0]

        up_pose = posx(
            current_pose[0],
            current_pose[1],
            current_pose[2] + up,
            current_pose[3],
            current_pose[4],
            current_pose[5]
        )

        # goal_pose 위에서 down만큼 내려간 최종 pose
        down_pose = posx(
            goal_pose[0],
            goal_pose[1],
            goal_pose[2] - down,
            goal_pose[3],
            goal_pose[4],
            goal_pose[5]
        )

        if from_home:
            path = [
                posb(DR_LINE, goal_pose, radius=radius), 
                posb(DR_LINE, down_pose, radius=0)        
            ]
        
        else:
            path = [
                posb(DR_LINE, up_pose, radius=radius),      # 현재 위치에서 위로 빠짐
                posb(DR_LINE, goal_pose, radius=radius),    # 다음 향료병/향수병 위로 이동
                posb(DR_LINE, down_pose, radius=0)          # 마지막은 정확히 멈추기
            ]

        moveb(
            path,
            vel=velocity,
            acc=acceleration,
            ref=DR_BASE
        )