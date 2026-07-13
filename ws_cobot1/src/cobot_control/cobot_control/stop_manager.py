import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from dsr_msgs2.srv import MoveStop


ROBOT_ID = "dsr01"

# 실제 시스템에서 ros2 service list로 확인한 이름으로 수정
MOVE_STOP_SERVICE = f"/{ROBOT_ID}/motion/move_stop"

# 정지 모드
DR_SSTOP = 2


class PerfumeStopNode(Node):

    def __init__(self):
        super().__init__("perfume_stop_node")

        self.stop_client = self.create_client(
            MoveStop,
            MOVE_STOP_SERVICE,
        )

        self.stop_sub = self.create_subscription(
            Bool,
            "/stop_perfume",
            self.stop_callback,
            10,
        )

        self.stop_lock = threading.Lock()
        self.stop_processing = False

        self.get_logger().info(
            f"Stop node ready: topic=/stop_perfume, "
            f"service={MOVE_STOP_SERVICE}"
        )

    def stop_callback(self, msg):
        if not msg.data:
            return

        with self.stop_lock:
            if self.stop_processing:
                self.get_logger().warn("MoveStop 요청을 이미 처리 중입니다.")
                return

            self.stop_processing = True

        # callback 안에서 대기하지 않도록 별도 스레드 사용
        threading.Thread(
            target=self.send_stop_repeatedly,
            daemon=True,
        ).start()

    def send_stop_repeatedly(self):
        try:
            self.get_logger().warn("🛑 /stop_perfume=True 수신")

            if not self.stop_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error(f"MoveStop service unavailable: {MOVE_STOP_SERVICE}")
                return

            for attempt in range(3):
                request = MoveStop.Request()
                request.stop_mode = DR_SSTOP

                future = self.stop_client.call_async(request)

                self.get_logger().warn(f"🛑 MoveStop 요청 {attempt+1}/3 전송")

                # 외부 stop 노드는 DSR_ROBOT2 모션 함수를 실행하지 않으므로
                # future 완료를 잠깐 기다려도 메인 제어 노드와 충돌하지 않음
                end_time = time.monotonic() + 1.0

                while (
                    rclpy.ok()
                    and not future.done()
                    and time.monotonic() < end_time
                ):
                    time.sleep(0.01)

                if future.done():
                    try:
                        response = future.result()
                        self.get_logger().info(f"MoveStop {attempt+1}/3 응답: {response}")

                    except Exception as e:
                        self.get_logger().error(f"MoveStop {attempt+1}/3 실패: {e}")
                        
                else:
                    self.get_logger().warn(f"MoveStop {attempt+1}/3 응답 대기 시간 초과")

                time.sleep(0.1)

        finally:
            with self.stop_lock:
                self.stop_processing = False


def main(args=None):
    rclpy.init(args=args)

    node = PerfumeStopNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()