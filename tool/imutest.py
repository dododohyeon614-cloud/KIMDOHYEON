#!/usr/bin/env python3
"""
imu_checker.py - IMU 나침반 모드(Absolute Heading) 진단 툴
가만히 멈춰있을 때 방위각이 계속 변하는지(Drift)를 추적하여 지자기 센서의 락킹 상태를 판별합니다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import math
import time
from collections import deque

class ImuCheckerNode(Node):
    def __init__(self):
        super().__init__('imu_checker_node')

        self.subscription = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        # 최근 3초간의 Yaw(방위) 데이터 저장용 큐 (20Hz 기준 약 60개)
        self.yaw_history = deque(maxlen=60)
        
        self.current_yaw = 0.0
        self.is_moving = False
        self.last_print_time = time.time()

        self.get_logger().info("🧭 IMU 진단 툴이 시작되었습니다. 센서를 가만히 두세요.")
        self.create_timer(0.5, self.print_status)  # 0.5초마다 터미널 출력 업데이트

    def quat_to_yaw(self, x, y, z, w):
        # 쿼터니언을 오일러 Yaw(방위각) 성분으로 변환 (외부 라이브러리 없이 수학 연산)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.degrees(math.atan2(t3, t4))

    def imu_callback(self, msg):
        # 1. 방위각 추출
        q = msg.orientation
        self.current_yaw = self.quat_to_yaw(q.x, q.y, q.z, q.w)
        
        # 2. 움직임 감지 (자이로스코프 각속도 기준)
        # 각속도가 1.0 deg/s 이상이면 기기가 움직이고 있다고 판단
        gyro_z = math.degrees(abs(msg.angular_velocity.z))
        if gyro_z > 1.0:
            self.is_moving = True
            self.yaw_history.clear() # 움직이면 기록 초기화
        else:
            self.is_moving = False
            self.yaw_history.append(self.current_yaw)

    def print_status(self):
        # 데이터가 충분히 쌓이지 않았거나 기기가 움직이는 중일 때
        if self.is_moving or len(self.yaw_history) < 30:
            print(f"\r[측정 중...] 현재 방위: {self.current_yaw:6.1f}° | 기기를 3초 이상 가만히 두세요...       ", end="")
            return

        # 3초간 가만히 있었을 때의 방위각 변화량(Drift) 계산
        max_yaw = max(self.yaw_history)
        min_yaw = min(self.yaw_history)
        drift = max_yaw - min_yaw

        # 판별 로직
        # 0.5도 이내로 변동이 없으면 지자기 센서가 꽉 잡아주고 있는 정상 상태!
        if drift < 0.5:
            status_msg = "✅ 정상 (나침반 락킹됨 - Absolute Mode)"
        else:
            status_msg = f"❌ 불량 (초당 {drift/3.0:.2f}°씩 흘러감 - Relative Mode)"

        # 결과 출력
        print(f"\r[결과] 현재 방위: {self.current_yaw:6.1f}° | 변화량: {drift:4.2f}° | 상태: {status_msg}", end="")

def main(args=None):
    rclpy.init(args=args)
    node = ImuCheckerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()