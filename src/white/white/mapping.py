#!/usr/bin/env python3
"""
mapping.py ― 경로 매핑 노드 [자동 리모델링 포함]

기능
1. /ego_state를 5Hz로 CSV 저장
2. direction 컬럼 유지 (+1 전진 / -1 후진)
3. 매핑 종료 시 원본 route_YYYYMMDD_HHMMSS.csv 저장
4. 곧바로 route_YYYYMMDD_HHMMSS_remodeled.csv 자동 생성

주의
- route_remodeler.py를 같은 white 패키지 폴더에 같이 넣어야 자동 리모델링 import가 됩니다.
- 자동 리모델링 실패 시에도 원본 CSV는 그대로 남습니다.
"""

import os
import csv
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool
from geometry_msgs.msg import Twist

# 패키지 설치 후 실행 / 로컬 단독 실행 둘 다 대응
try:
    from white.route_remodeler import remodel_route
except Exception:
    try:
        from route_remodeler import remodel_route
    except Exception:
        remodel_route = None


class MappingNode(Node):
    def __init__(self):
        super().__init__("mapping_node")
        self.data_dir = os.path.expanduser("~/white_ws/gps_data")
        os.makedirs(self.data_dir, exist_ok=True)

        self.is_mapping = False
        self.csv_file = None
        self.csv_writer = None
        self.current_csv_path = None
        self._flush_counter = 0          # 🔧 BUG8: 주기적 flush 카운터
        self._remodel_thread = None      # 🔧 BUG9: 비동기 리모델링 스레드

        # 자동 리모델링 기본값
        self.auto_remodel_enabled = True
        self.remodel_spacing = 0.25      # waypoint 간격 [m]
        self.remodel_epsilon = 0.08      # 직선 smoothing 최대 이동 [m]
        self.remodel_smooth_iter = 1     # 직선 smoothing 반복 횟수

        # ego_state
        self.lat = None
        self.lon = None
        self.heading = 0.0
        self.speed = 0.0
        self.steer = 0.0
        self.pitch = 0.0
        self.terrain = 0.0

        # cmd_vel_raw.linear.x 부호 기반 주행 방향
        self.direction = 1

        self.create_subscription(Float64MultiArray, "/ego_state", self.cb_ego, 10)
        self.create_subscription(Bool, "/mapping_cmd", self.cb_mapping_cmd, 10)
        self.create_subscription(Twist, "/cmd_vel_raw", self.cb_cmd_vel, 10)

        self.create_timer(0.2, self.record_loop)  # 5Hz 기록
        self.get_logger().info(
            f"🗺️ Mapping node 시작 | auto_remodel={self.auto_remodel_enabled} | "
            f"spacing={self.remodel_spacing}m epsilon={self.remodel_epsilon}m smooth={self.remodel_smooth_iter}"
        )

    def cb_cmd_vel(self, msg: Twist):
        """[보조] cmd_vel_raw.linear.x 부호. 자율주행 중 매핑 시에만 발행됨.
        리모컨 매핑 때는 발행되지 않으므로 cb_ego의 속도 부호가 주 신호다."""
        v = msg.linear.x
        if v > 0.05:
            self.direction = 1
        elif v < -0.05:
            self.direction = -1
        # 0 부근이면 이전 방향 유지

    def cb_ego(self, msg: Float64MultiArray):
        d = msg.data
        if len(d) >= 7:
            self.lat = d[0]
            self.lon = d[1]
            self.heading = d[4]
            self.speed = d[5]
            self.steer = d[6]
            # 🔧 [v2] 주행방향을 ego 속도(엔코더 부호 반영) 기준으로 직접 추적.
            #   리모컨 수동 매핑 때는 /cmd_vel_raw가 없으므로 이 신호가 주(主) 신호.
            #   gps_imu가 발행하는 d[5]는 후진 시 음수가 된다(current_speed_dir 반영).
            if self.speed > 0.05:
                self.direction = 1
            elif self.speed < -0.05:
                self.direction = -1
            # 0 부근이면 이전 방향 유지(정지·미세속도 구간)
        if len(d) >= 9:
            self.pitch = d[7]
            self.terrain = d[8]

    def cb_mapping_cmd(self, msg: Bool):
        if msg.data and not self.is_mapping:
            self.start_mapping()
        elif not msg.data and self.is_mapping:
            self.stop_mapping()

    def start_mapping(self):
        fname = os.path.join(
            self.data_dir,
            f"route_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
        )
        self.current_csv_path = fname
        self.csv_file = open(fname, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "latitude", "longitude", "heading", "speed", "steer",
            "direction", "pitch", "terrain",
        ])
        self.is_mapping = True
        self.get_logger().info(f"🗺️ 매핑 시작: {fname}")

    def stop_mapping(self):
        self.is_mapping = False
        saved_path = self.current_csv_path

        if self.csv_file:
            self.csv_file.flush()
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None

        self.get_logger().info(f"✅ 매핑 종료 및 원본 저장 완료: {saved_path}")

        if self.auto_remodel_enabled and saved_path:
            self.run_auto_remodel(saved_path)

    def run_auto_remodel(self, input_csv: str):
        if remodel_route is None:
            self.get_logger().error(
                "❌ route_remodeler.py를 import하지 못했습니다. "
                "white/route_remodeler.py 파일이 같은 패키지에 있는지 확인하세요."
            )
            return

        # 🔧 BUG9: 리모델링을 백그라운드 스레드에서 실행 → ROS2 콜백 블로킹 방지
        def _worker():
            try:
                self.get_logger().info("🔧 매핑 경로 자동 리모델링 시작 (백그라운드)...")
                out_csv = remodel_route(
                    input_csv=input_csv,
                    output_csv=None,
                    spacing=self.remodel_spacing,
                    epsilon=self.remodel_epsilon,
                    smooth_iter=self.remodel_smooth_iter,
                )
                self.get_logger().info(f"✅ 리모델링 경로 저장 완료: {out_csv}")
                self.get_logger().info("🚗 자율주행에서는 *_remodeled.csv 파일을 선택하면 됩니다.")
            except Exception as e:
                self.get_logger().error(f"❌ 자동 리모델링 실패: {e}")
                self.get_logger().warn("원본 매핑 CSV는 정상 저장되어 있으므로 원본으로 주행하거나 수동 리모델링하세요.")

        self._remodel_thread = threading.Thread(target=_worker, daemon=True)
        self._remodel_thread.start()

    def record_loop(self):
        if self.is_mapping and self.lat is not None and self.csv_writer is not None:
            self.csv_writer.writerow([
                f"{self.lat:.8f}",
                f"{self.lon:.8f}",
                f"{self.heading:.2f}",
                f"{self.speed:.4f}",
                f"{self.steer:.2f}",
                str(self.direction),
                f"{self.pitch:.2f}",
                f"{self.terrain:.1f}",
            ])
            # 🔧 BUG8: 매 10행(2초)마다 flush → 크래시 시 데이터 유실 방지
            self._flush_counter += 1
            if self._flush_counter >= 10:
                self.csv_file.flush()
                self._flush_counter = 0


def main(args=None):
    rclpy.init(args=args)
    node = MappingNode()
    try:
        rclpy.spin(node)
    finally:
        if node.is_mapping:
            node.stop_mapping()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()