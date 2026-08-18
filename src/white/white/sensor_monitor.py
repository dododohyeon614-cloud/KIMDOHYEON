#!/usr/bin/env python3
"""
sensor_monitor.py - 센서 상태 통합 모니터링 노드 [300틱 속도표시 정합]
ROS2 터미널에서 GPS / IMU / 엔코더 / 주행 상태를 실시간으로 확인합니다.
실행: ros2 run white sensor_monitor
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32, String
from sensor_msgs.msg import NavSatFix, Imu
import math
import time

class SensorMonitorNode(Node):
    def __init__(self):
        super().__init__("sensor_monitor_node")

        # ── 상태 변수 ──────────────────────────────────
        self.gps_status_raw   = -1       # NavSatFix status.status (드라이버 원본)
        # [GQ] gps_imu 가 매긴 품질 등급 0~4. status 만으로는 RTK Fixed/Float 가
        #      구분되지 않아(둘 다 2) covariance 를 같이 본 값이다. -1 = 아직 미수신.
        self.gps_quality      = -1
        self.gps_h_std        = float('nan')   # /fix 에서 직접 계산한 수평 std[m]
        self.gps_last_time    = None
        self.gps_lat          = 0.0
        self.gps_lon          = 0.0
        self.gps_recv_count   = 0        # 수신 카운터 (Hz 계산용)
        self.gps_hz_buf       = []       # 최근 수신 시각 버퍼

        self.imu_roll         = 0.0
        self.imu_pitch        = 0.0
        self.imu_yaw          = 0.0
        self.imu_last_time    = None
        self.imu_hz_buf       = []

        self.encoder_val      = 0
        self.encoder_speed_ms = 0.0
        self.encoder_last_time= None
        self.encoder_hz_buf   = []

        self.ego_heading      = 0.0
        self.ego_speed        = 0.0
        self.ego_fused_lat    = 0.0
        self.ego_fused_lon    = 0.0
        self.ego_last_time    = None

        self.gps_status_str   = "수신 대기 중..."

        # ── 구독 ───────────────────────────────────────
        self.create_subscription(NavSatFix,         "/fix",        self.cb_fix,     10)
        self.create_subscription(Imu,               "/imu/data",   self.cb_imu,     10)
        self.create_subscription(Int32,             "/encoder",    self.cb_encoder, 10)
        self.create_subscription(Float64MultiArray, "/ego_state",  self.cb_ego,     10)
        self.create_subscription(String,            "/gps_status", self.cb_gps_status, 10)
        self.create_subscription(Int32,             "/gps_quality", self.cb_gps_quality, 10)

        # 1초마다 터미널에 상태 출력
        self.create_timer(1.0, self.print_status)

        self.get_logger().info("🖥️  센서 모니터 시작 (1초 주기 상태 출력)")

    # ── 콜백 ───────────────────────────────────────────
    def cb_fix(self, msg):
        now = time.time()
        self.gps_status_raw = msg.status.status
        self.gps_lat        = msg.latitude
        self.gps_lon        = msg.longitude
        # [GQ] 수평 std[m] — gps_imu 가 등급을 가를 때 쓰는 바로 그 값을 같이 보여준다
        try:
            c0 = float(msg.position_covariance[0])
            self.gps_h_std = math.sqrt(c0) if (math.isfinite(c0) and c0 > 0.0) else float('nan')
        except (AttributeError, IndexError, TypeError, ValueError):
            self.gps_h_std = float('nan')
        self.gps_last_time  = now
        self.gps_hz_buf.append(now)
        self.gps_hz_buf     = [t for t in self.gps_hz_buf if now - t <= 5.0]

    def cb_imu(self, msg):
        now = time.time()
        # 쿼터니언 → 오일러 변환
        x, y, z, w = (msg.orientation.x, msg.orientation.y,
                      msg.orientation.z, msg.orientation.w)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        self.imu_roll  = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        self.imu_pitch = math.degrees(math.asin(sinp))

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        self.imu_yaw   = math.degrees(math.atan2(siny_cosp, cosy_cosp))

        self.imu_last_time = now
        self.imu_hz_buf.append(now)
        self.imu_hz_buf = [t for t in self.imu_hz_buf if now - t <= 5.0]

    def cb_encoder(self, msg):
        now = time.time()
        self.encoder_val = msg.data
        WHEEL_CIRCUMFERENCE = 0.27 * math.pi
        TICKS_PER_REV = 300.0   # 실제 확인된 바퀴 1회전 전체 틱 수
        DT = 0.01              # Arduino v6 속도 PID / d_val 갱신 기준 10ms
        self.encoder_speed_ms = (float(msg.data) / TICKS_PER_REV) * WHEEL_CIRCUMFERENCE / DT
        self.encoder_last_time = now
        self.encoder_hz_buf.append(now)
        self.encoder_hz_buf = [t for t in self.encoder_hz_buf if now - t <= 5.0]

    def cb_ego(self, msg):
        if len(msg.data) >= 7:
            self.ego_fused_lat = msg.data[0]
            self.ego_fused_lon = msg.data[1]
            self.ego_heading   = msg.data[4]
            self.ego_speed     = msg.data[5]
            self.ego_last_time = time.time()

    def cb_gps_status(self, msg):
        self.gps_status_str = msg.data

    def cb_gps_quality(self, msg):
        self.gps_quality = int(msg.data)

    # ── 유틸 ───────────────────────────────────────────
    def calc_hz(self, buf):
        if len(buf) < 2:
            return 0.0
        return (len(buf) - 1) / (buf[-1] - buf[0]) if (buf[-1] - buf[0]) > 0 else 0.0

    def age_str(self, last_time):
        if last_time is None:
            return "수신 없음"
        age = time.time() - last_time
        if age > 10.0:
            return f"⛔ {age:.0f}초 전 (단절)"
        if age > 2.0:
            return f"⚠️  {age:.1f}초 전"
        return f"✅ {age*1000:.0f}ms 전"

    def gps_quality_label(self, q):
        """[GQ] gps_imu 가 매긴 5단계 등급 라벨.

        기존 4단계 라벨(NavSatFix.status 그대로)은 ★status=2 를 전부
        'RTK FIX 오차 ~2cm' 라고 표시하는 문제★ 가 있었다. status 는 정확도가
        아니라 '보정을 어디서 받았나'를 뜻하는 칸이라, 정수 모호도를 못 푼
        RTK Float(오차 30cm~2m)도 똑같이 2 로 들어온다. 실측 로스백에서
        전체 fix 의 4.3% 가 그런 Float 였다.
        """
        labels = {
            0: "❌ NO FIX     (위성 신호 없음)",
            1: "🔴 GPS 단독   (보정 없음, 오차 ~2-5m)",
            2: "🟠 SBAS/DGPS  (위성 보정, 오차 ~1m)",
            3: "🟡 RTK FLOAT  (실수해, 오차 ~0.3-2m)",
            4: "🟢 RTK FIXED  (고정해, 오차 ~2cm)",
        }
        return labels.get(q, "❓ 등급 미수신 (gps_imu 노드 확인)")

    def gps_status_label(self, status):
        """드라이버 원본 status — 참고용으로만 표시한다(정확도 지표가 아님)."""
        labels = {
            -1: "NO_FIX(-1)  보정 없음",
             0: "FIX(0)      보정 없음",
             1: "SBAS_FIX(1) 위성 기반 보정",
             2: "GBAS_FIX(2) 지상 기반 보정 ← Fixed/Float 구분 안 됨",
        }
        return labels.get(status, f"UNKNOWN ({status})")

    # ── 상태 출력 (1초 주기) ────────────────────────────
    def print_status(self):
        now = time.time()
        sep = "═" * 50

        print(f"\n{sep}")
        print(f"  🖥️  센서 상태 모니터  [{time.strftime('%H:%M:%S')}]")
        print(sep)

        # ── GPS ─────────────────────────────────────────
        gps_age = (now - self.gps_last_time) if self.gps_last_time else None
        gps_hz  = self.calc_hz(self.gps_hz_buf)
        print(f"  📡 [GPS / RTK]")
        print(f"     등급    : {self.gps_quality_label(self.gps_quality)}")
        if math.isfinite(self.gps_h_std):
            print(f"     수평std : {self.gps_h_std:.3f} m   "
                  f"(원본 status: {self.gps_status_label(self.gps_status_raw)})")
        else:
            print(f"     원본    : status={self.gps_status_label(self.gps_status_raw)}")
        print(f"     수신    : {self.age_str(self.gps_last_time)}  ({gps_hz:.1f} Hz)")
        if self.gps_last_time:
            print(f"     좌표    : {self.gps_lat:.8f}, {self.gps_lon:.8f}")
        if gps_age and gps_age > 2.0:
            print(f"     ⚠️  GPS 수신 지연 {gps_age:.1f}초 — 위치 정확도 저하!")
        # [GQ] 경고 기준을 '등급 4(RTK Fixed) 미만'으로 바꿨다. 기존 기준
        #      (status < 2)은 RTK Float 를 정상으로 통과시켜서 경고가 안 떴다.
        if self.gps_last_time and 0 <= self.gps_quality < 4:
            print(f"     ⚠️  RTK FIXED 아님 — 위치 오차가 큽니다(주행 정확도 저하).")

        print()

        # ── IMU ─────────────────────────────────────────
        imu_hz = self.calc_hz(self.imu_hz_buf)
        print(f"  🧭 [IMU]")
        print(f"     수신    : {self.age_str(self.imu_last_time)}  ({imu_hz:.1f} Hz)")
        if self.imu_last_time:
            print(f"     Roll    : {self.imu_roll:+.2f}°   Pitch: {self.imu_pitch:+.2f}°   Yaw: {self.imu_yaw:+.2f}°")
        if self.imu_last_time is None or (now - self.imu_last_time) > 1.0:
            print(f"     ⛔ IMU 데이터 없음 — /dev/ttyUSB0 연결 확인 필요!")

        print()

        # ── 엔코더 ──────────────────────────────────────
        enc_hz = self.calc_hz(self.encoder_hz_buf)
        print(f"  🔢 [엔코더 / 속도]")
        print(f"     수신    : {self.age_str(self.encoder_last_time)}  ({enc_hz:.1f} Hz)")
        if self.encoder_last_time:
            direction = "전진 ▶" if self.encoder_speed_ms >= 0 else "후진 ◀"
            print(f"     틱값    : {self.encoder_val}  →  {abs(self.encoder_speed_ms):.3f} m/s  [{direction}]")
        if self.encoder_last_time is None or (now - self.encoder_last_time) > 1.0:
            print(f"     ⛔ 엔코더 없음 — Arduino 연결 확인 필요!")

        print()

        # ── 융합 상태 (ego_state) ────────────────────────
        print(f"  🚗 [융합 상태 (ego_state)]")
        print(f"     수신    : {self.age_str(self.ego_last_time)}")
        if self.ego_last_time:
            print(f"     헤딩    : {self.ego_heading:.1f}°")
            print(f"     속도    : {self.ego_speed:.3f} m/s")
            print(f"     융합위치: {self.ego_fused_lat:.8f}, {self.ego_fused_lon:.8f}")
        else:
            print(f"     ⚠️  ego_state 없음 — gps_imu 노드 헤딩 미확정 상태일 수 있음")

        print(sep)


def main(args=None):
    rclpy.init(args=args)
    node = SensorMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()