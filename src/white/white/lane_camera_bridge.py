#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lane_camera_bridge.py ― 카메라 차선 계측 브리지 [v1.0]
─────────────────────────────────────────────────────────────────
역할: cam/test2_perception 의 /lane/state(BEV 픽셀공간 다항식)를
      driving 이 그대로 섞을 수 있는 "미터 단위·뒷차축 기준" 계측으로 변환.
      + 물리 게이트(폭·점프·평행성)로 오검출을 실시간 자가검증.

핵심 기하 (인수인계):
  · BEV: x=우측+, y=아래+, 차량은 하단 중앙에서 위를 봄. 차선 fit: x = a·y² + b·y + c
  · 근점(y_near = bev_h×bottom_ratio) 에서 차선중앙 x_c 를 구하고
      cte_near[m, 좌+] = (x_c − bev_w/2) × px2m
      (차선중앙이 화면 오른쪽 = 차가 중앙의 왼쪽 = 좌이탈 + → GPS CTE 부호와 동일)
  · θ_lane[CCW+, 차량이 차선 기준 좌로 틀어짐+] = −atan2(Δx, Δy)  (근점↔전방점)
      + cam_yaw_offset_deg (카메라 장착 요 오차 보정 — 섀도 주행으로 캘리브)
  · 근점은 뒷차축보다 d_near 앞이므로 뒷차축 투영:
      cte_rear = cte_near + d_near × sin(θ_lane)
      d_near = bev_bottom_ahead_rear_m + (bev_h − y_near)×px2m
      ※ bev_bottom_ahead_rear_m: 뒷차축→BEV 최하단 실거리.
        기존 캘리브 0.55m 는 카메라가 뒷차축 0.61m 앞이라는 신규 실측과 상충 가능
        → README 의 줄자 재실측법으로 갱신 권장 (카메라 높이 0.38m 는 BEV
        호모그래피 캘리브에 이미 흡수되어 여기선 사용하지 않음 — 기록용 메타)

발행: /lane_metrics Float32MultiArray[10]
  [0] cte_rear_m   [1] cte_near_m   [2] theta_lane_deg  [3] curvature_1pm
  [4] conf_eff(게이트 반영, 치명 게이트 실패 시 0)      [5] lane_width_m
  [6] flags        [7] d_near_m     [8] conf_raw        [9] seq

flags 비트: 1=차선없음 2=폭 비정상 4=CTE 점프 8=conf 미달 16=단독차선(반폭 추정)
"""

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

F_NO_LANE   = 1
F_WIDTH_BAD = 2
F_JUMP      = 4
F_CONF_LOW  = 8
F_SINGLE    = 16


class LaneCameraBridge(Node):
    def __init__(self):
        super().__init__("lane_camera_bridge")

        # ── 파라미터 (perception/judgment 공유값과 반드시 일치) ──────────
        self.declare_parameter("pixel_to_meter_bev",      0.0065)  # [m/px] BEV 캘리브 실측
        self.declare_parameter("bev_w",                   640)
        self.declare_parameter("bev_h",                   480)
        self.declare_parameter("bottom_ratio",            0.92)    # 근점(측정) 행
        self.declare_parameter("lookahead_ratio",         0.45)    # θ 산출용 전방 행
        self.declare_parameter("lane_width_m",            3.0)     # 실제 차선폭
        # 장착 기하 (뒷차축 기준) — 카메라 0.61m 전방 / 지면 0.38m (기록용)
        self.declare_parameter("bev_bottom_ahead_rear_m", 0.55)    # ★ 재실측 권장(README)
        self.declare_parameter("cam_ahead_rear_m",        0.61)    # 메타(계산 미사용)
        self.declare_parameter("cam_height_m",            0.38)    # 메타(계산 미사용)
        self.declare_parameter("cam_yaw_offset_deg",      0.0)     # ★ 섀도 주행으로 캘리브
        # 자가검증 게이트
        self.declare_parameter("conf_min",                0.15)
        self.declare_parameter("width_min_m",             1.8)
        self.declare_parameter("width_max_m",             4.5)
        self.declare_parameter("jump_max_m",              0.35)    # 프레임간 CTE 최대 점프
        self.declare_parameter("stale_warn_s",            2.0)

        g = lambda k: self.get_parameter(k).value
        self.px2m       = float(g("pixel_to_meter_bev"))
        self.bev_w      = int(g("bev_w"))
        self.bev_h      = int(g("bev_h"))
        self.y_near     = float(self.bev_h) * float(g("bottom_ratio"))
        self.y_look     = float(self.bev_h) * float(g("lookahead_ratio"))
        self.lane_w_m   = float(g("lane_width_m"))
        self.d_bev0     = float(g("bev_bottom_ahead_rear_m"))
        self.yaw_off    = float(g("cam_yaw_offset_deg"))
        self.conf_min   = float(g("conf_min"))
        self.w_min      = float(g("width_min_m"))
        self.w_max      = float(g("width_max_m"))
        self.jump_max   = float(g("jump_max_m"))
        self.stale_warn = float(g("stale_warn_s"))

        # 근점의 뒷차축 전방 거리 [m]
        self.d_near = self.d_bev0 + (self.bev_h - self.y_near) * self.px2m
        self.half_w_px_default = (self.lane_w_m * 0.5) / self.px2m

        # 상태
        self.prev_cte_near = None
        self.prev_t        = 0.0
        self.last_rx       = 0.0
        self.seq           = 0
        self.rx_times      = []

        self.pub = self.create_publisher(Float32MultiArray, "/lane_metrics", 10)
        self.create_subscription(Float32MultiArray, "/lane/state", self.cb_lane, 10)
        self.create_timer(1.0, self._status_tick)

        self.get_logger().info(
            f"🌉 lane_camera_bridge | px2m={self.px2m} y_near={self.y_near:.0f}px "
            f"→ 근점 d_near={self.d_near:.2f}m(뒷차축 전방) | "
            f"yaw_off={self.yaw_off:+.2f}° | 게이트: 폭{self.w_min}~{self.w_max}m "
            f"점프≤{self.jump_max}m conf≥{self.conf_min}")

    # ──────────────────────────────────────────────────────────────────
    @staticmethod
    def _poly_x(fit, y):
        a, b, c = fit
        return a * y * y + b * y + c

    def cb_lane(self, msg: Float32MultiArray):
        d = list(msg.data)
        if len(d) < 8:
            return
        now = time.time()
        self.last_rx = now
        self.rx_times.append(now)
        self.rx_times = [t for t in self.rx_times if now - t <= 3.0]
        self.seq += 1

        aL, bL, cL, aR, bR, cR = d[0], d[1], d[2], d[3], d[4], d[5]
        conf_raw = float(d[6])
        half_w_px_meas = float(d[7])

        left_ok  = abs(aL) + abs(bL) + abs(cL) > 1e-9
        right_ok = abs(aR) + abs(bR) + abs(cR) > 1e-9
        flags = 0
        width_m = -1.0

        if not left_ok and not right_ok:
            flags |= F_NO_LANE
            self._publish(0.0, 0.0, 0.0, 0.0, 0.0, width_m, flags, conf_raw)
            return

        hw_px = half_w_px_meas if half_w_px_meas > 1.0 else self.half_w_px_default

        if left_ok and right_ok:
            xL_n = self._poly_x((aL, bL, cL), self.y_near)
            xR_n = self._poly_x((aR, bR, cR), self.y_near)
            xL_l = self._poly_x((aL, bL, cL), self.y_look)
            xR_l = self._poly_x((aR, bR, cR), self.y_look)
            xc_n = 0.5 * (xL_n + xR_n)
            xc_l = 0.5 * (xL_l + xR_l)
            width_m = (xR_n - xL_n) * self.px2m
            curv = (abs(aL) + abs(aR)) * 0.5 * 2.0 / self.px2m   # κ≈2a/px2m [1/m]
            if not (self.w_min <= width_m <= self.w_max):
                flags |= F_WIDTH_BAD
        else:
            flags |= F_SINGLE
            fit = (aL, bL, cL) if left_ok else (aR, bR, cR)
            sgn = +1.0 if left_ok else -1.0     # 좌차선 → 중앙은 +x(우), 우차선 → −x
            xc_n = self._poly_x(fit, self.y_near) + sgn * hw_px
            xc_l = self._poly_x(fit, self.y_look) + sgn * hw_px
            curv = abs(fit[0]) * 2.0 / self.px2m

        # ── 계측 산출 ──
        cte_near = (xc_n - self.bev_w * 0.5) * self.px2m           # [m, 좌+]
        dx_px = xc_l - xc_n
        dy_px = max(1.0, self.y_near - self.y_look)
        theta = -math.atan2(dx_px, dy_px) + math.radians(self.yaw_off)  # [rad, CCW+]
        cte_rear = cte_near + self.d_near * math.sin(theta)

        # ── 점프 게이트 ──
        if (self.prev_cte_near is not None
                and (now - self.prev_t) < 0.30
                and abs(cte_near - self.prev_cte_near) > self.jump_max):
            flags |= F_JUMP
        self.prev_cte_near = cte_near
        self.prev_t = now

        if conf_raw < self.conf_min:
            flags |= F_CONF_LOW

        # 치명 게이트(폭·점프·conf) 실패 → conf_eff=0 → 상류(camfu)가 자동으로 w=0
        fatal = flags & (F_WIDTH_BAD | F_JUMP | F_CONF_LOW)
        conf_eff = 0.0 if fatal else conf_raw

        self._publish(cte_rear, cte_near, math.degrees(theta), curv,
                      conf_eff, width_m, flags, conf_raw)

    def _publish(self, cte_rear, cte_near, theta_deg, curv,
                 conf_eff, width_m, flags, conf_raw):
        m = Float32MultiArray()
        m.data = [float(cte_rear), float(cte_near), float(theta_deg), float(curv),
                  float(conf_eff), float(width_m), float(flags),
                  float(self.d_near), float(conf_raw), float(self.seq)]
        self.pub.publish(m)

    def _status_tick(self):
        now = time.time()
        if self.last_rx == 0.0:
            self.get_logger().warn("⏳ /lane/state 수신 대기 중 (perception 미기동?)")
            return
        if now - self.last_rx > self.stale_warn:
            self.get_logger().warn(
                f"⛔ /lane/state {now - self.last_rx:.1f}s 두절 — 카메라/perception 확인!")


def main(args=None):
    rclpy.init(args=args)
    node = LaneCameraBridge()
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