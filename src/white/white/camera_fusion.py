#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
camera_fusion.py ― driving 노드에 얹는 카메라 융합 애드온 [v1.0]
─────────────────────────────────────────────────────────────────
설계 원칙 (인수인계 문서와 동일):
  · ego_state(gps_imu)는 성역 — 위치·헤딩 융합을 건드리지 않는다.
  · 카메라는 driving의 "CTE 가중융합(PID 채널)"과 "속도 캡"으로만 개입한다.
  · Pure Pursuit(조향의 94%)는 계속 GPS 경로를 조준한다. 카메라는 발밑 이탈량만.
  · cam_enable=False(기본) = 섀도 모드: 융합·캡을 전혀 적용하지 않고
    /fusion_debug 로깅만 수행 → "듣기만 하는 주행"으로 검증 데이터 수집.

driving_fusion.py 쪽 훅(4개):
  1) __init__ 끝:      self.camfu = CameraFusion(self)
  2) CTE 확정 직후:    cte, cam_w = camfu.fuse_cte(cte, is_rev, dr_active)
  3) 경로접선 확보 후:  camfu.update_route_ref(path_psi_deg, near_peak_deg, is_straight)
  4) DR 감속 직후:     v, reason, hold = camfu.apply_speed_caps(v, reason, dr_active)
  5) 루프 끝:          camfu.publish_debug(loop_dt)

발행:
  /fusion_debug     Float32MultiArray[18]  ← rosbag_extractor.py 의 FUSION_DEBUG_FIELDS 와 1:1
  /lane_heading_ref Float64MultiArray[3]   [절대헤딩 추정°, 품질 0..1, 발행시각]
                                            (gps_imu_fusion 이 DR 중 헤딩보정 참조로 사용)
구독:
  /lane_metrics   Float32MultiArray[10] ← lane_camera_bridge.py 출력(미터 단위·게이트 통과 계측)
  /tl/state       String  "RED"/"RED_FAR"/"GREEN"/"UNKNOWN"  ← cam/test2_perception
  /stop_line_dist Float32 [m], 미검출 -1                      ← cam/test2_perception
"""

import math
import time

from std_msgs.msg import Float32MultiArray, Float64MultiArray, String, Float32

# /fusion_debug 인덱스 (rosbag_extractor.py FUSION_DEBUG_FIELDS 와 반드시 일치)
FD_CAM_OK        = 0   # 이번 루프에 카메라 계측 사용가능(신선+게이트) 1/0
FD_CAM_CTE_REAR  = 1   # 뒷차축 투영 카메라 CTE [m, 좌+]
FD_CAM_CTE_NEAR  = 2   # 근점(투영 전) CTE [m]
FD_THETA_LANE    = 3   # 차선 대비 헤딩오차 [deg, CCW+ = 차량이 차선 기준 좌로 틀어짐]
FD_CAM_CONF      = 4   # bridge 유효 confidence(게이트 반영) 0..1
FD_CAM_AGE       = 5   # 계측 나이 [s]
FD_GPS_CTE       = 6   # 융합 전 GPS 경로 CTE [m]
FD_CTE_FUSED     = 7   # 융합 후 CTE [m] (PID 채널 실입력)
FD_W_APPLIED     = 8   # 실제 적용 가중치(슬루 후)
FD_W_TARGET      = 9   # 목표 가중치(게이트·모드 반영, 슬루 전)
FD_DR_ACTIVE     = 10  # DR 여부 1/0
FD_TL_CODE       = 11  # 신호등: -1 UNKNOWN / 0 GREEN / 1 RED_FAR / 2 RED
FD_STOP_DIST     = 12  # 정지선 거리 [m], 미검출 -1
FD_CAP_CODE      = 13  # 캡 사유: 0 없음 /1 TL_BRAKE /2 TL_HOLD /3 DR_MID /4 DR_LANE_LOST
FD_V_CAP         = 14  # 캡 적용 후 목표속도 [m/s] (캡 미적용 시 입력 그대로)
FD_HEAD_REF      = 15  # /lane_heading_ref 로 발행한 절대헤딩 추정 [deg] (미발행 시 999)
FD_HEAD_QUAL     = 16  # 그 품질 0..1 (미발행 0)
FD_LOOP_DT       = 17  # driving 루프 dt [s]

TL_CODE = {"UNKNOWN": -1.0, "GREEN": 0.0, "RED_FAR": 1.0, "RED": 2.0}


class CameraFusion:
    def __init__(self, node):
        self.node = node
        n = node

        # ── 파라미터 (런치에서 주입 가능) ──────────────────────────────
        n.declare_parameter("cam_enable",           False)  # False=섀도(로깅만)
        n.declare_parameter("cam_w_max",            0.40)   # RTK 정상 시 카메라 최대 가중치
        n.declare_parameter("cam_w_dr",             0.90)   # DR(GPS 두절) 시 목표 가중치
        n.declare_parameter("cam_w_rate",           1.5)    # w 슬루 [1/s] (급변 방지)
        n.declare_parameter("cam_conf_min",         0.35)   # 이 미만이면 w 목표 0
        n.declare_parameter("cam_metrics_timeout",  0.40)   # [s] 계측 신선도
        n.declare_parameter("cam_gps_diverge_m",    0.80)   # RTK 정상인데 |cam-gps| 초과 → 카메라 불신
        # 신호등
        n.declare_parameter("tl_enable",            True)
        n.declare_parameter("tl_decel",             1.0)    # 정지선 접근 감속도 [m/s²]
        n.declare_parameter("tl_stop_margin",       0.50)   # 정지선 이 앞에서 완전정지 [m]
        n.declare_parameter("tl_far_cap",           1.2)    # RED_FAR(멀리 빨간불) 순항 상한 [m/s]
        n.declare_parameter("tl_nodist_cap",        0.8)    # RED인데 정지선 미검출 시 서행 상한
        n.declare_parameter("tl_hold_s",            0.4)    # RED 판정 유지시간(깜빡임 필터)
        # DR 중속 (기존 dr_speed_factor 0.6 위에 절대 상한을 한 겹 더)
        n.declare_parameter("dr_speed_cap",         1.2)    # DR 중 절대 상한 [m/s] = "중속"
        n.declare_parameter("dr_lane_lost_cap",     0.5)    # DR + 차선까지 상실 시 상한
        # 헤딩 참조 발행 게이트 (gps_imu 쪽 적용 게이트와 별개·직렬)
        n.declare_parameter("head_ref_enable",      True)
        n.declare_parameter("head_ref_max_theta",   8.0)    # |θ_lane| 이내에서만 [deg]
        n.declare_parameter("head_ref_max_peak",    3.0)    # 지도 직선 판정: near_peak 이내 [deg]

        g = lambda k: n.get_parameter(k).value
        self.enable          = bool(g("cam_enable"))
        self.w_max           = float(g("cam_w_max"))
        self.w_dr            = float(g("cam_w_dr"))
        self.w_rate          = float(g("cam_w_rate"))
        self.conf_min        = float(g("cam_conf_min"))
        self.metrics_timeout = float(g("cam_metrics_timeout"))
        self.diverge_m       = float(g("cam_gps_diverge_m"))
        self.tl_enable       = bool(g("tl_enable"))
        self.tl_decel        = float(g("tl_decel"))
        self.tl_stop_margin  = float(g("tl_stop_margin"))
        self.tl_far_cap      = float(g("tl_far_cap"))
        self.tl_nodist_cap   = float(g("tl_nodist_cap"))
        self.tl_hold_s       = float(g("tl_hold_s"))
        self.dr_speed_cap    = float(g("dr_speed_cap"))
        self.dr_lane_lost_cap= float(g("dr_lane_lost_cap"))
        self.head_ref_enable = bool(g("head_ref_enable"))
        self.head_max_theta  = float(g("head_ref_max_theta"))
        self.head_max_peak   = float(g("head_ref_max_peak"))

        # ── 상태 ──────────────────────────────────────────────────────
        self.m_cte_rear = 0.0; self.m_cte_near = 0.0
        self.m_theta    = 0.0; self.m_conf     = 0.0
        self.m_flags    = 0.0; self.m_time     = 0.0
        self.tl_state   = "UNKNOWN"; self.tl_time = 0.0
        self.red_since  = None            # RED 최초 관측 시각 (hold 필터)
        self.stop_dist  = -1.0; self.stop_time = 0.0
        self.w_applied  = 0.0
        self._last_w_t  = time.time()
        # 이번 루프 디버그 스냅샷
        self._dbg = [0.0] * 18
        self._dbg[FD_HEAD_REF] = 999.0

        # ── 통신 ──────────────────────────────────────────────────────
        n.create_subscription(Float32MultiArray, "/lane_metrics",   self._cb_metrics, 10)
        n.create_subscription(String,            "/tl/state",       self._cb_tl,      10)
        n.create_subscription(Float32,           "/stop_line_dist", self._cb_stop,    10)
        self.pub_dbg  = n.create_publisher(Float32MultiArray, "/fusion_debug",     10)
        self.pub_href = n.create_publisher(Float64MultiArray, "/lane_heading_ref", 10)

        n.get_logger().info(
            f"📷 CameraFusion 로드 | enable={self.enable}"
            f"{'(실융합)' if self.enable else '(섀도-로깅만)'} "
            f"w_max={self.w_max} w_dr={self.w_dr} conf_min={self.conf_min} | "
            f"TL={'on' if self.tl_enable else 'off'} decel={self.tl_decel} "
            f"margin={self.tl_stop_margin}m | DR중속캡={self.dr_speed_cap}m/s")

    # ══════════════════════════════════════════════════════════════════
    # 콜백
    # ══════════════════════════════════════════════════════════════════
    def _cb_metrics(self, msg: Float32MultiArray):
        d = msg.data
        if len(d) < 7:
            return
        self.m_cte_rear = float(d[0])
        self.m_cte_near = float(d[1])
        self.m_theta    = float(d[2])          # deg, CCW+
        self.m_conf     = float(d[4])          # 게이트 반영 유효 conf
        self.m_flags    = float(d[6])
        self.m_time     = time.time()

    def _cb_tl(self, msg: String):
        s = msg.data.strip().upper()
        now = time.time()
        if s in ("RED", "RED_FAR"):
            if self.red_since is None:
                self.red_since = now
        else:
            self.red_since = None
        self.tl_state = s if s in TL_CODE else "UNKNOWN"
        self.tl_time  = now

    def _cb_stop(self, msg: Float32):
        self.stop_dist = float(msg.data)
        self.stop_time = time.time()

    # ══════════════════════════════════════════════════════════════════
    # 유틸
    # ══════════════════════════════════════════════════════════════════
    def _cam_fresh(self):
        return (time.time() - self.m_time) <= self.metrics_timeout

    def _red_confirmed(self):
        """RED/RED_FAR 가 tl_hold_s 이상 지속되어야 '확정'(깜빡임·오검출 필터)."""
        if self.red_since is None:
            return False
        if (time.time() - self.tl_time) > 1.0:      # 신호등 토픽 자체가 끊김 → 미확정
            return False
        return (time.time() - self.red_since) >= self.tl_hold_s

    # ══════════════════════════════════════════════════════════════════
    # 훅 2) CTE 가중융합 — PID 채널 입력을 만든다
    #      cte_used = w·cam_cte_rear + (1−w)·gps_cte
    # ══════════════════════════════════════════════════════════════════
    def fuse_cte(self, gps_cte: float, is_rev: bool, dr_active: bool):
        now  = time.time()
        age  = now - self.m_time if self.m_time > 0 else 999.0
        fresh = age <= self.metrics_timeout
        cam_ok = fresh and (self.m_conf >= self.conf_min) and not is_rev

        # RTK 정상인데 두 측정이 크게 어긋나면 → 카메라를 불신 (오검출/매핑-차선 불일치 방어)
        diverged = (not dr_active) and cam_ok and \
                   (abs(self.m_cte_rear - gps_cte) > self.diverge_m)
        if diverged:
            cam_ok = False

        # 목표 가중치
        if not self.enable or not cam_ok:
            w_target = 0.0
        elif dr_active:
            w_target = self.w_dr        # GPS 두절: 차선이 횡방향 전담
        else:
            w_target = self.w_max       # 평상시: 보조 (매핑≠차선중앙 가능성 고려해 보수적)

        # w 슬루 (급변 방지 — 차선 상실/복구 순간 조향 충격 차단)
        dt = max(1e-3, min(0.5, now - self._last_w_t))
        self._last_w_t = now
        max_dw = self.w_rate * dt
        dw = max(-max_dw, min(max_dw, w_target - self.w_applied))
        self.w_applied = max(0.0, min(1.0, self.w_applied + dw))

        cte_fused = self.w_applied * self.m_cte_rear + (1.0 - self.w_applied) * gps_cte \
                    if self.w_applied > 1e-4 else gps_cte

        # 디버그 스냅샷
        d = self._dbg
        d[FD_CAM_OK]       = 1.0 if cam_ok else 0.0
        d[FD_CAM_CTE_REAR] = self.m_cte_rear
        d[FD_CAM_CTE_NEAR] = self.m_cte_near
        d[FD_THETA_LANE]   = self.m_theta
        d[FD_CAM_CONF]     = self.m_conf
        d[FD_CAM_AGE]      = min(age, 99.0)
        d[FD_GPS_CTE]      = gps_cte
        d[FD_CTE_FUSED]    = cte_fused
        d[FD_W_APPLIED]    = self.w_applied
        d[FD_W_TARGET]     = w_target
        d[FD_DR_ACTIVE]    = 1.0 if dr_active else 0.0
        return cte_fused, self.w_applied

    # ══════════════════════════════════════════════════════════════════
    # 훅 3) 헤딩 참조 발행 — DR 중 gps_imu 의 자이로적분 drift 억제 참조
    #      절대헤딩 추정 = 지도접선(H_map) − θ_lane   (ψ_v = lane_abs − θ)
    #      게이트: 카메라 신선·conf ∧ |θ|작음 ∧ 지도 직선 ∧ driving의 직선 판정
    # ══════════════════════════════════════════════════════════════════
    def update_route_ref(self, path_psi_deg: float, near_peak_deg: float, is_straight: bool):
        d = self._dbg
        d[FD_HEAD_REF]  = 999.0
        d[FD_HEAD_QUAL] = 0.0
        if not self.head_ref_enable or not self._cam_fresh():
            return
        if self.m_conf < self.conf_min:
            return
        if abs(self.m_theta) > self.head_max_theta:
            return
        if near_peak_deg > self.head_max_peak or not is_straight:
            return
        head_est = path_psi_deg - self.m_theta       # [deg] 절대헤딩 추정
        while head_est > 180.0:  head_est -= 360.0
        while head_est < -180.0: head_est += 360.0
        quality = min(1.0, self.m_conf)
        msg = Float64MultiArray()
        msg.data = [float(head_est), float(quality), time.time()]
        self.pub_href.publish(msg)
        d[FD_HEAD_REF]  = head_est
        d[FD_HEAD_QUAL] = quality

    # ══════════════════════════════════════════════════════════════════
    # 훅 4) 속도 캡 — 기존 5중 캡 뒤의 6·7번째
    #      ① 신호등 RED: 정지선 제동곡선 v=√(2·a·(d−margin)), margin 내 완전정지(hold)
    #      ② DR 중속: 절대 상한 dr_speed_cap, 차선까지 상실 시 dr_lane_lost_cap
    #      hold=True → 호출측이 min_speed 하한(normal_floor)을 해제해야 함
    # ══════════════════════════════════════════════════════════════════
    def apply_speed_caps(self, v_target: float, reason: str, dr_active: bool):
        hold = False
        cap_code = 0.0
        v_in = v_target

        if self.enable:
            # ── ① 신호등 ──
            if self.tl_enable and self._red_confirmed():
                sd_fresh = (time.time() - self.stop_time) <= 0.6
                sd = self.stop_dist if (sd_fresh and self.stop_dist >= 0.0) else -1.0
                if sd >= 0.0:
                    if sd <= self.tl_stop_margin:
                        v_target = 0.0
                        reason   = "TL_HOLD"
                        hold     = True
                        cap_code = 2.0
                    else:
                        v_brk = math.sqrt(max(0.0, 2.0 * self.tl_decel *
                                              (sd - self.tl_stop_margin)))
                        if self.tl_state == "RED_FAR":
                            v_brk = max(v_brk, 0.0)
                            v_brk = min(max(v_brk, 0.3), max(self.tl_far_cap, 0.3)) \
                                    if sd > 6.0 else v_brk
                        if v_brk < v_target:
                            v_target = v_brk
                            reason   = "TL_BRAKE"
                            cap_code = 1.0
                            if v_target <= 0.05:
                                hold = True
                                reason = "TL_HOLD"
                                cap_code = 2.0
                else:
                    # 빨간불인데 정지선 미검출 → 서행 접근 (RED_FAR 는 순항 상한만)
                    cap = self.tl_far_cap if self.tl_state == "RED_FAR" else self.tl_nodist_cap
                    if cap < v_target:
                        v_target = cap
                        reason   = "TL_SLOW"
                        cap_code = 1.0

            # ── ② DR 중속 ──
            if dr_active:
                lane_ok = self._cam_fresh() and self.m_conf >= self.conf_min
                cap = self.dr_speed_cap if lane_ok else self.dr_lane_lost_cap
                if cap < v_target:
                    v_target = cap
                    reason   = "DR_MID" if lane_ok else "DR_LANE_LOST"
                    cap_code = 3.0 if lane_ok else 4.0

        d = self._dbg
        d[FD_TL_CODE]   = TL_CODE.get(self.tl_state, -1.0)
        d[FD_STOP_DIST] = self.stop_dist
        d[FD_CAP_CODE]  = cap_code
        d[FD_V_CAP]     = v_target if cap_code > 0 else v_in
        return v_target, reason, hold

    # ══════════════════════════════════════════════════════════════════
    # 훅 5) 루프당 1회 디버그 발행 (20Hz) — rosbag 검증의 1차 소스
    # ══════════════════════════════════════════════════════════════════
    def publish_debug(self, loop_dt: float):
        self._dbg[FD_LOOP_DT] = loop_dt
        msg = Float32MultiArray()
        msg.data = [float(x) for x in self._dbg]
        self.pub_dbg.publish(msg)