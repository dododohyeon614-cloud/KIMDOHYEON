#!/usr/bin/env python3
"""
one_launch.py ― GPS+IMU+카메라 통합 자율주행 launch [카메라 융합판 v1.0]
─────────────────────────────────────────────────────────────────────
기존(white) 대비 변경:
  · driving  → white.driving_fusion  (카메라 CTE 가중융합 + 신호등/DR 속도캡)
  · gps_imu  → white.gps_imu_fusion  (DR 중 카메라 절대헤딩 참조 보정)
  · [신규] use_camera=true 시 카메라 그룹 기동:
      usb_cam → cam/test2_perception → white/lane_bridge(/lane_metrics)
  · cam 패키지의 judgment / driver / test_motor_controller 는 '의도적으로' 미기동
      - /cmd_vel_raw 이중발행 · /control_state 권한충돌 · 아두이노 포트/프로토콜
        (57600·rad×50 vs 115200·"C,틱,도") 충돌 때문. 제어·아두이노는 white 전담.

모드:
  · cam_enable:=false (기본)  → 섀도 모드: 융합 미적용, /fusion_debug 로깅만
                                 (검증용 로스백 수집 = "듣기만 하는 주행")
  · cam_enable:=true          → 실융합: CTE 가중융합 + 신호등 정지 + DR 중속 활성

실행 예:
  ros2 launch white one_launch.py                          # GPS주행 + 카메라 섀도
  ros2 launch white one_launch.py cam_enable:=true         # 융합 활성
  ros2 launch white one_launch.py use_camera:=false        # 순수 GPS(기존과 동일)
"""

import serial.tools.list_ports

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def find_usb_port(candidates, device_name, used_ports=None):
    """여러 VID/PID 후보 중 현재 연결된 USB 포트를 찾는다. (기존과 동일)"""
    if used_ports is None:
        used_ports = set()
    ports = sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)
    for port in ports:
        if port.device in used_ports:
            continue
        for vid, pid in candidates:
            if port.vid == vid and port.pid == pid:
                used_ports.add(port.device)
                desc = port.description or ""
                print(f"✅ [{device_name}] 감지 성공: {port.device} {desc}")
                return port.device
    print(f"⚠️ [경고] {device_name} 장치를 찾을 수 없습니다! (연결 없이 디버깅 진행)")
    return "/dev/tty_NOT_FOUND"


def generate_launch_description():
    package_name = 'white'

    use_monitor      = LaunchConfiguration('use_monitor')
    use_camera       = LaunchConfiguration('use_camera')
    cam_enable       = LaunchConfiguration('cam_enable')
    gps_port_arg     = LaunchConfiguration('gps_port')
    arduino_port_arg = LaunchConfiguration('arduino_port')
    imu_port_arg     = LaunchConfiguration('imu_port')
    camera_device    = LaunchConfiguration('camera_device')
    camera_info_url  = LaunchConfiguration('camera_info_url')
    used_ports = set()

    print("\n=====================================================")
    print(" 🔍 USB 포트 자동 탐색을 시작합니다...")
    gps_port = find_usb_port([
        (0x1546, 0x01A9),   # u-blox 9 계열
        (0x1546, 0x01A8),   # u-blox 8 계열 / 일부 수신기
    ], "U-Blox/SMC GPS", used_ports)
    arduino_port = find_usb_port([
        (0x2341, 0x0042), (0x2341, 0x0010),
        (0x2A03, 0x0042), (0x1A86, 0x7523),
    ], "Arduino Mega", used_ports)
    imu_port = find_usb_port([
        (0x10C4, 0xEA60),
    ], "IMU Sensor", used_ports)
    print("=====================================================\n")

    args = [
        DeclareLaunchArgument('use_monitor', default_value='true',
                              description='sensor_monitor 노드 실행 여부'),
        DeclareLaunchArgument('gps_port',     default_value=gps_port),
        DeclareLaunchArgument('arduino_port', default_value=arduino_port),
        DeclareLaunchArgument('imu_port',     default_value=imu_port),
        # ── 카메라 융합 ──
        DeclareLaunchArgument('use_camera', default_value='true',
                              description='카메라 그룹(usb_cam+perception+bridge) 기동'),
        DeclareLaunchArgument('cam_enable', default_value='false',
                              description='false=섀도(로깅만) / true=실융합(CTE·신호등·DR중속)'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video2'),
        DeclareLaunchArgument('camera_info_url',
                              default_value='file:///home/ros2/camera_ws/calibrationdata/usb_cam_calibration.yaml'),
        # 공유 캘리브 (perception / bridge 가 반드시 같은 값을 봐야 함)
        DeclareLaunchArgument('lane_width_m',            default_value='3.0'),
        DeclareLaunchArgument('pixel_to_meter_bev',      default_value='0.0065'),
        DeclareLaunchArgument('bev_bottom_ahead_rear_m', default_value='0.55',
                              description='뒷차축→BEV최하단 실거리[m] ★줄자 재실측 권장(README)'),
        DeclareLaunchArgument('cam_yaw_offset_deg',      default_value='0.0',
                              description='카메라 장착 요 오차[°] ★섀도 로스백으로 캘리브'),
        DeclareLaunchArgument('dr_speed_cap',            default_value='1.2',
                              description='GPS두절(DR) 중속 절대상한[m/s]'),
    ]

    f = lambda cfg: ParameterValue(cfg, value_type=float)
    b = lambda cfg: ParameterValue(cfg, value_type=bool)

    # ══════════════════ white 핵심 (기존 구성 유지) ══════════════════
    gps = Node(package='nmea_navsat_driver', executable='nmea_serial_driver',
               name='nmea_serial_driver', output='screen',
               parameters=[{'port': gps_port_arg, 'baud': 115200}])

    iahrs = Node(package=package_name, executable='iahrs', name='iahrs_node',
                 output='screen',
                 parameters=[{'port': imu_port_arg, 'baud': 115200, 'send_tf': True}])

    motor = Node(package=package_name, executable='motor', name='motor_node',
                 output='screen',
                 parameters=[{'port': arduino_port_arg, 'baud': 115200}])

    gps_imu = Node(package=package_name, executable='gps_imu', name='gps_imu_node',
                   output='screen')

    mapping = Node(package=package_name, executable='mapping', name='mapping_node',
                   output='screen')

    driving = Node(package=package_name, executable='driving', name='driving_node',
                   output='screen',
                   parameters=[{
                       'cam_enable':   b(cam_enable),
                       'dr_speed_cap': f(LaunchConfiguration('dr_speed_cap')),
                   }])

    monitor = Node(package=package_name, executable='sensor_monitor',
                   name='sensor_monitor_node', output='screen',
                   condition=IfCondition(use_monitor))

    # ══════════════════ 카메라 그룹 (use_camera:=true) ═══════════════
    usb_cam = Node(
        package='usb_cam', executable='usb_cam_node_exe', name='usb_cam',
        output='screen', condition=IfCondition(use_camera),
        parameters=[{
            'video_device': camera_device,
            'framerate': 30.0,
            'image_width': 1920, 'image_height': 1080,
            'pixel_format': 'uyvy',
            'camera_name': 'narrow_stereo',
            'io_method': 'mmap',
            'camera_info_url': camera_info_url,
            'brightness': 0, 'contrast': 128, 'saturation': 60,
            'sharpness': 64, 'gain': 10,
            'auto_exposure': False, 'exposure': 120,
        }])

    # usb_cam 기동 2초 후 v4l2 강제 적용 (기존 cam 런치와 동일 동작)
    v4l2_force = TimerAction(
        period=2.0, condition=IfCondition(use_camera),
        actions=[ExecuteProcess(
            cmd=['bash', '-c',
                 'v4l2-ctl -d "$1" --set-ctrl=auto_exposure=1 && '
                 'v4l2-ctl -d "$1" --set-ctrl=exposure_time_absolute=120 && '
                 'v4l2-ctl -d "$1" --set-ctrl=gain=10 && '
                 'v4l2-ctl -d "$1" --set-ctrl=saturation=60 && '
                 'echo "[v4l2-ctl] camera controls applied"',
                 '_', camera_device],
            output='screen')])

    perception = Node(
        package='cam', executable='test2_perception', name='perception',
        output='screen', condition=IfCondition(use_camera),
        parameters=[{
            'lane_width_m':       f(LaunchConfiguration('lane_width_m')),
            'pixel_to_meter_bev': f(LaunchConfiguration('pixel_to_meter_bev')),
            'show_window':        False,   # 주행 중 GUI 끔(부하·headless)
        }])

    lane_bridge = Node(
        package=package_name, executable='lane_bridge', name='lane_camera_bridge',
        output='screen', condition=IfCondition(use_camera),
        parameters=[{
            'pixel_to_meter_bev':      f(LaunchConfiguration('pixel_to_meter_bev')),
            'lane_width_m':            f(LaunchConfiguration('lane_width_m')),
            'bev_bottom_ahead_rear_m': f(LaunchConfiguration('bev_bottom_ahead_rear_m')),
            'cam_yaw_offset_deg':      f(LaunchConfiguration('cam_yaw_offset_deg')),
        }])

    # ※ cam 패키지의 judgment / test_motor_controller / driver 는 여기서 절대
    #   기동하지 않는다 (명령·권한·아두이노 프로토콜 3중 충돌 — 헤더 참조).

    return LaunchDescription(args + [
        gps, iahrs, motor, gps_imu, mapping, driving, monitor,
        usb_cam, v4l2_force, perception, lane_bridge,
    ])