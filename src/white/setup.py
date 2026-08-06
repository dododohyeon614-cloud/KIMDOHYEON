from setuptools import setup
import os
from glob import glob

package_name = 'white'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            sorted(set(glob('launch/*.launch.py')) | set(glob('launch/*.py')))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='domi',
    maintainer_email='domi@todo.todo',
    description='GPS+IMU+Camera fusion autonomous driving (white)',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ── 실행파일 이름은 기존 그대로(prompt/one_launch 호환), 모듈만 개명 ──
            'iahrs          = white.iahrs:main',
            'motor          = white.motor:main',
            'gps_imu        = white.gps_imu_fusion:main',      # ← gps_imu.py 후속(카메라 DR 헤딩보정 수신)
            'mapping        = white.mapping:main',
            'driving        = white.driving_fusion:main',      # ← driving.py 후속(카메라 CTE융합+신호등캡)
            'prompt         = white.prompt:main',
            'sensor_monitor = white.sensor_monitor:main',
            # ── 신규: 카메라 차선 계측 브리지 (/lane/state → /lane_metrics) ──
            'lane_bridge    = white.lane_camera_bridge:main',
        ],
    },
)