"""
[호환용 Wrapper]
이 launch 의 본체는 xarm_1s_bringup/launch/pick_place_demo.launch.py 로 이전되었습니다.

사용 예 (둘 다 동일하게 동작):
    ros2 launch xarm_1s_bringup pick_place_demo.launch.py
    ros2 launch mtc             pick_place_demo.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_launch = os.path.join(
        get_package_share_directory("xarm_1s_bringup"),
        "launch", "pick_place_demo.launch.py",
    )
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup_launch)),
    ])
