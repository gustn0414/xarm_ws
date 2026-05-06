"""
MoveIt Task Constructor 기반 pick&place 데모 launch.

먼저 demo.launch.py 가 실행 중이어야 한다 (move_group + ros2_control + 컨트롤러).

사용 예:
    # 터미널 1
    ros2 launch xarm_1s_bringup demo.launch.py use_mock:=true
    # 터미널 2
    ros2 launch xarm_1s_bringup pick_place_demo.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("xarm_1s", package_name="xarm_1s_moveit_config")
        .robot_description(file_path="config/xarm_1s.urdf.xacro")
        .robot_description_semantic(file_path="config/xarm_1s.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    ).to_dict()

    pick_place_demo = Node(
        package="mtc",
        executable="mtc_node",
        output="screen",
        parameters=[moveit_config],
    )

    return LaunchDescription([pick_place_demo])
