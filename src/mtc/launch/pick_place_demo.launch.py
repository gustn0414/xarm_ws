from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_dict()

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