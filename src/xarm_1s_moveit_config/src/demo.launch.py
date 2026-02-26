import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("xarm_1s_moveit_config", package_name="xarm_1s_custom_moveit_config")
        .robot_description(file_path="config/xarm_1s.urdf.xacro")
        .robot_description_semantic(file_path="config/xarm_1s.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_base = os.path.join(moveit_config.package_path, "config")
    rviz_config = os.path.join(rviz_base, "moveit.rviz")
    run_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )

    # mtc_node = Node(
    #     package="mtc_tutorial",
    #     executable="mtc_node",
    #     output="screen",
    #     parameters=[moveit_config.to_dict()],
    # )

    return LaunchDescription([
        run_move_group_node,
        run_rviz_node,
        robot_state_publisher,
        joint_state_publisher,
        # mtc_node,
    ])