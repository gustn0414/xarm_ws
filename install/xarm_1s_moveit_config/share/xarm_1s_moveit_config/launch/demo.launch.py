import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "xarm_1s_moveit_config"
    urdf_path = os.path.join(
        get_package_share_directory("xarm_1s_description"),
        "urdf", "xarm_1s.urdf.xacro",
    )

    # use_mock:=true  → 시뮬레이션 / use_mock:=false → 실제 로봇
    declare_use_mock = DeclareLaunchArgument(
        "use_mock",
        default_value="true",
        description="true: 시뮬레이션(mock), false: 실제 로봇(HID)"
    )
    use_mock = LaunchConfiguration("use_mock")

    moveit_config = (
        MoveItConfigsBuilder("xarm_1s_moveit_config", package_name=package_name)
        .robot_description(file_path=urdf_path, mappings={"use_mock": use_mock})
        .robot_description_semantic(file_path="config/xarm_1s.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "stomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        "ros2_controllers.yaml",
    )

    rviz_config = os.path.join(
        get_package_share_directory(package_name),
        "rviz", "moveit.rviz",
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
            # 그리퍼 stall 허용 시간 확보를 위한 실행 timeout 설정
            {"trajectory_execution.allowed_execution_duration_scaling": 5.0},
            {"trajectory_execution.allowed_goal_duration_margin": 3.0},
            # start state 검증 비활성화 → 0-duration 궤적 2포인트 문제 해결
            {"trajectory_execution.allowed_start_tolerance": 0.0},
        ],
    )

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

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            ros2_controllers_path,
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ]
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "xarm_1s_arm_controller",
            "--controller-manager",
            "/controller_manager",
        ]
    )

    hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "hand_controller",
            "--controller-manager",
            "/controller_manager",
        ]
    )

    return LaunchDescription([
        declare_use_mock,
        run_move_group_node,
        run_rviz_node,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        hand_controller_spawner,
    ])
