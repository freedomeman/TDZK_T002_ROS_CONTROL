from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_share = FindPackageShare("t002_description")
    controller_share = FindPackageShare("t002_controller")

    xacro_file = LaunchConfiguration("xacro_file")
    controller_config = LaunchConfiguration("controller_config")

    robot_description = Command([
        FindExecutable(name="xacro"), " ", xacro_file
    ])

    robot_description_param = {
        "robot_description": ParameterValue(robot_description, value_type=str),
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description_param],
        output="screen",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description_param, controller_config],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    t002_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["t002_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    torque_control_spawner = Node(
        package="head_solver",
        executable="torque_control_node",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "xacro_file",
            default_value=PathJoinSubstitution(
                [description_share, "xacro", "T002_description.urdf.xacro"]),
            description="Path to the robot URDF/xacro file.",
        ),
        DeclareLaunchArgument(
            "controller_config",
            default_value=PathJoinSubstitution(
                [controller_share, "config", "controller.yaml"]),
            description="Controller YAML config.",
        ),
        robot_state_publisher,
        ros2_control_node,
        TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=4.0, actions=[t002_controller_spawner]),
        TimerAction(period=5.0, actions=[torque_control_spawner]),
    ])
