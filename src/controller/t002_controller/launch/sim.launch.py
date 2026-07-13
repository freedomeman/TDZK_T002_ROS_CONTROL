"""Gazebo Ignition 仿真 launch (含 IMU)"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_description = FindPackageShare("t002_description")
    pkg_controller  = FindPackageShare("t002_controller")
    share_root      = PathJoinSubstitution([pkg_description, ".."])

    xacro_file        = LaunchConfiguration("xacro_file")
    controller_config = LaunchConfiguration("controller_config")
    world_file        = PathJoinSubstitution([pkg_description, "worlds", "empty_imu.sdf"])

    # GZ_SIM_RESOURCE_PATH
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[share_root, os.pathsep, EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value="")])

    # 机器人描述
    robot_description = Command([
        FindExecutable(name="xacro"), " ", xacro_file,
        " ", "controller_config_file:=", controller_config])

    # clock bridge
    clock_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="screen")

    # IMU bridge
    imu_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
        arguments=["/imu@sensor_msgs/msg/Imu[gz.msgs.IMU"], output="screen")

    # Gazebo
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([
        PathJoinSubstitution([get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"])]),
        launch_arguments={"gz_args": ["-r ", world_file]}.items())

    # Spawn
    spawn_entity = Node(package="ros_gz_sim", executable="create",
        arguments=["-topic", "robot_description", "-name", "T002_Robot", "-z", "0.3"], output="screen")

    # robot_state_publisher
    robot_state_publisher = Node(package="robot_state_publisher", executable="robot_state_publisher",
        parameters=[{"robot_description": ParameterValue(robot_description, value_type=str), "use_sim_time": True}],
        output="screen")

    # Controller spawners
    jb_spawner  = Node(package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager", "--controller-manager-timeout", "20"],
        output="screen")
    t002_spawner = Node(package="controller_manager", executable="spawner",
        arguments=["t002_controller", "-c", "/controller_manager", "--controller-manager-timeout", "20"],
        output="screen")

    # 脖子解算
    torque_node = Node(package="head_solver", executable="torque_control_node",
        output="screen", parameters=[{"use_sim_time": True}])

    return LaunchDescription([
        DeclareLaunchArgument("xacro_file",
            default_value=PathJoinSubstitution([pkg_description, "xacro", "T002_gazebo_des.urdf.xacro"])),
        DeclareLaunchArgument("controller_config",
            default_value=PathJoinSubstitution([pkg_controller, "config", "controller.yaml"])),
        gazebo_resource_path,
        robot_state_publisher,
        clock_bridge,
        imu_bridge,
        gazebo,
        TimerAction(period=3.0, actions=[spawn_entity]),
        TimerAction(period=5.0, actions=[jb_spawner]),
        TimerAction(period=7.0, actions=[t002_spawner]),
        TimerAction(period=8.0, actions=[torque_node]),
    ])
