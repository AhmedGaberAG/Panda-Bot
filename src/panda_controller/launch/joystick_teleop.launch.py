import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    package_share = get_package_share_directory("panda_controller")

    twist_mux_launch = os.path.join(
        get_package_share_directory("twist_mux"),
        "launch",
        "twist_mux_launch.py",
    )

    joy_config = os.path.join(
        package_share,
        "config",
        "joy_config.yaml",
    )

    joy_teleop_config = os.path.join(
        package_share,
        "config",
        "joy_teleop.yaml",
    )

    twist_mux_topics_config = os.path.join(
        package_share,
        "config",
        "twist_mux_topics.yaml",
    )

    twist_mux_locks_config = os.path.join(
        package_share,
        "config",
        "twist_mux_locks.yaml",
    )

    twist_mux_joy_config = os.path.join(
        package_share,
        "config",
        "twist_mux_joy.yaml",
    )

    # Joystick driver
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[joy_config],
        output="screen",
    )

    # Joystick -> Twist
    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[joy_teleop_config],
        output="screen",
    )

    # Twist multiplexer
    twist_mux_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(twist_mux_launch),
        launch_arguments={
            "cmd_vel_out": "panda_controller/cmd_vel_unstamped",
            "config_topics": twist_mux_topics_config,
            "config_locks": twist_mux_locks_config,
            "config_joy": twist_mux_joy_config,
        }.items(),
    )

    twist_relay_node = Node(
        package="panda_controller",
        executable="twist_relay.py",
        name="twist_relay"
    )

    return LaunchDescription([
        joy_node,
        joy_teleop,
        twist_mux_node,
        twist_relay_node
    ])