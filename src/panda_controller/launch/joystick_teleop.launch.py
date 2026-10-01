import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_share = get_package_share_directory("panda_controller")

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

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[joy_config],
        output="screen",
    )

    joy_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[joy_teleop_config],
        output="screen",
    )

    return LaunchDescription([
        joy_node,
        joy_teleop,
    ])