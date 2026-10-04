from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    laser_filter_config = PathJoinSubstitution([
        get_package_share_directory("panda_utils"),
        "config",
        "laser_filter.yaml",
    ])

    scan_self_filter = Node(
        package="panda_utils",
        executable="scan_self_filter.py",
        name="scan_self_filter",
        output="screen",
    )

    laser_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="laser_filter",
        parameters=[laser_filter_config],
        remappings=[
            ("scan", "scan_self_filtered"),
            ("scan_filtered", "scan_filtered"),
        ],
        output="screen",
    )

    return LaunchDescription([
        scan_self_filter,
        laser_filter,
    ])

