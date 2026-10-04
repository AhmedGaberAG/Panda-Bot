from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    laser_filter_config = PathJoinSubstitution([
        get_package_share_directory("panda_utils"),
        "config",
        "laser_filter.yaml",
    ])
        
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time", 
        default_value="True",
    )

    scan_self_filter = Node(
        package="panda_utils",
        executable="scan_self_filter.py",
        name="scan_self_filter",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    laser_filter = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        name="laser_filter",
        parameters=[laser_filter_config, {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[
            ("scan", "scan_self_filtered"),
            ("scan_filtered", "scan_filtered"),
        ],
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        scan_self_filter,
        laser_filter,
    ])

