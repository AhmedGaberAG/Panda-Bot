from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Launch arguments
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.065",
        description="Wheel radius of the mobile DDR robot",
    )

    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.37",
        description="Distance between the two drive wheels",
    )

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="true",
        description="Use the custom differential-drive controller",
    )

    # Launch configurations
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Standard DiffDriveController
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panda_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        condition=UnlessCondition(use_simple_controller),
        output="screen",
    )

    # Custom differential-drive controller
    simple_controller_group = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[

            # Wheel velocity controller
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),

            # Differential-drive kinematics node
            Node(
                package="panda_controller",
                executable="simple_controller.py",
                parameters=[
                    {
                        "wheel_radius": wheel_radius,
                        "wheel_separation": wheel_separation,
                    }
                ],
                output="screen",
            ),
        ],
    )

    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        use_simple_controller_arg,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        simple_controller_group,
    ])