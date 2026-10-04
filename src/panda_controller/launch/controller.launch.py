from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )

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

    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005",
        description="Wheel radius error of the mobile DDR robot",
    )

    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.02",
        description="Distance error between the two drive wheels",
    )

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="true",
        description="Use the custom differential-drive controller",
    )

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    wheel_radius_error = LaunchConfiguration("wheel_radius_error")
    wheel_separation_error = LaunchConfiguration("wheel_separation_error")
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
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
        ],
    )

    # Noise controller node
    noisy_controller_node = Node(
        package="panda_controller",
        executable="noisy_controller.py",
        parameters=[
            {
                "wheel_radius": PythonExpression([
                    wheel_radius,
                    " + ",
                    wheel_radius_error,
                ]),
                "wheel_separation": PythonExpression([
                    wheel_separation,
                    " + ",
                    wheel_separation_error,
                ]),
                "use_sim_time": use_sim_time,
            }
        ],
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        wheel_radius_error_arg,
        wheel_separation_error_arg,
        use_simple_controller_arg,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        simple_controller_group,
        noisy_controller_node
    ])