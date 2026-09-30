import os 
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    panda_description = get_package_share_directory("panda_description")

    # Launch argument
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            panda_description,
            "urdf",
            "panda.urdf.xacro"
        ),
        description="Absolute path to robot Xacro file",
    )

    rviz_path = os.path.join(
        panda_description,
        "rviz",
        "panda.rviz",
    )

    # Robot description
    robot_description = ParameterValue(Command(["xacro ", 
                                       LaunchConfiguration("model")]), 
                                       value_type=str
                                    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description
        }]
    )

    # Joint state publisher gui 
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # Rviz2 
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            '-d', rviz_path
        ],
        output="screen"
    )

    return LaunchDescription([
        # Arguments
        model_arg,
        # Robot
        robot_state_publisher_node,
        # Joint
        joint_state_publisher_gui_node,
        # Rviz2
        rviz2_node
    ])