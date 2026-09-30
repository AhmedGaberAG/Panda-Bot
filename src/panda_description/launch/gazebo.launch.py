import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Package paths
    panda_description = get_package_share_directory("panda_description")

    gz_sim = os.path.join(
        get_package_share_directory("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py",
    )

    bridge_config = os.path.join(
        panda_description,
        "config",
        "gz_bridge.yaml",
    )

    # Launch arguments
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            panda_description,
            "urdf",
            "panda.urdf.xacro",
        ),
        description="Absolute path to robot Xacro file",
    )

    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="empty",
        description="Gazebo world name",
    )

    # Gazebo world
    world_path = PathJoinSubstitution([
        panda_description,
        "worlds",
        PythonExpression([
            "'",
            LaunchConfiguration("world_name"),
            "' + '.world'",
        ]),
    ])

    # Gazebo resource path
    model_path = str(Path(panda_description).parent.resolve())
    model_path += pathsep + os.path.join(
        panda_description,
        "models",
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=model_path,
    )

    # Robot description
    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
        ]),
        value_type=str,
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
                "robot_description": robot_description,
                "use_sim_time": True,
            }],
    )

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim),
        launch_arguments={
            "gz_args": PythonExpression([
                "'",
                world_path,
                " -v 4 -r'",
            ]),
        }.items(),
    )

    # Spawn robot into Gazebo
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic","robot_description",
            "-name", "panda",
        ],
    )

    # Gazebo <-> ROS 2 bridge
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{
                "config_file": bridge_config,
            }]
    )

    # Launch description
    return LaunchDescription([
        # Arguments
        model_arg,
        world_name_arg,
        # Environment
        gazebo_resource_path,
        # Robot
        robot_state_publisher_node,
        # Simulation
        gazebo,
        gz_spawn_entity,
        # ROS 2 interfaces
        gz_ros2_bridge
    ])