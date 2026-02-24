from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Robot namespace (empty by default)",
        ),
        DeclareLaunchArgument(
            "sim_reduction",
            default_value="2",
            description="By how much to downsample the cameras' output in simulation",
        ),
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="Run in Gazebo sim (enables sensors and uses sim time)",
        ),
    ]

    namespace = LaunchConfiguration("namespace")
    sim_reduction = LaunchConfiguration("sim_reduction")
    sim = LaunchConfiguration("sim")

    model_name = "scout_mini.xacro"
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("scout_description"), "urdf", model_name]
            ),
            " namespace:=",
            namespace,
            " sim_reduction:=",
            sim_reduction,
            " sim:=",
            sim,
        ]
    )
    robot_description = ParameterValue(robot_description_content, value_type=str)

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="scout_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": sim,
                "robot_description": robot_description,
                "frame_prefix": [namespace, "/"],
            }
        ],
    )

    return LaunchDescription(
        declared_args
        + [
            robot_state_publisher,
        ]
    )
