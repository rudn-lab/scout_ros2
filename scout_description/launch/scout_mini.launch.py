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
            "use_sim_time",
            default_value=False,
            description="Robot namespace (empty by default)",
        ),
    ]

    model_name = "scout_mini.xacro"
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("scout_description"), "urdf", model_name]
            ),
            " namespace:=",
            LaunchConfiguration("namespace"),
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
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "robot_description": robot_description,
            }
        ],
    )

    return LaunchDescription(
        declared_args
        + [
            robot_state_publisher,
        ]
    )
