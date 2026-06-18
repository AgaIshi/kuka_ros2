from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # Forwarded to the underlying gazebo.launch.py
    ld.add_action(
        DeclareLaunchArgument(
            name="ctrl",
            default_value="cartesian_impedance_controller",
            description="Controller to spawn (forwarded to gazebo.launch.py).",
        )
    )

    # Which LBR arm to load (iiwa7 | iiwa14 | med7 | med14)
    ld.add_action(
        DeclareLaunchArgument(
            name="model",
            default_value="iiwa7",
            description="LBR model to load (forwarded to gazebo.launch.py).",
            choices=["iiwa7", "iiwa14", "med7", "med14"],
        )
    )

    # Set rviz:=false to run headless (e.g. if rviz2 is not installed)
    ld.add_action(
        DeclareLaunchArgument(
            name="rviz",
            default_value="true",
            description="Start RViz with the stiffness-ellipsoid display.",
        )
    )

    # Bring up Gazebo + the controller (reuses the existing, working launch)
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("kuka_control"), "launch", "gazebo.launch.py"]
                )
            ),
            launch_arguments={
                "ctrl": LaunchConfiguration("ctrl"),
                "model": LaunchConfiguration("model"),
            }.items(),
        )
    )

    # RViz, preconfigured with the stiffness-ellipsoid marker display
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("kuka_control"), "config", "stiffness_ellipsoid.rviz"]
    )
    ld.add_action(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
            condition=IfCondition(LaunchConfiguration("rviz")),
        )
    )

    return ld
