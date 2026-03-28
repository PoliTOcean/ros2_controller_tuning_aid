from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nereo_controller_node"), "launch", "nereo_controller.launch.py"]
            )
        )
    )

    tuner_gui_node = Node(
        package="nereo_controller_node",
        executable="pid_tuner_gui.py",
        name="nereo_pid_tuner_gui",
        output="screen",
        parameters=[{"target_node": "/nereo_controller_node"}],
    )

    return LaunchDescription([controller_launch, tuner_gui_node])
