from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("nereo_controller_node"), "config", "controller_params.yaml"]
        ),
        description="Params YAML holding kp/ki/kd, anti_windup_gains, authority_cap "
                    "and the cs_* gains (D-03)",
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nereo_controller_node"), "launch", "nereo_controller.launch.py"]
            )
        ),
        launch_arguments={"params_file": LaunchConfiguration("params_file")}.items(),
    )

    tuner_gui_node = Node(
        package="nereo_controller_node",
        executable="pid_tuner_gui.py",
        name="nereo_pid_tuner_gui",
        output="screen",
        parameters=[{
            "target_node": "/nereo_controller_node",
            "params_file": LaunchConfiguration("params_file"),
        }],
    )

    return LaunchDescription([params_file_arg, controller_launch, tuner_gui_node])
