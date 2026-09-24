from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare launch arguments
    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='0',
        description='Control mode: 0=Direct passthrough, 1=PID, 2=PID with anti-windup, 3=CS controller'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('nereo_controller_node'), 'config', 'controller_params.yaml']
        ),
        description='Params YAML holding kp/ki/kd, anti_windup_gains, authority_cap and the cs_* gains (D-03)'
    )

    # Create node
    nereo_controller_node = Node(
        package='nereo_controller_node',
        executable='nereo_controller_node',
        name='nereo_controller_node',
        parameters=[
            LaunchConfiguration('params_file'),
            {'control_mode': LaunchConfiguration('control_mode')}
        ],
        output='screen'
    )

    return LaunchDescription([
        control_mode_arg,
        params_file_arg,
        nereo_controller_node
    ])
