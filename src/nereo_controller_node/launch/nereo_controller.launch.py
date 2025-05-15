from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare launch arguments
    control_mode_arg = DeclareLaunchArgument(
        'control_mode',
        default_value='0',
        description='Control mode: 0=Direct passthrough, 1=PID, 2=PID with anti-windup, 3=CS controller'
    )

    kp_arg = DeclareLaunchArgument(
        'kp',
        default_value='[0.1, 0.1, 0.1, 0.1]',
        description='Proportional gains for z, roll, pitch, yaw controllers'
    )

    ki_arg = DeclareLaunchArgument(
        'ki',
        default_value='[0.01, 0.01, 0.01, 0.01]',
        description='Integral gains for z, roll, pitch, yaw controllers'
    )

    kd_arg = DeclareLaunchArgument(
        'kd',
        default_value='[0.05, 0.05, 0.05, 0.05]',
        description='Derivative gains for z, roll, pitch, yaw controllers'
    )

    cs_kx0_arg = DeclareLaunchArgument(
        'cs_kx0',
        default_value='[198.0952, 468.0585]',
        description='CS controller kx0 gains'
    )

    cs_kx1_arg = DeclareLaunchArgument(
        'cs_kx1',
        default_value='[3.8191, 6.0003]',
        description='CS controller kx1 gains'
    )

    cs_kx2_arg = DeclareLaunchArgument(
        'cs_kx2',
        default_value='[5.2577, 11.8206]',
        description='CS controller kx2 gains'
    )

    cs_ki0_arg = DeclareLaunchArgument(
        'cs_ki0',
        default_value='-359.2481',
        description='CS controller ki0 gain'
    )

    cs_ki1_arg = DeclareLaunchArgument(
        'cs_ki1',
        default_value='0.0',
        description='CS controller ki1 gain'
    )

    cs_ki2_arg = DeclareLaunchArgument(
        'cs_ki2',
        default_value='-16.7389',
        description='CS controller ki2 gain'
    )

    cs_heave_min_arg = DeclareLaunchArgument(
        'cs_heave_min',
        default_value='-60.0',
        description='CS controller heave minimum limit'
    )

    cs_heave_max_arg = DeclareLaunchArgument(
        'cs_heave_max',
        default_value='80.0',
        description='CS controller heave maximum limit'
    )

    cs_angle_min_arg = DeclareLaunchArgument(
        'cs_angle_min',
        default_value='-30.0',
        description='CS controller angle minimum limit'
    )

    cs_angle_max_arg = DeclareLaunchArgument(
        'cs_angle_max',
        default_value='30.0',
        description='CS controller angle maximum limit'
    )

    # Create node
    nereo_controller_node = Node(
        package='nereo_controller_node',
        executable='nereo_controller_node',
        name='nereo_controller_node',
        parameters=[{
            'control_mode': LaunchConfiguration('control_mode'),
            'kp': LaunchConfiguration('kp'),
            'ki': LaunchConfiguration('ki'),
            'kd': LaunchConfiguration('kd'),
            'cs_kx0': LaunchConfiguration('cs_kx0'),
            'cs_kx1': LaunchConfiguration('cs_kx1'),
            'cs_kx2': LaunchConfiguration('cs_kx2'),
            'cs_ki0': LaunchConfiguration('cs_ki0'),
            'cs_ki1': LaunchConfiguration('cs_ki1'),
            'cs_ki2': LaunchConfiguration('cs_ki2'),
            'cs_heave_min': LaunchConfiguration('cs_heave_min'),
            'cs_heave_max': LaunchConfiguration('cs_heave_max'),
            'cs_angle_min': LaunchConfiguration('cs_angle_min'),
            'cs_angle_max': LaunchConfiguration('cs_angle_max')
        }],
        output='screen'
    )

    return LaunchDescription([
        control_mode_arg,
        kp_arg,
        ki_arg,
        kd_arg,
        cs_kx0_arg,
        cs_kx1_arg,
        cs_kx2_arg,
        cs_ki0_arg,
        cs_ki1_arg,
        cs_ki2_arg,
        cs_heave_min_arg,
        cs_heave_max_arg,
        cs_angle_min_arg,
        cs_angle_max_arg,
        nereo_controller_node
    ])