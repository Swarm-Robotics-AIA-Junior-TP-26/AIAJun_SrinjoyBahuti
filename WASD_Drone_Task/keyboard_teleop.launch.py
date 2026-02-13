#!/usr/bin/env python3


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate the launch description for keyboard teleoperation."""
    

    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14540@localhost:14557',
        description='FCU connection URL for PX4 SITL'
    )
    
    gcs_url_arg = DeclareLaunchArgument(
        'gcs_url',
        default_value='',
        description='GCS connection URL (empty to disable)'
    )
    
    target_system_id_arg = DeclareLaunchArgument(
        'target_system_id',
        default_value='1',
        description='MAVLink target system ID'
    )
    
    target_component_id_arg = DeclareLaunchArgument(
        'target_component_id',
        default_value='1',
        description='MAVLink target component ID'
    )
    

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': LaunchConfiguration('fcu_url'),
            'gcs_url': LaunchConfiguration('gcs_url'),
            'target_system_id': LaunchConfiguration('target_system_id'),
            'target_component_id': LaunchConfiguration('target_component_id'),
            'plugin_allowlist': [
                'sys_status',
                'sys_time', 
                'command',
                'setpoint_velocity',
                'setpoint_raw',
                'local_position',
                'global_position',
            ],
        }],
    )
    

    keyboard_input_node = Node(
        package='px4_keyboard_teleop',
        executable='keyboard_input_node',
        name='keyboard_input_node',
        output='screen',
        prefix='xterm -e',  

    )
    

    velocity_controller_node = Node(
        package='px4_keyboard_teleop',
        executable='velocity_controller_node',
        name='velocity_controller_node',
        output='screen',
    )
    

    launch_info = LogInfo(
        msg='\n' + '='*60 + '\n'
            'PX4 Keyboard Teleoperation\n'
            '='*60 + '\n'
            'Controls:\n'
            '  W/S - Forward/Backward\n'
            '  A/D - Left/Right\n'
            '  Z/X - Down/Up\n'
            '  Q/E - Yaw Left/Right\n'
            '  ESC - Emergency Stop & Disarm\n'
            '='*60 + '\n'
            'Note: Keyboard input node runs in separate xterm window.\n'
            'If xterm is not available, run keyboard_input_node manually.\n'
            '='*60
    )
    
    return LaunchDescription([

        fcu_url_arg,
        gcs_url_arg,
        target_system_id_arg,
        target_component_id_arg,
        

        launch_info,
        

        mavros_node,
        velocity_controller_node,
        keyboard_input_node,
    ])
