import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_yaml = os.path.join(
        get_package_share_directory('resple'),
        'config',
        'config_airy96.yaml')

    return launch.LaunchDescription([
        # 1) TF statiques indispensables
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_to_base_link',
            arguments=['0','0','0','0','0','0','odom','base_link']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_base_link_to_rslidar',
            arguments=['0','0','0','0','0','0','base_link','rslidar']
        ),

        # 2) Noeud RESPLE
        Node(
            package='resple',
            executable='RESPLE',
            name='RESPLE',
            emulate_tty=True,
            output='screen',
            parameters=[config_yaml],
            arguments=['--ros-args','--log-level','warn']
        ),

        # 3) Noeud Mapping
        Node(
            package='resple',
            executable='Mapping',
            name='Mapping',
            emulate_tty=True,
            output='screen',
            parameters=[config_yaml],
            arguments=['--ros-args','--log-level','debug']
        ),
    ])
