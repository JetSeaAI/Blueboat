"""啟動 joy_linux + bb_joy teleop。

用法：
  ros2 launch bb_joy bb_joy.launch.py                             # Xbox，twist
  ros2 launch bb_joy bb_joy.launch.py pad:=ps5                    # PS5
  ros2 launch bb_joy bb_joy.launch.py output_mode:=rc_override    # MANUAL 直接覆寫 PWM
  ros2 launch bb_joy bb_joy.launch.py device:=/dev/input/js1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pad = LaunchConfiguration('pad')
    device = LaunchConfiguration('device')
    output_mode = LaunchConfiguration('output_mode')

    share = get_package_share_directory('bb_joy')

    params_file = PathJoinSubstitution([share, 'config', [pad, '.yaml']])

    return LaunchDescription([
        DeclareLaunchArgument('pad', default_value='xbox',
                              description='xbox 或 ps5，決定讀哪個 config'),
        DeclareLaunchArgument('device', default_value='/dev/input/js0',
                              description='手把裝置節點'),
        DeclareLaunchArgument('output_mode', default_value='twist',
                              description='twist 或 rc_override'),

        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            output='screen',
            parameters=[{
                'dev': device,
                'deadzone': 0.0,        # 死區交給 bb_joy 統一處理
                'autorepeat_rate': 20.0,
                'coalesce_interval': 0.01,
            }],
        ),

        Node(
            package='bb_joy',
            executable='joy_teleop_node',
            name='bb_joy_teleop',
            output='screen',
            parameters=[params_file, {'output_mode': output_mode}],
        ),
    ])
