import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='angela_movement',
            executable='motor_controller',
            name='motor_controller',
            output='screen',
        ),
        Node(
            package='angela_ultrasensor_control',
            executable='ultrasensor',
            name='ultrasensor',
            output='screen',
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
