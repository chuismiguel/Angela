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
        Node(
            package='angela_joint_control',
            executable='joint_control_node',
            name='joint_control_node',
            output='screen',
        ),
        # Node(
        #     package='camera_ros',
        #     executable='camera_node',
        #     name='camera_node',
        #     output='screen',
        #     parameters=[
        #         {'camera': '/base/soc/i2c0mux/i2c@1/ov5647@36'}
        #     ],
        # ),
    ])

if __name__ == '__main__':
    generate_launch_description()
