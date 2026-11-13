from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_bno055',
            executable='bno055_node',
            name='bno055'
        )
    ])
