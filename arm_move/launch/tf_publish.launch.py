from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
      Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.012', '-0.071', '0', '0', '0', '0', 'tool0', 'camera_link']
      ),
   ])
