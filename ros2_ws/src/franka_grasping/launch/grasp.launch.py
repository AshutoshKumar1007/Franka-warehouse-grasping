from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('franka_grasping'),
        'config',
        'grasp.yaml'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=config,
            description='Path to the config file'
        ),
        
        Node(
            package='franka_grasping',
            executable='grasp_node',
            name='grasp_node',
            output='screen',
            parameters=[LaunchConfiguration('config')]
        )
    ])