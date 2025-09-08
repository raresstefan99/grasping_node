from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_grasping_node = get_package_share_directory('grasping_node')

    # Percorso al file YAML con i parametri
    param_file = os.path.join(pkg_grasping_node, 'config', 'config.yaml')

    return LaunchDescription([
        # (Opzionale) Dichiarazione di argomenti da terminale, se vuoi sovrascrivere valori
        DeclareLaunchArgument(
            'params_file',
            default_value=param_file,
            description='Path to the parameter file'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),

        Node(
            package='grasping_node',
            executable='auto_grasp_bag',
            name='grasp_node',
            parameters=[param_file],
            output='screen'
        )
    ])