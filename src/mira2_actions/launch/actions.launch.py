import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    tree_xml = os.path.join(
        get_package_share_directory('mira2_actions'),
        'config',
        'sauvc_mission.xml'
    )

    return LaunchDescription([
        Node(
            package='mira2_actions',
            executable='mira2_actions_exe',
            name='mira2_actions_node',
            output='screen',
            parameters=[{'tree_xml': tree_xml}]
        )
    ])