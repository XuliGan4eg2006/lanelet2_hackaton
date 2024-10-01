from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare the launch argument
    osm_file_arg = DeclareLaunchArgument(
        'osm_file',
        default_value='map.osm',
        description='Path to the OSM file to load'
    )

    return LaunchDescription([
        osm_file_arg,
        Node(
            package='osm_cartography',
            executable='osm_cartography_node',
            name='osm_cartography_node',
            parameters=[{
                'osm_file': LaunchConfiguration('osm_file')
            }],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('osm_cartography'), 'config', 'osm_viz.rviz')]
        )
    ])