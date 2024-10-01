from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('osm_cartography')

    # Declare the launch argument
    osm_file_arg = DeclareLaunchArgument(
        'osm_file',
        default_value='map.osm',
        description='Path to the OSM file to load'
    )

    # URDF file path
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    return LaunchDescription([
        osm_file_arg,

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}],
        ),

        # OSM Cartography node
        Node(
            package='osm_cartography',
            executable='osm_cartography_node',
            name='osm_cartography_node',
            parameters=[{
                'osm_file': LaunchConfiguration('osm_file')
            }],
            output='screen'
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'config', 'config.rviz')]
        )
    ])