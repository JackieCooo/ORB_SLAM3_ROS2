from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    launch_dir = PathJoinSubstitution([FindPackageShare('orb_slam3'), 'launch'])

    return LaunchDescription([
        # Enable Foxglove bridge server
        DeclareLaunchArgument(
            'foxglove_bridge', default_value='False'
        ),
        # Image sequence directory
        DeclareLaunchArgument(
            'sequence_dir', default_value='fr1_xyz'
        ),
        # Association file
        DeclareLaunchArgument(
            'association_file', default_value='fr1_xyz.txt'
        ),

        # SLAM node
        Node(
            package='orb_slam3',
            executable='rgbd_tum_example',
            name='rgbd_tum',
            parameters=[{
                'voc_file': PathJoinSubstitution([FindPackageShare('orb_slam3'), 'vocabulary', 'ORBvoc.txt']),
                'setting_file': PathJoinSubstitution([FindPackageShare('orb_slam3'), 'config', 'TUM.yaml']),
                'sequence_dir': LaunchConfiguration('sequence_dir'),
                'association_file': LaunchConfiguration('association_file')
            }]
        ),

        # Foxglove node
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'foxglove_bridge.launch.py']),
            condition=IfCondition(LaunchConfiguration('foxglove_bridge'))
        ),
    ])
