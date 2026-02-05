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

        # Realsense D435i node
        IncludeLaunchDescription(
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
        ),

        # SLAM node
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'orb_slam3.launch.py']),
            launch_arguments=[{
                'rgb_topic': '/camera/rgb/image_raw',
                'depth_topic': '/camera/depth_registered/image_raw',
                'setting': 'RealSense_D435i',
            }]
        ),
        
        # Foxglove node
        IncludeLaunchDescription(
            PathJoinSubstitution([launch_dir, 'foxglove_bridge.launch.py']),
            condition=IfCondition(LaunchConfiguration('foxglove_bridge'))
        ),
    ])
