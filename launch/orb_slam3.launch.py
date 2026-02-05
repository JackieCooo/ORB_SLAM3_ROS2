from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_dir = PathJoinSubstitution([FindPackageShare('orb_slam3'), 'config'])
    voc_file = PathJoinSubstitution([FindPackageShare('orb_slam3'), 'vocabulary', 'ORBvoc.txt'])

    return LaunchDescription([
        # RGB image topic
        DeclareLaunchArgument(
            'rgb_topic', default_value='/camera/rgb/image_raw'
        ),
        # Depth image topic
        DeclareLaunchArgument(
            'depth_topic', default_value='/camera/depth_registered/image_raw'
        ),
        # ORB_SLAM3 vocabulary file
        DeclareLaunchArgument(
            'voc_file', default_value=voc_file
        ),
        # ORB_SLAM3 setting
        DeclareLaunchArgument(
            'setting', default_value='RealSense_D435i'
        ),

        # SLAM node
        Node(
            package='orb_slam3',
            executable='rgbd_node',
            name='orb_slam3_rgbd',
            parameters=[{
                'rgb_topic': LaunchConfiguration('rgb_topic'),
                'depth_topic': LaunchConfiguration('depth_topic'),
                'voc_file': LaunchConfiguration('voc_file'),
                'setting_file': PathJoinSubstitution([config_dir, LaunchConfiguration('setting') + TextSubstitution('.yaml')]),
            }]
        )
    ])
