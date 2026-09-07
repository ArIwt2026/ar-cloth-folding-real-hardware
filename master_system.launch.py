from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    root = '/home/iwtros/Documents/ar'
    kuka_launch = root + '/teleop_kuka_iiwa7/kuka-iiwa-fri-teleoperation/lbr_bringup/launch/fri_monitor_rviz.launch.py'
    static_launch = root + '/static_publishers.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('launch_rviz', default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(kuka_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(static_launch)),
        Node(package='franka_handeye_calibration', executable='lucid_apriltag_tf',
             name='lucid_apriltag_tf_publisher', output='screen', parameters=[{
                 'image_topic': '/lucid/triton/image_color',
                 'camera_info_topic': '/lucid/triton/camera_info',
                 'camera_frame': 'lucid_triton_color_optical_frame',
                 'tag_frame': 'lucid_triton_tag24',
             }]),
        Node(package='franka_handeye_calibration', executable='lucid_apriltag_tf',
             name='panda_d455_apriltag_tf_publisher', output='screen', parameters=[{
                 'image_topic': '/panda/d455/color/image_raw',
                 'camera_info_topic': '/panda/d455/color/camera_info',
                 'camera_frame': 'panda_d455_color_optical_frame',
                 'tag_frame': 'panda_d455_tag24',
             }]),
        Node(package='franka_handeye_calibration', executable='lucid_apriltag_tf',
             name='kuka_d455_apriltag_tf_publisher', output='screen', parameters=[{
                 'image_topic': '/iiwa7/d455/color/image_raw',
                 'camera_info_topic': '/iiwa7/d455/color/camera_info',
                 'camera_frame': 'kuka_d455_color_optical_frame',
                 'tag_frame': 'kuka_d455_tag24',
             }]),
        Node(package='button_bridge', executable='button_bridge',
             name='button_bridge', output='screen'),
        Node(package='button_bridge', executable='button_bridge_switcher',
             name='panda_controller_switcher', output='screen'),
        Node(package='rviz2', executable='rviz2', name='global_rviz', output='screen',
             arguments=['-d', root + '/teleop_kuka_iiwa7/kuka-iiwa-fri-teleoperation/lbr_bringup/config/hardware.rviz'],
             condition=IfCondition(LaunchConfiguration('launch_rviz'))),
    ])
