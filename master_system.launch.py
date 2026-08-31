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
             name='lucid_apriltag_tf_publisher', output='screen'),
        Node(package='rviz2', executable='rviz2', name='global_rviz', output='screen',
             arguments=['-d', root + '/teleop_kuka_iiwa7/kuka-iiwa-fri-teleoperation/lbr_bringup/config/hardware.rviz'],
             condition=IfCondition(LaunchConfiguration('launch_rviz'))),
    ])
