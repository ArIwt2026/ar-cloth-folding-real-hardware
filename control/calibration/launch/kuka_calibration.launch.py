from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("job_config", default_value=""),
        DeclareLaunchArgument("job_name", default_value="kuka_fixed_lucid_apriltag"),
        DeclareLaunchArgument("autostart", default_value="false"),
        DeclareLaunchArgument("autostart_solve_after_collection", default_value="true"),
        DeclareLaunchArgument("enable_preview", default_value="true"),
        Node(
            package="franka_handeye_calibration",
            executable="coordinator_node",
            output="screen",
            parameters=[{
                "job_config_path": LaunchConfiguration("job_config"),
                "default_job_config_path": LaunchConfiguration("job_config"),
                "autostart_job_config_path": LaunchConfiguration("job_config"),
                "job_name": LaunchConfiguration("job_name"),
                "autostart": ParameterValue(LaunchConfiguration("autostart"), value_type=bool),
                "autostart_solve_after_collection": ParameterValue(
                    LaunchConfiguration("autostart_solve_after_collection"), value_type=bool
                ),
                "enable_preview": ParameterValue(LaunchConfiguration("enable_preview"), value_type=bool),
                "use_fake_hardware": False,
            }],
        ),
    ])
