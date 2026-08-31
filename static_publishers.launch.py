from launch import LaunchDescription
from launch_ros.actions import Node


def static(name, xyz, quat, parent, child):
    return Node(
        package='tf2_ros', executable='static_transform_publisher', name=name,
        output='screen', arguments=[
            '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
            '--qx', str(quat[0]), '--qy', str(quat[1]),
            '--qz', str(quat[2]), '--qw', str(quat[3]),
            '--frame-id', parent, '--child-frame-id', child])


def generate_launch_description():
    return LaunchDescription([
        # Panda wrist camera mount calibration.
        static('panda_hand_camera_tf',
               (-0.0225426486, -0.0454216748, 0.0344988497),
               (0.0232016724, -0.0270233572, 0.7093108444, 0.7039954165),
               'panda_hand_tcp', 'panda_hand_camera_link'),
        # Frozen snapshot: table AprilTag is the temporary world origin.
        # Frozen table-tag snapshot: world is the current tag reference.
        static('world_to_triton_snapshot', (0.098, -0.615, 0.665),
               (-0.931, 0.008, 0.023, 0.364), 'world',
               'lucid_triton_color_optical_frame'),
        # Exact saved Easy Hand-Eye result.
        static('kuka_base_to_triton_easy_handeye',
               (1.39593194, 0.26242952, 0.62941687),
               (-0.62659684, -0.67975793, 0.26630050, 0.27274458),
               'iiwa7_link_0', 'lucid_triton_color_optical_frame'),
        # Panda fixed-camera calibration (inverse of panda_link0 -> Triton).
        static('triton_to_panda_base_calibration',
               (-0.69437679, 0.11547400, 0.79943691),
               (0.93696988, 0.00547817, -0.01887198, 0.34885711),
               'lucid_triton_color_optical_frame', 'panda_link0'),
        # Existing KUKA eye-in-hand D455 calibration.
        static('kuka_d455_handeye_tf',
               (-0.0210401879, 0.0400542411, 0.0542652021),
               (0.0069869452, 0.0081553324, 0.3872469246, 0.9219134952),
               'iiwa7_link_ee', 'kuka_d455_color_optical_frame'),
    ])
