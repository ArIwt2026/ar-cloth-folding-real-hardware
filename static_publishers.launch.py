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
        # Panda D455 eye-in-hand calibration from Easy Handeye. The RealSense
        # driver then publishes panda_d455_link -> panda_d455_color_optical_frame.
        static('panda_d455_handeye_tf',
               (0.05006900349042004, 0.015266267172393305, 0.07409719296917006),
               (-0.26623178107864587, -0.6453999784702183,
                -0.2837929078727779, 0.6572983279877153),
               'panda_link8', 'panda_d455_link'),
        # Frozen snapshot: table AprilTag is the temporary world origin.
        # Frozen table-tag snapshot: world is the current tag reference.
        static('world_to_triton_snapshot', (0.098, -0.615, 0.665),
               (-0.931, 0.008, 0.023, 0.364), 'world',
               'lucid_triton_color_optical_frame'),
        # Inverse of the saved Easy Hand-Eye result so the fixed Lucid camera
        # remains the single parent of both robot bases in the world TF tree.
        static('triton_to_kuka_base_calibration',
               (-0.19753837, -0.61974747, 1.44356388),
               (-0.64941709, -0.66851211, 0.26902736, -0.24284415),
               'lucid_triton_color_optical_frame', 'iiwa7_link_0'),
        # Panda fixed-camera calibration (inverse of panda_link0 -> Triton).
        static('triton_to_panda_base_calibration',
               (-0.69437679, 0.11547400, 0.79943691),
               (0.93696988, 0.00547817, -0.01887198, 0.34885711),
               'lucid_triton_color_optical_frame', 'panda_link0'),
        # KUKA D455 eye-in-hand calibration. The camera driver owns
        # kuka_d455_link -> kuka_d455_color_optical_frame, so attach the
        # calibrated EE pose to kuka_d455_link to avoid duplicate parents.
        # Canonical record:
        # control/calibration/config/calibrations/kuka_d455_eye_in_hand.yaml.
        static('kuka_d455_handeye_tf',
               (-0.062337038479209, -0.002079446160069, 0.054833116964160),
               (0.662151348834694, -0.266749091542836,
                0.647009070900363, 0.267917479030610),
               'iiwa7_link_ee', 'kuka_d455_link'),
    ])
