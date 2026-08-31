from setuptools import setup


package_name = "franka_handeye_calibration"


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (
            f"share/{package_name}",
            ["package.xml", "README.md"],
        ),
        (
            f"share/{package_name}/launch",
            [
                "launch/handeye_calibration.launch.py",
                "launch/kuka_calibration.launch.py",
                "launch/temp_sequence_debug.launch.py",
            ],
        ),
        (
            f"share/{package_name}/config/jobs",
            [
                "config/jobs/d455_handeye.yaml",
                "config/jobs/panda_eye_on_base.yaml",
                "config/jobs/kuka_eye_in_hand.yaml",
                "config/jobs/panda_fixed_lucid_apriltag.yaml",
                "config/jobs/kuka_fixed_lucid_apriltag.yaml",
                "config/jobs/temp_sequence_debug.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Local User",
    maintainer_email="user@example.com",
    description="Event-driven Franka hand-eye calibration built on taught sequences.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "coordinator_node = franka_handeye_calibration.coordinator_main:main",
            "handeye_cli = franka_handeye_calibration.calibration_cli:main",
            "sample_collector = franka_handeye_calibration.sample_collector_node:main",
            "raw_handeye_solver = franka_handeye_calibration.raw_handeye_solver:main",
            "lucid_apriltag_tf = franka_handeye_calibration.apriltag_tf_publisher:main",
        ],
    },
)
