from setuptools import setup

package_name = 'gripper_button_bridge'
setup(name=package_name, version='0.1.0', packages=[package_name], entry_points={
    'console_scripts': ['gripper_button_bridge = gripper_button_bridge.bridge:main'],
})
