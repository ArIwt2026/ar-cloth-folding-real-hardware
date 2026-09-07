from setuptools import setup

package_name = 'button_bridge'
setup(name=package_name, version='0.1.0', packages=[package_name], entry_points={
    'console_scripts': [
        'button_bridge = button_bridge.bridge:main',
        'button_bridge_switcher = button_bridge.controller_switcher:main',
    ],
})
