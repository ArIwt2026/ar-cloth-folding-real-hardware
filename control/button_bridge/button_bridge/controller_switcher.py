#!/usr/bin/env python3
import subprocess

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class PandaControllerSwitcher(Node):
    def __init__(self):
        super().__init__('panda_controller_switcher')
        self._zero_impedance_active = False
        self._service = self.create_service(
            Trigger, '/panda/joint_impedance/toggle', self._toggle)

    def _control(self, args):
        command = ['ros2', 'control', *args, '-c', '/panda/controller_manager']
        return subprocess.run(command, capture_output=True, text=True, timeout=10)

    def _ensure_loaded(self, name):
        listed = self._control(['list_controllers'])
        if listed.returncode != 0:
            raise RuntimeError((listed.stdout + listed.stderr).strip())
        # ros2_control adds ANSI color prefixes to this output, so do not use
        # startswith() on the raw line.
        controller_line = next(
            (line for line in listed.stdout.splitlines() if name in line), None)
        if controller_line is None:
            result = self._control(['load_controller', name])
            if result.returncode != 0:
                raise RuntimeError((result.stdout + result.stderr).strip())
            controller_line = name
        if 'inactive' in controller_line.lower():
            return
        result = self._control(['set_controller_state', name, 'inactive'])
        if result.returncode != 0 and 'already inactive' not in (result.stdout + result.stderr).lower():
            raise RuntimeError((result.stdout + result.stderr).strip())

    def _toggle(self, _request, response):
        try:
            if not self._zero_impedance_active:
                self._ensure_loaded('joint_impedance_zero_controller')
                result = self._control([
                    'switch_controllers',
                    '--deactivate', 'joint_impedance_hold_controller',
                    '--activate', 'joint_impedance_zero_controller',
                    '--strict'])
                message = 'Zero joint impedance activated.'
            else:
                result = self._control([
                    'switch_controllers',
                    '--deactivate', 'joint_impedance_zero_controller',
                    '--activate', 'joint_impedance_hold_controller',
                    '--strict'])
                message = 'High joint impedance activated.'
            if result.returncode != 0:
                raise RuntimeError((result.stdout + result.stderr).strip())
            self._zero_impedance_active = not self._zero_impedance_active
            response.success = True
            response.message = message
        except Exception as exc:
            response.success = False
            response.message = f'Controller switch failed: {exc}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PandaControllerSwitcher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
