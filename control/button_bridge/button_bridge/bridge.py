#!/usr/bin/env python3
import threading
import queue
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger
from wsg50_msgs.action import Command as WsgCommand
from wsg50_msgs.msg import State as WsgState

try:
    from franka_msgs.action import Move as PandaMove
    from franka_msgs.action import Grasp as PandaGrasp
except ImportError:
    PandaMove = None
    PandaGrasp = None


class GripperButtonBridge(Node):
    """Translate Arduino AA/state/55 packets into non-blocking gripper actions."""

    def __init__(self):
        super().__init__('button_bridge')
        self.declare_parameter(
            'serial_device',
            '/dev/serial/by-id/usb-Arduino__www.arduino.cc__0043_95635333031351704032-if00')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('panda_open_width', 0.05)
        # Panda uses one speed for both opening and closing. Match the WSG50
        # opening speed; the WSG50 close speed remains separately configurable.
        self.declare_parameter('panda_speed', 0.420)
        self.declare_parameter('panda_force', 20.0)
        # WSG50 targets: closed-to-zero (may stop on an object) and 50 mm open.
        self.declare_parameter('wsg_open_width', 0.05)
        self.declare_parameter('wsg_speed', 0.420)
        self.declare_parameter('wsg_close_speed', 0.210)
        self.declare_parameter('wsg_acceleration', 5.0)
        self.declare_parameter('wsg_force', 40.0)  # 50% of the 5-80 N range, rounded safely.
        self._last_press_time = {0x01: 0.0, 0x02: 0.0}
        # Match the Arduino debounce. Do not add a long ROS-side dead time:
        # consecutive valid press/release cycles must all be preserved.
        self._press_guard_s = 0.02
        self.declare_parameter('wsg_command_action', '/wsg50/driver/command')
        self.declare_parameter('wsg_stop_service', '/wsg50/driver/stop')
        self._panda_open = True
        self._wsg_open = True
        self._wsg_holding_object = False
        self._wsg_last_command_open = None
        self._wsg_result_event = None
        self._panda_move = ActionClient(self, PandaMove, '/panda/panda_gripper/move') if PandaMove else None
        self._panda_grasp = ActionClient(self, PandaGrasp, '/panda/panda_gripper/grasp') if PandaGrasp else None
        self._wsg = ActionClient(
            self, WsgCommand, self.get_parameter('wsg_command_action').value)
        self._panda_stop = self.create_client(Trigger, '/panda/panda_gripper/stop') if PandaMove else None
        self._gravity_toggle = self.create_client(
            Trigger, '/panda/gravity_compensation/toggle')
        self._wsg_stop = self.create_client(
            Trigger, self.get_parameter('wsg_stop_service').value)
        self._event_id = 0
        self._events = queue.Queue()
        self._command_busy = {0x01: False, 0x02: False}
        self._command_locks = {0x01: threading.Lock(), 0x02: threading.Lock()}
        # Strict blocking mode: presses received during an active command are
        # deliberately ignored. The next press after completion is accepted.
        self._wsg_state = None
        self._wsg_state_sub = self.create_subscription(
            WsgState, '/wsg50_gripper_driver/state', self._wsg_state_callback, 10)
        self._event_timer = self.create_timer(0.01, self._process_events)
        self._serial = None
        self._reader = None
        self._open_serial()

    def _open_serial(self):
        try:
            import serial
            self._serial = serial.Serial(
                self.get_parameter('serial_device').value,
                int(self.get_parameter('baudrate').value), timeout=0.1)
            self.get_logger().info(f'Connected to Arduino on {self._serial.port}')
            self._reader = threading.Thread(target=self._read_loop, daemon=True)
            self._reader.start()
        except Exception as exc:
            self.get_logger().error(f'Cannot open Arduino serial port: {exc}')

    def _read_loop(self):
        # Packet parser is deliberately byte-oriented, so partial reads and noise are safe.
        state = 0
        while rclpy.ok() and self._serial:
            byte = self._serial.read(1)
            if not byte:
                continue
            value = byte[0]
            if state == 0 and value == 0xAA:
                state = 1
            elif state == 1:
                self._packet_state = value
                state = 2
            elif state == 2:
                if value == 0x55:
                    new_state = self._packet_state
                    self.get_logger().info(f'Arduino event packet: {new_state}')
                    handlers = {1: (('PANDA', 0x01, self._panda_button),),
                                2: (('WSG50', 0x02, self._wsg_button),),
                                3: (('PANDA', 0x01, self._panda_button),
                                    ('WSG50', 0x02, self._wsg_button))}.get(new_state)
                    if new_state in (4, 5):
                        button_name = 'PANDA' if new_state == 4 else 'WSG50'
                        self.get_logger().info(
                            f'DOUBLE_PRESS button={button_name} event={new_state}')
                        if new_state == 4:
                            self._toggle_gravity_compensation()
                        state = 0
                        continue
                    if handlers is None:
                        self.get_logger().warn(
                            f'IGNORED packet={new_state} reason=invalid_event_code')
                    else:
                        for button_name, bit, handler in handlers:
                            self._event_id += 1
                            event_id = self._event_id
                            self.get_logger().info(
                                f'PRESS id={event_id} button={button_name} event={new_state}')
                            if self._command_busy[bit]:
                                self.get_logger().warn(
                                    f'IGNORED id={event_id} reason=command_active')
                            else:
                                self._command_busy[bit] = True
                                self.get_logger().info(f'ACCEPTED id={event_id} reason=idle')
                                self._events.put((bit, handler))
                state = 0

    def _process_events(self):
        try:
            while True:
                bit, callback = self._events.get_nowait()
                # Execute commands in order and wait for each action result.
                threading.Thread(
                    target=self._run_command, args=(bit, callback), daemon=True).start()
        except queue.Empty:
            pass

    def _run_command(self, bit, callback):
        with self._command_locks[bit]:
            callback()
        self._command_busy[bit] = False

    def _wsg_state_callback(self, message):
        self._wsg_state = message
        if message.connected and message.referenced:
            # The WSG is considered open when its actual opening is above the
            # midpoint between closed and the configured open target.
            open_width = float(self.get_parameter('wsg_open_width').value)
            self._wsg_open = message.width >= open_width * 0.5

    def _panda_button(self):
        if self._panda_move is None or self._panda_grasp is None:
            self.get_logger().error('Panda interfaces are not available in this ROS environment')
            return
        self._panda_open = not self._panda_open
        panda_client = self._panda_move if self._panda_open else self._panda_grasp
        if not panda_client.wait_for_server(timeout_sec=0.2):
            self.get_logger().error(
                f'Panda action is unavailable: {"move" if self._panda_open else "grasp"}')
            return
        self._stop(self._panda_stop, 'Panda')
        if self._panda_open:
            goal = PandaMove.Goal()
            goal.width = float(self.get_parameter('panda_open_width').value)
            goal.speed = float(self.get_parameter('panda_speed').value)
        else:
            goal = PandaGrasp.Goal()
            goal.width = 0.0
            goal.speed = float(self.get_parameter('panda_speed').value)
            goal.force = float(self.get_parameter('panda_force').value)
        panda_client.send_goal_async(goal)

    def _toggle_gravity_compensation(self):
        if not self._gravity_toggle.wait_for_service(timeout_sec=0.2):
            self.get_logger().error(
                'Gravity toggle service unavailable: '
                '/panda/gravity_compensation/toggle')
            return
        future = self._gravity_toggle.call_async(Trigger.Request())
        future.add_done_callback(self._gravity_toggle_result)

    def _gravity_toggle_result(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'GRAVITY_TOGGLE success: {response.message}')
            else:
                self.get_logger().error(f'GRAVITY_TOGGLE failed: {response.message}')
        except Exception as exc:
            self.get_logger().error(f'GRAVITY_TOGGLE service error: {exc}')

    def _wsg_button(self):
        if not self._wsg.wait_for_server(timeout_sec=0.2):
            self.get_logger().error(
                f'COMMAND_FAILED reason=action_unavailable '
                f'action={self.get_parameter("wsg_command_action").value}')
            return
        if self._wsg_state is not None:
            if not self._wsg_state.connected or not self._wsg_state.referenced:
                self.get_logger().error(
                    f'COMMAND_FAILED reason=not_ready connected={self._wsg_state.connected} '
                    f'referenced={self._wsg_state.referenced} '
                    f'width={self._wsg_state.width:.3f}')
                return
            open_width = float(self.get_parameter('wsg_open_width').value)
            current_open = self._wsg_state.width >= open_width * 0.5
        else:
            current_open = self._wsg_open
        # A stalled GRASP means the object stopped the jaws. The next button
        # press must release it, never issue another zero-width grasp.
        next_open = self._wsg_holding_object or not current_open
        self._wsg_last_command_open = next_open
        self.get_logger().info(
            f'WSG50 command: action={"MOVE_OPEN" if self._wsg_holding_object else "MOVE/GRASP"} '
            f'width={float(self.get_parameter("wsg_open_width").value) if next_open else 0.0:.3f} '
            f'holding_object={self._wsg_holding_object}')
        # Do not stop immediately before a normal command. The bridge is
        # blocking, so no prior WSG motion is active here; issuing stop and
        # RELEASE/GRASP back-to-back can make the controller reject the motion
        # with "access denied". Keep the stop service for explicit emergency
        # handling only.
        goal = WsgCommand.Goal()
        # Opening is a position move; closing is a force-limited grasp so an
        # object may stop the jaws before they reach zero width.
        # Release a grasp with an explicit position move. The WSG RELEASE
        # command can be denied when the jaws stopped at an arbitrary object
        # width; MOVE to the configured opening is deterministic.
        goal.mode = WsgCommand.Goal.MOVE if next_open else WsgCommand.Goal.GRASP
        goal.width = float(self.get_parameter('wsg_open_width').value) if next_open else 0.0
        goal.speed = float(
            self.get_parameter('wsg_speed').value if next_open else
            self.get_parameter('wsg_close_speed').value)
        goal.acceleration = float(self.get_parameter('wsg_acceleration').value)
        goal.force = float(self.get_parameter('wsg_force').value)
        # A close command targets zero, but the WSG must stop safely on contact.
        goal.stop_on_block = not next_open
        result_event = threading.Event()
        self._wsg_result_event = result_event
        goal_future = self._wsg.send_goal_async(goal)
        goal_future.add_done_callback(self._wsg_goal_response)
        # Blocking command semantics: do not process another command until this
        # action has returned. ROS callbacks continue on the main executor thread.
        result_event.wait()

    def _wsg_goal_response(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error(
                    'COMMAND_REJECTED reason=driver_rejected '
                    'possible_causes=not_homed_or_invalid_goal')
                if self._wsg_result_event:
                    self._wsg_result_event.set()
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self._wsg_result)
        except Exception as exc:
            self.get_logger().error(f'WSG50 command failed: {exc}')
            if self._wsg_result_event:
                self._wsg_result_event.set()

    def _wsg_result(self, future):
        try:
            result = future.result().result
            if result.reached_goal or result.stalled:
                self._wsg_open = not self._wsg_open
                if self._wsg_last_command_open:
                    self._wsg_holding_object = False
                elif result.stalled:
                    self._wsg_holding_object = True
            self.get_logger().info(
                f'WSG50 action completed: width={result.width:.3f} m, '
                f'reached_goal={result.reached_goal}, stalled={result.stalled}, '
                f'holding_object={self._wsg_holding_object}')
        except Exception as exc:
            self.get_logger().error(f'WSG50 result failed: {exc}')
        finally:
            if self._wsg_result_event:
                self._wsg_result_event.set()

    def _stop(self, client, name):
        if client.service_is_ready():
            client.call_async(Trigger.Request())
        else:
            self.get_logger().warn(f'{name} stop service is unavailable')


def main(args=None):
    rclpy.init(args=args)
    node = GripperButtonBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
