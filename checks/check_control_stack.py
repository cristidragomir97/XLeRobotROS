#!/usr/bin/env python3
"""Verify actual ros2_control ownership, not just action/topic names."""
from xlerobot_sim.configuration import control_joints
import rclpy
import time
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers, ListHardwareInterfaces, ListHardwareComponents

rclpy.init()
node = Node('xlerobot_control_contract')
try:
    def query(kind, name):
        client = node.create_client(kind, '/controller_manager/' + name)
        assert client.wait_for_service(timeout_sec=15), name
        future = client.call_async(kind.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        assert future.done() and future.result() is not None
        result = future.result()
        node.destroy_client(client)
        return result
    controllers = query(ListControllers, 'list_controllers').controller
    expected = {'joint_state_broadcaster', 'base_controller', 'left_arm_controller', 'right_arm_controller', 'head_controller'}
    deadline = time.monotonic()+20
    while {c.name for c in controllers if c.state == 'active'} != expected and time.monotonic()<deadline:
        rclpy.spin_once(node, timeout_sec=.2)
        controllers = query(ListControllers, 'list_controllers').controller
    assert {c.name for c in controllers if c.state == 'active'} == expected, controllers
    base = next(c for c in controllers if c.name == 'base_controller')
    assert base.type == 'omni_wheel_drive_controller/OmniWheelDriveController'
    for name in ('left_arm_controller', 'right_arm_controller', 'head_controller'):
        assert next(c for c in controllers if c.name == name).type == 'joint_trajectory_controller/JointTrajectoryController'
    interfaces = query(ListHardwareInterfaces, 'list_hardware_interfaces')
    # The chainable omni controller also exports three reference commands.
    hardware_commands = [i for i in interfaces.command_interfaces if not i.name.startswith('base_controller/')]
    configuration = 'mod101' if any(i.name.startswith('mod101_L_') for i in hardware_commands) else 'standard'
    names = control_joints(configuration)
    assert {i.name.rsplit('/',1)[0] for i in hardware_commands} == set(names), hardware_commands
    assert all(i.is_claimed and i.is_available for i in hardware_commands)
    assert len(interfaces.state_interfaces) == 2*len(names)
    components = query(ListHardwareComponents, 'list_hardware_components').component
    assert len(components) == 1 and components[0].state.label == 'active', components
    assert components[0].plugin_name == 'xlerobot_control/SimulatorSystem'
    print(f'PASS: {configuration} controller_manager owns {len(names)} claimed commands and {2*len(names)} state interfaces; standard omni/trajectory controllers are active')
finally:
    node.destroy_node()
    rclpy.shutdown()
