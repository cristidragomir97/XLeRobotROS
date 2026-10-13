"""Overlay LeKiwi's controller defaults for XLeRobot's existing joints/model."""
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory
from .model import HEAD_JOINTS, base_configuration
from .configuration import WHEEL_JOINTS, arm_joints


def controller_parameters(configuration='standard', wheel_tf=True):
    arms = arm_joints(configuration)
    upstream = Path(get_package_share_directory('lekiwi_control')) / 'config/control.yaml'
    config = yaml.safe_load(upstream.read_text())
    manager = config['controller_manager']['ros__parameters']
    manager.pop('imu_sensor_broadcaster', None)  # owned RGB-D/IMU publisher remains the sensor source
    manager['use_sim_time'] = True
    base = config['base_controller']['ros__parameters']
    geometry = base_configuration()
    base.update(wheel_names=WHEEL_JOINTS, **{k: geometry[k] for k in ('wheel_radius', 'robot_radius', 'wheel_offset')}, enable_odom_tf=wheel_tf,
                odom_frame_id="wheel_odom", base_frame_id="base_footprint",
                position_feedback=False, use_sim_time=True)
    config.pop('imu_sensor_broadcaster', None)
    config['joint_state_broadcaster'] = {'ros__parameters': {'use_sim_time': True}}
    for controller, names in [('left_arm_controller', arms['L']),
                              ('right_arm_controller', arms['R']),
                              ('head_controller', HEAD_JOINTS)]:
        manager[controller] = {'type': 'joint_trajectory_controller/JointTrajectoryController'}
        config[controller] = {'ros__parameters': {
            'joints': names, 'command_interfaces': ['position'],
            'state_interfaces': ['position', 'velocity'],
            'allow_partial_joints_goal': True, 'use_sim_time': True,
            'constraints': {'goal_time': 2., 'stopped_velocity_tolerance': .2,
                            **{name: {'goal': .08} for name in names}}}}
    return config


def controller_file(configuration='standard', wheel_tf=True):
    with tempfile.NamedTemporaryFile(mode='w', prefix='xlerobot-controllers-', suffix='.yaml', delete=False) as output:
        yaml.safe_dump(controller_parameters(configuration, wheel_tf), output)
        return output.name


def control_description(description, runtime):
    # The canonical URDF owns all interfaces and geometry. Resolve only the runtime file.
    root = ET.fromstring(description)
    root.find("ros2_control/hardware/param[@name='state_file']").text = str(Path(runtime) / 'control.bin')
    return ET.tostring(root, encoding='unicode')
