"""Robot profiles are data; the dashboard has no robot-specific topic names."""
from pathlib import Path
import copy
import math
import xml.etree.ElementTree as ET
import yaml

DEFAULTS = {
    'name': 'Robot', 'id': 'robot', 'tagline': 'Robot dashboard',
    'cameras': [{'topic': '/camera/image_raw', 'label': 'Camera'}],
    'maps': [{'topic': '/map', 'label': 'SLAM map'}],
    'topics': {'joints': '/joint_states', 'odom': '/odom', 'pose': '', 'plan': '/plan', 'tracking': ''},
    'frames': {'fixed': 'map', 'robot': ''},
    'drive': {'topic': '/cmd_vel', 'type': 'geometry_msgs/msg/Twist',
              'linear_max': .4, 'angular_max': .8, 'omnidirectional': False},
    'telemetry': [], 'joint_groups': [],
}


def load_profile(filename='', description_file=''):
    profile = copy.deepcopy(DEFAULTS)
    if filename:
        document = yaml.safe_load(Path(filename).read_text())
        if not isinstance(document, dict):
            raise ValueError('Dashboard profile must be a YAML mapping')
        for key, value in document.items():
            if key not in profile:
                raise ValueError(f'Unknown dashboard setting: {key}')
            if isinstance(profile[key], dict):
                profile[key].update(value)
            else:
                profile[key] = value
    for key in ('name', 'id', 'tagline'):
        if not isinstance(profile[key], str) or not profile[key]:
            raise ValueError(f'{key} must be a nonempty string')
    for key in ('cameras', 'maps'):
        if not profile[key] or any(not item.get('topic') or not item.get('label') for item in profile[key]):
            raise ValueError(f'{key} must list topic/label pairs')
    if not isinstance(profile['frames']['fixed'], str) or not profile['frames']['fixed']:
        raise ValueError('frames.fixed must name the map frame')
    if not isinstance(profile['frames']['robot'], str):
        raise ValueError('frames.robot must be a frame name or empty for odometry child frame')
    drive = profile['drive']
    if drive['type'] not in ('geometry_msgs/msg/Twist', 'geometry_msgs/msg/TwistStamped'):
        raise ValueError('Drive type must be Twist or TwistStamped')
    for key in ('linear_max', 'angular_max'):
        if not math.isfinite(drive[key]) or drive[key] <= 0:
            raise ValueError(f'Drive {key} must be finite and positive')
    kinds = {'localization', 'navigation', 'plan', 'odometry', 'imu', 'lidar', 'joints', 'compute'}
    for item in profile['telemetry']:
        if item.get('kind') not in kinds or item.get('section') not in ('navigation', 'hardware') or not item.get('topic'):
            raise ValueError('Telemetry needs a supported kind, section and topic')
    groups = profile['joint_groups']
    joints = {}
    if groups and description_file:
        joints = {j.get('name'): j for j in ET.parse(description_file).getroot().findall('joint')}
    for group in groups:
        if group.get('type') not in ('control_msgs/action/FollowJointTrajectory',
                                      'trajectory_msgs/msg/JointTrajectory', 'std_msgs/msg/Float64MultiArray'):
            raise ValueError('Unsupported joint controller interface')
        for i, joint in enumerate(group['joints']):
            if isinstance(joint, str):
                if joint not in joints:
                    raise ValueError(f'Joint {joint} needs a robot_description_file with limits')
                source = joints[joint]
                limit = source.find('limit')
                joint = {'name': joint, 'min': float(limit.get('lower')),
                         'max': float(limit.get('upper')), 'velocity': float(limit.get('velocity')),
                         'unit': 'm' if source.get('type') == 'prismatic' else 'rad'}
                group['joints'][i] = joint
            if not all(math.isfinite(joint[k]) for k in ('min', 'max', 'velocity')) or joint['min'] >= joint['max'] or joint['velocity'] <= 0:
                raise ValueError(f'Invalid limits for {joint["name"]}')
    return profile
