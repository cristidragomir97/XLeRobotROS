"""Arm profiles shared by description, simulator, hardware I/O and controllers."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

WHEEL_JOINTS = ['ST3215_Servo_Motor-v1-2_Hub---Servo',
                'ST3215_Servo_Motor-v1-1_Hub-2---Servo',
                'ST3215_Servo_Motor-v1_Revolute-40']
HEAD_JOINTS = ['head_pan_joint', 'head_tilt_joint']


def arm_joints(configuration='standard'):
    if configuration == 'standard':
        return {s:[f'{n}_{s}' for n in ('Rotation','Pitch','Elbow','Wrist_Pitch','Wrist_Roll','Jaw')] for s in ('L','R')}
    if configuration == 'mod101':
        return {s:[f'mod101_{s}_{n}' for n in ('joint_base','joint_shoulder','joint_elbow',
                  'joint_wrist_yaw','joint_wrist_tilt','joint_wrist_roll','6')] for s in ('L','R')}
    raise ValueError(f'Unknown configuration: {configuration}')


def control_joints(configuration='standard'):
    groups=arm_joints(configuration)
    return WHEEL_JOINTS+groups['L']+groups['R']+HEAD_JOINTS


def description_path(configuration, runtime):
    arm_joints(configuration)  # reject unsupported configurations
    directory=Path(get_package_share_directory('xlerobot_description'))/'urdf'
    if configuration=='standard':
        return directory/'xlerobot.urdf'
    import xacro
    path=Path(runtime)/'xlerobot_mod101.urdf'
    path.parent.mkdir(parents=True,exist_ok=True)
    doc=xacro.process_file(str(directory/'xlerobot_mod101.urdf.xacro'))
    path.write_text(doc.toprettyxml(indent='  '))
    return path
