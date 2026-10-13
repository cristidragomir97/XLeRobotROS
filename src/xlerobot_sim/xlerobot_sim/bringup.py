"""Head RGB-D + base IMU; RTAB-Map owns estimated odometry and map transforms."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from xlerobot_sim.world import default_assets
from xlerobot_sim.control_setup import controller_file, control_description
from xlerobot_sim.configuration import description_path


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory('xlerobot_sim'))
    configuration = value('configuration')
    description = description_path(configuration, value('runtime'))
    # Launch is protected by run_sim.sh's lifetime lock. Discard only transient
    # I/O from the previous session; preserve every map database and asset.
    runtime = Path(value('runtime'))
    runtime.mkdir(parents=True, exist_ok=True)
    for filename in ('control.bin', 'physics.npz', 'state.npz'):
        (runtime / filename).unlink(missing_ok=True)
    mode, odometry = value('mode'), value('odometry')
    database = value('database') or str(Path(value('runtime')) / f"{value('world')}{'_mod101' if configuration == 'mod101' else ''}.db")
    if mode not in ('mapping', 'localization', 'none'):
        raise ValueError('mode must be mapping, localization, or none')
    if odometry not in ('rgbd', 'ground_truth'):
        raise ValueError('odometry must be rgbd or ground_truth')
    if mode != 'none' and odometry != 'rgbd':
        raise ValueError('SLAM modes require RGB-D odometry; ground_truth is a diagnostic mode only')
    if mode == 'localization' and not Path(database).is_file():
        raise ValueError('Localization requires an existing RTAB-Map database from a mapping run')
    if value('nav') == 'true' and mode == 'none':
        raise ValueError('Navigation needs mapping or localization')
    camera_remaps = [('rgb/image', '/camera/color/image_raw'),
                     ('rgb/camera_info', '/camera/color/camera_info'),
                     ('depth/image', '/camera/depth_filtered/image_raw'),
                     ('imu', '/imu/data')]
    rgbd_params = {'use_sim_time': True, 'frame_id': 'base_footprint',
                   'odom_frame_id': 'odom', 'approx_sync': False,
                   'qos': 1, 'qos_camera_info': 1, 'qos_imu': 2,
                   'wait_imu_to_init': True, 'publish_tf': True,
                   'Reg/Force3DoF': 'true', 'Vis/MinInliers': '15',
                   'guess_frame_id': 'wheel_odom', 'publish_null_when_lost': False,
                   'Odom/Strategy': '0', 'Odom/ResetCountdown': '10'}
    simulator = Node(package='xlerobot_sim', executable='sim', output='screen', parameters=[{
        'upstream': value('upstream'), 'runtime': value('runtime'), 'world': value('world'),
        'world_assets': value('world_assets'), 'configuration': configuration,
        'description': str(description), 'headless': value('headless') == 'true',
        'camera_rate': float(value('camera_rate')), 'use_sim_time': True,
        'ground_truth_tf': odometry == 'ground_truth'}])
    actions = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': control_description(description.read_text(), value('runtime')), 'use_sim_time': True}], output='screen'),
        simulator,
        RegisterEventHandler(OnProcessExit(target_action=simulator,
            on_exit=[EmitEvent(event=Shutdown(reason='MuJoCo simulator closed'))])),
    ]
    camera = Node(package='xlerobot_sim', executable='camera', output='screen',
                  parameters=[{'runtime': value('runtime'), 'use_sim_time': True}])
    actions += [TimerAction(period=2., actions=[camera]),
        RegisterEventHandler(OnProcessExit(target_action=camera,
            on_exit=[EmitEvent(event=Shutdown(reason='RGB-D camera renderer exited'))]))]
    manager = Node(package='controller_manager', executable='ros2_control_node',
                   name='controller_manager', output='screen', parameters=[controller_file(configuration, wheel_tf=odometry == "rgbd"),
                       {'use_sim_time': True}],
                   remappings=[('~/robot_description', '/robot_description')])
    spawner = Node(package='controller_manager', executable='spawner', output='screen',
                   arguments=['joint_state_broadcaster', 'base_controller', 'left_arm_controller',
                              'right_arm_controller', 'head_controller', '--activate-as-group',
                              '--controller-manager-timeout', '60'])
    actions += [manager, spawner,
        RegisterEventHandler(OnProcessExit(target_action=spawner,
            on_exit=lambda event, context: [EmitEvent(event=Shutdown(reason='Controller activation failed'))] if event.returncode else [])),
        RegisterEventHandler(OnProcessExit(target_action=manager,
            on_exit=[EmitEvent(event=Shutdown(reason='Controller manager exited'))])),
        Node(package='lekiwi_control', executable='twist_switch_node', output='screen',
             parameters=[{'use_sim_time': True, 'input_topic_default': '/cmd_vel',
                          'input_topic_default.stamped': False,
                          'input_topic_switched': '/cmd_vel_external',
                          'input_topic_switched.stamped': False,
                          'output_topic': '/base_controller/cmd_vel',
                          'output_topic.stamped': True, 'frame_id': 'base_footprint'}]),
    ]
    if value('robocore_viz') == 'true':
        actions.append(Node(package='robocore_viz', executable='robocore_viz_node',
                            name='robocore_viz_node', output='screen', parameters=[{
                                'port': int(value('robocore_viz_port')),
                                'config_file': value('dashboard_config') or str(share / 'config' /
                                    ('dashboard_mod101.yaml' if configuration == 'mod101' else 'dashboard.yaml')),
                                'robot_description_file': str(description), 'use_sim_time': True}]))
    if value('mjviser') == 'true':
        actions.append(TimerAction(period=3., actions=[Node(
            package='xlerobot_sim', executable='viewer', output='screen',
            arguments=['--runtime', value('runtime'), '--port', value('mjviser_port')])]))
    if odometry == 'rgbd':
        actions.append(Node(package='rtabmap_odom', executable='rgbd_odometry',
                            name='rgbd_odometry', parameters=[rgbd_params],
                            remappings=camera_remaps, output='screen'))
    if mode != 'none':
        params = {'use_sim_time': True, 'frame_id': 'base_footprint',
                  'map_frame_id': 'map', 'subscribe_depth': True,
                  'subscribe_odom_info': True, 'approx_sync': False,
                  'qos': 2, 'qos_odom': 2, 'qos_imu': 2,
                  'database_path': database,
                  'Reg/Force3DoF': 'true', 'Grid/Sensor': '1',
                  'Grid/3D': 'false', 'Grid/RangeMax': '8.0',
                  'Grid/CellSize': '0.05', 'Grid/MaxGroundHeight': '0.1',
                  'Grid/MaxObstacleHeight': '2.0',
                  'Rtabmap/DetectionRate': '3.0',
                  'Mem/DepthCompressionFormat': '.png',
                  'Mem/IncrementalMemory': 'false' if mode == 'localization' else 'true',
                  'Mem/InitWMWithAllNodes': 'true' if mode == 'localization' else 'false'}
        actions.append(Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap',
                            parameters=[params], remappings=camera_remaps + [('grid_map', '/map')],
                            output='screen'))
    actions.append(Node(package='rtabmap_util', executable='point_cloud_xyzrgb',
                        name='depth_cloud', parameters=[{'use_sim_time': True, 'approx_sync': False,
                        'qos': 2, 'decimation': 4, 'max_depth': 8.0, 'voxel_size': 0.03}],
                        remappings=camera_remaps[:3]+[('cloud', '/camera/points')], output='screen'))
    if value('nav') == 'true':
        servers = [('nav2_controller', 'controller_server'),
                   ('nav2_planner', 'planner_server'), ('nav2_smoother', 'smoother_server'),
                   ('nav2_behaviors', 'behavior_server'), ('nav2_bt_navigator', 'bt_navigator'),
                   ('nav2_velocity_smoother', 'velocity_smoother')]
        for package, executable in servers:
            remaps = [('cmd_vel', '/cmd_vel_nav')] if executable in ('controller_server', 'behavior_server', 'velocity_smoother') else []
            if executable == 'velocity_smoother':
                remaps += [('cmd_vel_smoothed', '/cmd_vel')]
            actions.append(Node(package=package, executable=executable, name=executable,
                                parameters=[str(share / 'config/nav2.yaml'),
                                            {'use_sim_time': True, 'enable_stamped_cmd_vel': False}],
                                remappings=remaps, output='screen'))
        actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                            name='lifecycle_manager_navigation', parameters=[{
                                'use_sim_time': True, 'autostart': True,
                                'node_names': [name for _, name in servers]}], output='screen'))
    if value('rviz') == 'true':
        actions.append(Node(package='rviz2', executable='rviz2',
                            arguments=['-d', str(share / 'config/xlerobot.rviz')],
                            parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description(configuration='standard'):
    defaults = {'configuration': configuration, 'upstream': '/workspace/xlerobot/upstream/simulation/mujoco',
                'runtime': '/workspace/xlerobot/.runtime', 'world': 'robocasa_home',
                'world_assets': default_assets(), 'headless': 'false',
                'mode': 'mapping', 'odometry': 'rgbd', 'mjviser': 'true',
                'viewer': 'true', 'mjviser_port': '8080',
                'robocore_viz': 'true', 'robocore_viz_port': '8888',
                'dashboard_config': '',
                'camera_rate': '15.0', 'nav': 'true', 'rviz': 'false',
                'database': ''}
    # Keep the old viewer argument as an alias; mjviser is the neighboring stacks' name.
    return LaunchDescription([*[DeclareLaunchArgument(k, default_value=v)
                               for k, v in defaults.items() if k != 'mjviser'],
                               DeclareLaunchArgument('mjviser', default_value=LaunchConfiguration('viewer')),
                               OpaqueFunction(function=setup)])
