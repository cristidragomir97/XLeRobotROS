#!/usr/bin/env python3
"""Validate the dashboard's real HTTP/WS data, action controls and drive watchdog."""
import asyncio
import json
import time
from urllib.request import urlopen
from tornado.websocket import websocket_connect

BASE = 'http://127.0.0.1:8888'


def get(path):
    with urlopen(BASE + path, timeout=10) as response:
        return json.load(response)


async def main():
    config = get('/robocore_viz/config')
    assert config['name'] == 'XLeRobot' and config['drive']['omnidirectional']
    groups = get('/robocore_viz/joint-controls')['groups']
    mod101 = any(j['name'].startswith('mod101_L_') for g in groups for j in g['joints'])
    count = 19 if mod101 else 17
    assert sum(len(g['joints']) for g in groups) == count-3
    graph = get('/robocore_viz/graph')
    assert any(n['name'] == 'xlerobot_sim' for n in graph['nodes'])
    ws = await websocket_connect('ws://127.0.0.1:8888/robocore_viz/v1', connect_timeout=10)
    samples = {}
    targets = ['/joint_states', '/camera/color/image_raw', '/camera/aligned_depth_to_color/image_raw',
               '/map', '/imu/data', '/ground_truth/odom', '_robot_pose', '/odom_info_lite']
    def send(op, payload):
        ws.write_message(json.dumps([op, payload], allow_nan=False))
    async def wait(predicate, timeout=25):
        async with asyncio.timeout(timeout):
            while not predicate():
                raw = await ws.read_message()
                assert raw is not None
                op, data = json.loads(raw, parse_constant=lambda x: (_ for _ in ()).throw(ValueError(x)))
                if op == 'p':
                    send('q', data)
                elif op == 'm':
                    assert '_error' not in data, data
                    samples[data['_topic_name']] = data
    def command(topic, kind, msg):
        send('P', {'topicName': topic, 'topicType': kind, 'msg': msg})
    def joint(name):
        message = samples['/joint_states']
        return message['position'][message['name'].index(name)]
    try:
        for topic in targets:
            send('s', {'topicName': topic, 'maxUpdateRate': 10})
        try:
            await wait(lambda: all(t in samples for t in targets))
        except TimeoutError:
            print('Missing dashboard streams:',set(targets)-set(samples),flush=True)
            raise
        # A second client must receive the durable map even while the first
        # keeps the shared ROS subscription alive and the robot stays still.
        late = await websocket_connect('ws://127.0.0.1:8888/robocore_viz/v1', connect_timeout=10)
        try:
            late.write_message(json.dumps(['s', {'topicName': '/map', 'maxUpdateRate': 10}]))
            async with asyncio.timeout(5):
                while True:
                    op, data = json.loads(await late.read_message())
                    if op == 'p':
                        late.write_message(json.dumps(['q', data]))
                    elif op == 'm' and data['_topic_name'] == '/map':
                        assert data['info']['width'] > 0
                        break
        finally:
            late.close()
        assert len(samples['/joint_states']['name']) == count
        assert samples['/camera/color/image_raw']['_data_jpeg']
        depth = samples['/camera/aligned_depth_to_color/image_raw']
        assert depth['_data_jpeg'] and depth['_depth_range'][1] > 1
        assert samples['/map']['_data_jpeg']
        pose=samples['_robot_pose']
        assert pose['header']['frame_id']=='map'
        assert pose['_ros_time']-pose['header']['stamp']['sec']-pose['header']['stamp']['nanosec']*1e-9<1
        assert not samples['/odom_info_lite']['lost']
        print('PASS: current map-frame TF pose and visual tracking status over websocket')
        print(f'PASS: configured dashboard, ROS graph, {count} joints, RGB/metric-depth previews, map and IMU')
        group = next(g for g in groups if g['title'] == 'Head')
        names = [j['name'] for j in group['joints']]
        initial = [joint(n) for n in names]
        positions = initial.copy()
        positions[0] += .12
        def trajectory(values):
            command(group['topic'], group['type'], {'joint_names': names,
                'points': [{'positions': values, 'time_from_start': {'sec': 1, 'nanosec': 0}}]})
        trajectory(positions)
        await wait(lambda: abs(joint(names[0]) - positions[0]) < .035)
        await asyncio.sleep(1)
        trajectory(initial)
        await wait(lambda: abs(joint(names[0]) - initial[0]) < .035)
        print('PASS: browser-compatible FollowJointTrajectory commands move and restore the head')
        start = samples['/ground_truth/odom']['pose']['pose']['position']['y']
        twist = {'linear': {'x': 0., 'y': .18, 'z': 0.}, 'angular': {'x': 0., 'y': 0., 'z': 0.}}
        for _ in range(10):
            command(config['drive']['topic'], config['drive']['type'], twist)
            await asyncio.sleep(.08)
        await wait(lambda: samples['/ground_truth/odom']['pose']['pose']['position']['y'] > start + .06)
        quiet_start = time.monotonic()
        await wait(lambda: time.monotonic() - quiet_start > 1.5 and
                   abs(samples['/ground_truth/odom']['twist']['twist']['linear']['y']) < .01, timeout=8)
        print('PASS: dashboard strafe commands move the base; silence stops motion')
    finally:
        command(config['drive']['topic'], config['drive']['type'],
                {'linear': {'x': 0., 'y': 0., 'z': 0.}, 'angular': {'x': 0., 'y': 0., 'z': 0.}})
        ws.close()


if __name__ == '__main__':
    asyncio.run(main())
