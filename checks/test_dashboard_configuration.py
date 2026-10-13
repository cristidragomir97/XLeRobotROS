"""Profiles must work without ROS, and derive control limits from a real URDF."""
from pathlib import Path
import tempfile
import unittest
import yaml
from robocore_viz.configuration import load_profile

ROOT = Path(__file__).resolve().parents[1] / "src"


class ProfileTests(unittest.TestCase):
    def test_xlerobot_limits_and_sensor_interfaces(self):
        profile = load_profile(ROOT/'xlerobot_sim/config/dashboard.yaml',
                               ROOT/'xlerobot_description/urdf/xlerobot.urdf')
        self.assertTrue(profile['drive']['omnidirectional'])
        self.assertEqual(profile['drive']['type'], 'geometry_msgs/msg/Twist')
        self.assertEqual(sum(len(g['joints']) for g in profile['joint_groups']), 14)
        self.assertNotIn('lidar', [t['kind'] for t in profile['telemetry']])
        for group in profile['joint_groups']:
            self.assertTrue(group['type'].startswith('control_msgs/action/'))
            for joint in group['joints']:
                self.assertLess(joint['min'], joint['max'])
                self.assertEqual(joint['velocity'], 1.5)

    def test_second_robot_profile(self):
        profile = load_profile(ROOT/'robocore_viz/examples/mr_handy.yaml')
        self.assertEqual(profile['name'], 'Mr Handy')
        self.assertFalse(profile['drive']['omnidirectional'])
        self.assertEqual(profile['drive']['type'], 'geometry_msgs/msg/TwistStamped')
        self.assertIn('lidar', [t['kind'] for t in profile['telemetry']])
        self.assertEqual(load_profile()['name'], 'Robot')

    def test_bad_controller_limits_and_unknown_settings(self):
        for document in ({'drive': {'linear_max': float('nan')}}, {'typo': True},
                         {'joint_groups': [{'title': 'Bad', 'topic': '/bad',
                                            'type': 'control_msgs/action/FollowJointTrajectory',
                                            'joints': ['nonexistent_joint']}]}):
            with tempfile.TemporaryDirectory() as directory:
                filename = Path(directory)/'profile.yaml'
                filename.write_text(yaml.safe_dump(document))
                with self.assertRaises(ValueError):
                    load_profile(filename)


if __name__ == '__main__':
    unittest.main()
