#!/usr/bin/env python3
"""Send one short Nav2 goal using the live RGB-D map and verify physical movement."""
import math

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist

from check_runtime import Check


def main():
    rclpy.init()
    node = Check()
    handle = None
    try:
        node.wait(lambda: set(node.messages) >= {'truth', 'map', 'odom'})
        node.wait(lambda: node.buffer.can_transform('map', 'base_footprint', Time()))
        pose = node.buffer.lookup_transform('map', 'base_footprint', Time()).transform
        quat = pose.rotation
        yaw = math.atan2(2*(quat.w*quat.z+quat.x*quat.y), 1-2*(quat.y*quat.y+quat.z*quat.z))
        client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        assert client.wait_for_server(timeout_sec=15.), 'Nav2 action server unavailable; launch nav:=true'
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = pose.translation.x+.6*math.cos(yaw)
        goal.pose.pose.position.y = pose.translation.y+.6*math.sin(yaw)
        goal.pose.pose.orientation = quat
        physical = node.messages['truth'].pose.pose.position
        start = np.array([physical.x, physical.y])
        handle = node.future(client.send_goal_async(goal))
        assert handle.accepted, 'Navigation goal rejected'
        outcome = node.future(handle.get_result_async(), 90)
        assert outcome.status == 4, outcome
        physical = node.messages['truth'].pose.pose.position
        moved = np.linalg.norm(np.array([physical.x, physical.y])-start)
        assert moved > .25, f'Nav2 reported success without sufficient physical movement: {moved}'
        print(f'PASS: Nav2 reached a 0.6 m RGB-D map goal; simulated base moved {moved:.3f} m')
    finally:
        if handle is not None and handle.status not in (4, 5, 6):
            node.future(handle.cancel_goal_async(), 5)
        node.publisher.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
