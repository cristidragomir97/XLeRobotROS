"""Current robot pose for browser maps; sparse localization messages are not motion."""
from geometry_msgs.msg import PoseStamped
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class TfPoseStream:
    def __init__(self, node, frames, wanted, emit):
        self.fixed_frame = frames['fixed']
        self.explicit_robot = bool(frames['robot'])
        self.robot_frame = frames['robot'] or 'base_link'
        self.wanted, self.emit = wanted, emit
        self.buffer = Buffer(node=node)
        self.listener = TransformListener(self.buffer, node, spin_thread=False)
        self.timer = node.create_timer(.1, self.update)

    def odometry_frame(self, frame):
        if frame and not self.explicit_robot:
            self.robot_frame = frame

    def update(self):
        if not self.wanted():
            return
        try:
            transform = self.buffer.lookup_transform(self.fixed_frame, self.robot_frame, Time())
        except TransformException:
            return  # Never replace a missing transform with an invented origin pose.
        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.emit(pose)
