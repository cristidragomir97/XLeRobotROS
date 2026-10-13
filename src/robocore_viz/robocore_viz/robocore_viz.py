#!/usr/bin/env python3

import asyncio
import importlib
import json
import math
import os
import socket
import threading
import time
import tornado, tornado.web, tornado.websocket
import traceback

if os.environ.get("ROS_VERSION") == "1":
    import rospy # ROS1
elif os.environ.get("ROS_VERSION") == "2":
    import robocore_viz.rospy2 as rospy # ROS2
    from rclpy.qos import HistoryPolicy, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
else:
    print("ROS not detected. Please source your ROS environment\n(e.g. 'source /opt/ros/DISTRO/setup.bash')")
    exit(1)

from rosgraph_msgs.msg import Log

from robocore_viz.configuration import load_profile
from robocore_viz.serialization import ros2dict
from robocore_viz.subscribers.dmesg_subscriber import DMesgSubscriber
from robocore_viz.subscribers.processes_subscriber import ProcessesSubscriber
from robocore_viz.subscribers.system_stats_subscriber import SystemStatsSubscriber
from robocore_viz.subscribers.dummy_subscriber import DummySubscriber
from robocore_viz.handlers import RobocoreVizSocketHandler, NoCacheStaticFileHandler, JointControlConfigHandler, GraphHandler, ParametersHandler, DashboardConfigHandler

class RobocoreVizNode(object):
    instance = None
    def __init__(self, node_name = "robocore_viz_node"):
        self.__class__.instance = self
        rospy.init_node(node_name)
        self.port = rospy.get_param("~port", 8888)
        self.title = rospy.get_param("~title", socket.gethostname())
        self.dashboard_config = load_profile(rospy.get_param("~config_file", ""),
                                             rospy.get_param("~robot_description_file", ""))
        self.title = self.dashboard_config['name']
        self.joint_control_config = json.loads(rospy.get_param("~joint_control_config", "{}"))

        if self.dashboard_config['joint_groups']:
            self.joint_control_config = {'groups': self.dashboard_config['joint_groups']}
        self.command_topics = {self.dashboard_config['drive']['topic']: self.dashboard_config['drive']['type']}
        self.command_topics.update({g['topic']: g['type'] for g in self.joint_control_config.get('groups', [])})
        self._action_clients = {}
        self._active_actions = set()

        # types that the browser is allowed to publish via MSG_PUB. Both Twist
        # variants are accepted by default so the teleop card can match
        # whichever subscription type twist_mux exposes:
        #
        #   - twist_mux with use_stamped:=true  → subscribes /cmd_vel_joy as
        #     TwistStamped (verified empirically on Jazzy; the headers are
        #     misleading because there are separate handle classes per type).
        #   - twist_mux with use_stamped:=false → subscribes as plain Twist.
        #
        # The teleop card's topicType (set in index.js) decides which is sent.
        self.publish_allowlist = set(rospy.get_param(
            "~publish_allowlist",
            ["geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped",
             # joint-group position commands from the Joint sliders card.
             # Deliberately NOT covered by the zero-on-silence watchdog:
             # position commands must hold, not reset (see zero_payloads).
             "std_msgs/msg/Float64MultiArray", "trajectory_msgs/msg/JointTrajectory",
             "control_msgs/action/FollowJointTrajectory"],
        ))
        # if no client message arrives within this many seconds on a topic the
        # browser has published to, automatically republish a zero message
        # (so the robot stops if the page closes / wifi drops / tab is hidden).
        self.publish_watchdog_timeout = float(
            rospy.get_param("~publish_watchdog_timeout", 0.5)
        )

        # lazy publisher pool: topic_name -> rospy.Publisher
        self._client_publishers = {}
        # remember the normalised type per topic so the watchdog can build the
        # right zero payload (Twist vs TwistStamped have different shapes).
        self._client_publisher_types = {}
        # last-publish wall time per topic, for the watchdog
        self._client_publish_last_time = {}
        # last *non-zero* publish wall time per topic; once a zero has been sent
        # we won't re-publish more zeros until a non-zero command arrives.
        self._client_publish_last_nonzero = {}
        self._client_publishers_lock = threading.Lock()

        # desired subscriptions of all the websockets connecting to this instance.
        # these remote subs are updated directly by "friend" class RobocoreVizSocketHandler.
        # this class will read them and create actual ROS subscribers accordingly.
        # dict of topic_name -> set of sockets
        self.remote_subs = {}

        # actual ROS subscribers.
        # dict of topic_name -> ROS Subscriber
        self.local_subs = {}
        # Replay durable ROS data (maps, static TF) to later browser clients.
        self.retained_topics = set()
        self.retained_messages = {}

        # minimum update interval per topic (throttle rate) amang all subscribers to a particular topic.
        # we can throw data away if it arrives faster than this
        # dict of topic_name -> float (interval in seconds)
        self.update_intervals_by_topic = {}

        # last time data arrived for a particular topic
        # dict of topic_name -> float (time in seconds)
        self.last_data_times_by_topic = {}

        if rospy.__name__.endswith("rospy2"):
            # ros2 hack: need to subscribe to at least 1 topic
            # before dynamic subscribing will work later.
            # ros2 docs don't explain why but we need this magic.
            self.sub_rosout = rospy.Subscriber("/rosout", Log, lambda x:x)

        tornado_settings = {
            'debug': True,
            'static_path': os.path.join(os.path.dirname(os.path.realpath(__file__)), 'html')
        }

        tornado_handlers = [
                (r"/robocore_viz/config", DashboardConfigHandler, {"node": self}),
                (r"/robocore_viz/config.js", DashboardConfigHandler, {"node": self, "javascript": True}),
                (r"/robocore_viz/parameters", ParametersHandler),
                (r"/robocore_viz/graph", GraphHandler, {"node": self}),
                (r"/robocore_viz/joint-controls", JointControlConfigHandler, {"node": self}),
                (r"/robocore_viz/v1", RobocoreVizSocketHandler, {
                    "node": self,
                }),
                (r"/(.*)", NoCacheStaticFileHandler, {
                    "path": tornado_settings.get("static_path"),
                    "default_filename": "index.html"
                }),
        ]

        self.lock = threading.Lock()
        self.event_loop = None
        self.tornado_application = tornado.web.Application(tornado_handlers, **tornado_settings)
        asyncio.set_event_loop(asyncio.new_event_loop())
        self.event_loop = tornado.ioloop.IOLoop()
        self.tornado_application.listen(self.port)

        self._pose_stream = None
        if rospy.__name__.endswith('rospy2'):
            from robocore_viz.tf_pose import TfPoseStream
            self._pose_stream = TfPoseStream(rospy._node, self.dashboard_config['frames'],
                lambda: bool(self.remote_subs.get('_robot_pose')),
                lambda pose: self.on_ros_msg(pose, ('_robot_pose', 'geometry_msgs/msg/PoseStamped')))

        # allows tornado to log errors to ROS
        self.logwarn = rospy.logwarn
        self.logerr = rospy.logerr

        # tornado event loop. all the web server and web socket stuff happens here
        threading.Thread(target = self.event_loop.start, daemon = True).start()

        # loop to sync remote (websocket) subs with local (ROS) subs
        threading.Thread(target = self.sync_subs_loop, daemon = True).start()

        # loop to keep track of latencies and clock differences for each socket
        threading.Thread(target = self.pingpong_loop, daemon = True).start()

        # watchdog for browser-published topics — zeroes them if the client
        # goes quiet (only meaningful when something has actually been published)
        threading.Thread(target = self._publish_watchdog_loop, daemon = True).start()

        rospy.loginfo("Robocore Viz listening on :%d" % self.port)

    def start(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass

    def get_msg_class(self, msg_type):
        """
        Given a ROS message type specified as a string, e.g.
            "std_msgs/Int32"
        or
            "std_msgs/msg/Int32"
        it imports the message class into Python and returns the class, i.e. the actual std_msgs.msg.Int32

        Returns none if the type is invalid (e.g. if user hasn't bash-sourced the message package).
        """
        try:
            msg_module, dummy, msg_class_name = msg_type.replace("/", ".").rpartition(".")
        except ValueError:
            rospy.logerr("invalid type %s" % msg_type)
            return None

        try:
            if not msg_module.endswith(".msg"):
                msg_module = msg_module + ".msg"
            return getattr(importlib.import_module(msg_module), msg_class_name)
        except Exception as e:
            rospy.logerr(str(e))
            return None

    if os.environ.get("ROS_VERSION") == "2":
        def get_topic_qos(self, topic_name: str) -> QoSProfile:
            """!
            Given a topic name, get the QoS profile with which it is being published
            @param topic_name (str) the topic name
            @return QosProfile the qos profile with which the topic is published. If no publishers exist
            for the given topic, it returns the sensor data QoS. returns None in case ROS1 is being used
            """
            if rospy.__name__.endswith("rospy2"):
                topic_info = rospy._node.get_publishers_info_by_topic(topic_name=topic_name)
                if len(topic_info):
                    if topic_info[0].qos_profile.history == HistoryPolicy.UNKNOWN:
                        topic_info[0].qos_profile.history = HistoryPolicy.KEEP_LAST
                    return topic_info[0].qos_profile
                else:
                    rospy.logwarn(f"No publishers available for topic {topic_name}. Returning sensor data QoS")
                    return QoSProfile(
                            depth=10,
                            reliability=QoSReliabilityPolicy.BEST_EFFORT,
                            # reliability=QoSReliabilityPolicy.RELIABLE,
                            durability=QoSDurabilityPolicy.VOLATILE,
                            # durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                        )
            else:
                rospy.logwarn("QoS profiles are only used in ROS2")
                return None

    def pingpong_loop(self):
        """
        Loop to send pings to all active sockets every 5 seconds.
        """
        while True:
            time.sleep(5)

            if self.event_loop is None:
                continue
            try:
                self.event_loop.add_callback(RobocoreVizSocketHandler.send_pings)
            except Exception as e:
                rospy.logwarn(str(e))
                traceback.print_exc()

    def sync_subs_loop(self):
        """
        Periodically calls self.sync_subs(). Intended to be run in a thread.
        """
        while True:
            time.sleep(1)
            self.sync_subs()

    def sync_subs(self):
        """
        Looks at self.remote_subs and makes sure local subscribers exist to match them.
        Also cleans up unused local subscribers for which there are no remote subs interested in them.
        """

        # Acquire lock since either sync_subs_loop or websocket may call this function (from different threads)
        self.lock.acquire()

        try:
            # all topics and their types as strings e.g. {"/foo": "std_msgs/String", "/bar": "std_msgs/Int32"}
            self.all_topics = {}

            for topic_tuple in rospy.get_published_topics():
                topic_name = topic_tuple[0]
                topic_type = topic_tuple[1]
                if type(topic_type) is list:
                    topic_type = topic_type[0] # ROS2
                self.all_topics[topic_name] = topic_type

            if self._pose_stream is not None:
                self.all_topics['_robot_pose'] = 'geometry_msgs/msg/PoseStamped'

            self.event_loop.add_callback(
                RobocoreVizSocketHandler.broadcast,
                [RobocoreVizSocketHandler.MSG_TOPICS, self.all_topics ]
            )

            for topic_name in self.remote_subs:
                if len(self.remote_subs[topic_name]) == 0:
                    continue

                if topic_name == '_robot_pose':
                    continue  # Produced from TF at a fixed rate, not a ROS topic subscription.

                # remote sub special (non-ros) topic: _dmesg
                # handle it separately here
                if topic_name == "_dmesg":
                    if topic_name not in self.local_subs:
                        rospy.loginfo("Subscribing to dmesg [non-ros]")
                        self.local_subs[topic_name] = DMesgSubscriber(self.on_dmesg)
                    continue

                if topic_name == "_system_stats":
                    if topic_name not in self.local_subs:
                        rospy.loginfo("Subscribing to _system_stats [non-ros]")
                        self.local_subs[topic_name] = SystemStatsSubscriber(self.on_system_stats)
                    continue

                if topic_name == "_top":
                    if topic_name not in self.local_subs:
                        rospy.loginfo("Subscribing to _top [non-ros]")
                        self.local_subs[topic_name] = ProcessesSubscriber(self.on_top)
                    continue

                # check if remote sub request is not actually a ROS topic before proceeding
                if topic_name not in self.all_topics:
                    rospy.logwarn("warning: topic %s not found" % topic_name)
                    continue

                # if the local subscriber doesn't exist for the remote sub, create it
                if topic_name not in self.local_subs:
                    topic_type = self.all_topics[topic_name]
                    msg_class = self.get_msg_class(topic_type)

                    if msg_class is None:
                        # invalid message type or custom message package not source-bashed
                        # put a dummy subscriber in to avoid returning to this again.
                        # user needs to re-run robocore_viz with the custom message files sourced.
                        self.local_subs[topic_name] = DummySubscriber()
                        self.event_loop.add_callback(
                            RobocoreVizSocketHandler.broadcast,
                            [
                                RobocoreVizSocketHandler.MSG_MSG,
                                {
                                    "_topic_name": topic_name, # special non-ros topics start with _
                                    "_topic_type": topic_type,
                                    "_error": "Could not load message type '%s'. Are the .msg files for it source-bashed?" % topic_type,
                                },
                            ]
                        )
                        continue

                    self.last_data_times_by_topic[topic_name] = 0.0

                    rospy.loginfo("Subscribing to %s" % topic_name)

                    kwargs = {}
                    if rospy.__name__.endswith("rospy2"):
                        # In ros2 we also can pass QoS parameters to the subscriber.
                        # To avoid incompatibilities we subscribe using the same Qos
                        # of the topic's publishers
                        kwargs = {"qos": self.get_topic_qos(topic_name)}
                        if kwargs["qos"].durability == QoSDurabilityPolicy.TRANSIENT_LOCAL:
                            self.retained_topics.add(topic_name)
                    self.local_subs[topic_name] = rospy.Subscriber(
                        topic_name,
                        self.get_msg_class(topic_type),
                        self.on_ros_msg,
                        callback_args = (topic_name, topic_type),
                        **kwargs
                    )

            # clean up local subscribers for which remote clients have lost interest
            for topic_name in list(self.local_subs.keys()):
                if topic_name not in self.remote_subs or \
                    len(self.remote_subs[topic_name]) == 0:
                        rospy.loginfo("Unsubscribing from %s" % topic_name)
                        self.local_subs[topic_name].unregister()
                        del(self.local_subs[topic_name])
                        self.retained_topics.discard(topic_name)
                        self.retained_messages.pop(topic_name, None)

        except Exception as e:
            rospy.logwarn(str(e))
            traceback.print_exc()

        self.lock.release()

    def on_system_stats(self, system_stats):
        """
        system stats received. send it off to the client as a "fake" ROS message (which could at some point be a real ROS message)
        """
        if self.event_loop is None:
            return

        msg_dict = {
            "_topic_name": "_system_stats", # special non-ros topics start with _
            "_topic_type": "robocore_viz_msgs/msg/SystemStats",
        }

        for key, value in system_stats.items():
            msg_dict[key] = value

        self.event_loop.add_callback(
            RobocoreVizSocketHandler.broadcast,
            [
                RobocoreVizSocketHandler.MSG_MSG,
                msg_dict
            ]
        )

    def on_top(self, processes):
        """
        processes list received. send it off to the client as a "fake" ROS message (which could at some point be a real ROS message)
        """
        if self.event_loop is None:
            return

        self.event_loop.add_callback(
            RobocoreVizSocketHandler.broadcast,
            [
                RobocoreVizSocketHandler.MSG_MSG,
                {
                    "_topic_name": "_top", # special non-ros topics start with _
                    "_topic_type": "robocore_viz_msgs/msg/ProcessList",
                    "processes": processes,
                },
            ]
        )

    def on_dmesg(self, text):
        """
        dmesg log received. make it look like a rcl_interfaces/msg/Log and send it off
        """
        if self.event_loop is None:
            return

        self.event_loop.add_callback(
            RobocoreVizSocketHandler.broadcast,
            [
                RobocoreVizSocketHandler.MSG_MSG,
                {
                    "_topic_name": "_dmesg", # special non-ros topics start with _
                    "_topic_type": "rcl_interfaces/msg/Log",
                    "msg": text,
                },
            ]
        )

    def on_ros_msg(self, msg, topic_info):
        """
        ROS messaged received (any topic or type).
        """
        topic_name, topic_type = topic_info
        if self._pose_stream is not None and topic_name == self.dashboard_config['topics']['odom']:
            self._pose_stream.odometry_frame(getattr(msg, 'child_frame_id', ''))
        interval = self.update_intervals_by_topic.get(topic_name)
        if interval is None:
            return
        t = time.time()
        if t - self.last_data_times_by_topic.get(topic_name, 0) < interval - 1e-4:
            return

        if self.event_loop is None:
            return

        # convert ROS message into a dict and get it ready for serialization
        ros_msg_dict = ros2dict(msg)

        # add metadata
        ros_msg_dict["_topic_name"] = topic_name
        ros_msg_dict["_topic_type"] = topic_type
        ros_msg_dict["_time"] = time.time() * 1000
        if rospy.__name__.endswith("rospy2"):
            ros_msg_dict["_ros_time"] = rospy._node.get_clock().now().nanoseconds * 1e-9

        # log last time we received data on this topic
        self.last_data_times_by_topic[topic_name] = t

        if topic_name in self.retained_topics:
            self.retained_messages[topic_name] = ros_msg_dict

        # broadcast it to the listeners that care
        self.event_loop.add_callback(
            RobocoreVizSocketHandler.broadcast,
            [RobocoreVizSocketHandler.MSG_MSG, ros_msg_dict]
        )

    # --- browser-to-ROS publishing (used by the teleop viewer card) ---------
    #
    # Browser sends ['P', {topicName, topicType, msg}] over the WS. The msg
    # dict is the ros2dict shape (nested {linear:{x,y,z}, angular:{x,y,z}} for
    # a Twist). We lazily create a publisher per topic, validate the type
    # against publish_allowlist, then construct + publish the ROS message.

    def publish_from_client(self, topic_name, topic_type, msg_dict):
        # normalise both "pkg/Type" and "pkg/msg/Type" to "pkg/msg/Type"
        norm = topic_type
        if norm.count("/") == 1:
            pkg, t = norm.split("/")
            norm = f"{pkg}/msg/{t}"
        if norm not in self.publish_allowlist:
            rospy.logwarn(
                "publish refused: %s not in publish_allowlist (%s)"
                % (norm, sorted(self.publish_allowlist))
            )
            return

        if self.command_topics.get(topic_name) != norm:
            rospy.logwarn("publish refused: topic/type is not configured for this robot")
            return

        if norm in ("geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped"):
            twist = msg_dict.get('twist', {}) if norm.endswith('TwistStamped') else msg_dict
            linear, angular = twist.get('linear', {}), twist.get('angular', {})
            values = [float(linear.get('x', 0)), float(linear.get('y', 0)), float(angular.get('z', 0))]
            if not all(math.isfinite(v) for v in values):
                rospy.logwarn("publish refused: non-finite drive command")
                return
            limits = self.dashboard_config['drive']
            cap = lambda value, maximum: max(-maximum, min(maximum, value))
            twist['linear'] = {'x': cap(values[0], limits['linear_max']),
                               'y': cap(values[1], limits['linear_max']) if limits['omnidirectional'] else 0., 'z': 0.}
            twist['angular'] = {'x': 0., 'y': 0., 'z': cap(values[2], limits['angular_max'])}

        if norm in ("trajectory_msgs/msg/JointTrajectory", "control_msgs/action/FollowJointTrajectory"):
            group = next((g for g in self.joint_control_config.get("groups", []) if g["topic"] == topic_name), None)
            if not group or msg_dict.get("joint_names") != [j["name"] for j in group["joints"]]:
                rospy.logwarn("publish refused: undeclared trajectory controller or joint order")
                return
            for point in msg_dict.get("points", []):
                positions = point.get("positions", [])
                if len(positions) != len(group["joints"]) or any(not math.isfinite(float(v)) or not j["min"] <= float(v) <= j["max"] for v, j in zip(positions, group["joints"])):
                    rospy.logwarn("publish refused: joint command outside declared limits")
                    return
        if norm == "control_msgs/action/FollowJointTrajectory":
            self._send_trajectory_action(topic_name, msg_dict)
            return
        ros_msg = self._dict_to_ros(norm, msg_dict)
        if ros_msg is None:
            return

        with self._client_publishers_lock:
            pub = self._client_publishers.get(topic_name)
            cached_type = self._client_publisher_types.get(topic_name)

            # If we already created a publisher for this topic with a different
            # type (e.g. stale client cached the old code path), bail with a
            # clear message rather than try a second mismatched create.
            if pub is not None and cached_type != norm:
                rospy.logwarn(
                    "publish refused: %s already has cached publisher of type "
                    "%s; client asked for %s. Restart robocore_viz if you want to "
                    "change the publisher type." % (topic_name, cached_type, norm)
                )
                return

            if pub is None:
                msg_class = self.get_msg_class(norm)
                if msg_class is None:
                    return

                # Pre-check the ROS graph — if anything else is already on this
                # topic with a different type, create_publisher() will raise an
                # RCLError that takes the WS handler down. Detect first.
                if rospy.__name__.endswith("rospy2"):
                    try:
                        existing = (
                            rospy._node.get_publishers_info_by_topic(topic_name)
                            + rospy._node.get_subscriptions_info_by_topic(topic_name)
                        )
                    except Exception:
                        existing = []
                    for info in existing:
                        et = info.topic_type
                        if et and et != norm:
                            rospy.logwarn(
                                "publish refused: %s is already registered as "
                                "%s in the ROS graph (vs requested %s). Make "
                                "the client publish the matching type, or "
                                "remap." % (topic_name, et, norm)
                            )
                            return

                rospy.loginfo("Browser publishing on %s (%s)" % (topic_name, norm))
                try:
                    pub = rospy.Publisher(topic_name, msg_class, queue_size=10)
                except Exception as e:
                    rospy.logwarn(
                        "publish create failed on %s: %s" % (topic_name, str(e))
                    )
                    return
                self._client_publishers[topic_name] = pub
                self._client_publisher_types[topic_name] = norm

        try:
            pub.publish(ros_msg)
            now = time.time()
            self._client_publish_last_time[topic_name] = now
            if self._is_nonzero(norm, msg_dict):
                self._client_publish_last_nonzero[topic_name] = now
        except Exception as e:
            rospy.logwarn("publish failed on %s: %s" % (topic_name, str(e)))

    def _dict_to_ros(self, norm_type, msg_dict):
        """Build a ROS msg from a ros2dict-shaped dict.
        Only the allowlisted types need a converter here; add more as you
        extend publish_allowlist.
        """
        if norm_type == "geometry_msgs/msg/Twist":
            from geometry_msgs.msg import Twist
            t = Twist()
            lin = (msg_dict.get("linear") or {})
            ang = (msg_dict.get("angular") or {})
            t.linear.x = float(lin.get("x", 0.0) or 0.0)
            t.linear.y = float(lin.get("y", 0.0) or 0.0)
            t.linear.z = float(lin.get("z", 0.0) or 0.0)
            t.angular.x = float(ang.get("x", 0.0) or 0.0)
            t.angular.y = float(ang.get("y", 0.0) or 0.0)
            t.angular.z = float(ang.get("z", 0.0) or 0.0)
            return t

        if norm_type == "geometry_msgs/msg/TwistStamped":
            from geometry_msgs.msg import TwistStamped
            ts = TwistStamped()
            # Browsers can't get a sync'd ROS clock, so stamp on receipt — the
            # browser's `header.stamp` field (if any) is ignored. frame_id
            # carried through if the client supplied one.
            ts.header.stamp = rospy._clock.now().to_msg() \
                if hasattr(rospy, "_clock") and rospy._clock is not None \
                else ts.header.stamp
            header = (msg_dict.get("header") or {})
            ts.header.frame_id = str(header.get("frame_id", "") or "")
            twist_dict = msg_dict.get("twist") or {}
            lin = (twist_dict.get("linear") or {})
            ang = (twist_dict.get("angular") or {})
            ts.twist.linear.x = float(lin.get("x", 0.0) or 0.0)
            ts.twist.linear.y = float(lin.get("y", 0.0) or 0.0)
            ts.twist.linear.z = float(lin.get("z", 0.0) or 0.0)
            ts.twist.angular.x = float(ang.get("x", 0.0) or 0.0)
            ts.twist.angular.y = float(ang.get("y", 0.0) or 0.0)
            ts.twist.angular.z = float(ang.get("z", 0.0) or 0.0)
            return ts

        if norm_type == "trajectory_msgs/msg/JointTrajectory":
            from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
            trajectory = JointTrajectory()
            trajectory.joint_names = [str(n) for n in msg_dict.get("joint_names", [])]
            for data in msg_dict.get("points", []):
                point = JointTrajectoryPoint()
                point.positions = [float(v) for v in data.get("positions", [])]
                if len(point.positions) != len(trajectory.joint_names) or not all(math.isfinite(v) for v in point.positions):
                    return None
                duration = data.get("time_from_start", {})
                point.time_from_start.sec = int(duration.get("sec", 0))
                point.time_from_start.nanosec = int(duration.get("nanosec", 0))
                trajectory.points.append(point)
            return trajectory if trajectory.points else None

        if norm_type == "std_msgs/msg/Float64MultiArray":
            from std_msgs.msg import Float64MultiArray
            m = Float64MultiArray()
            m.data = [float(v) for v in (msg_dict.get("data") or [])]
            return m

        rospy.logwarn("no dict->ros converter registered for %s" % norm_type)
        return None

    def _send_trajectory_action(self, topic, payload):
        """Use the robot's existing action controller rather than inventing command topics."""
        from rclpy.action import ActionClient
        from control_msgs.action import FollowJointTrajectory
        if topic in self._active_actions:
            rospy.logwarn("Trajectory controller is busy; wait for motion to complete")
            return
        client = self._action_clients.get(topic)
        if client is None:
            client = self._action_clients[topic] = ActionClient(rospy._node, FollowJointTrajectory, topic)
        if not client.server_is_ready():
            rospy.logwarn("Trajectory action server is not ready: " + topic)
            return
        trajectory = self._dict_to_ros("trajectory_msgs/msg/JointTrajectory", payload)
        if trajectory is None:
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        self._active_actions.add(topic)
        def accepted(future):
            try:
                handle = future.result()
                if not handle.accepted:
                    self._active_actions.discard(topic)
                    rospy.logwarn("Trajectory rejected: " + topic)
                    return
                def completed(result):
                    self._active_actions.discard(topic)
                    try:
                        response = result.result().result
                        if response.error_code:
                            rospy.logwarn("Trajectory failed: " + response.error_string)
                    except Exception as error:
                        rospy.logwarn(str(error))
                handle.get_result_async().add_done_callback(completed)
            except Exception as error:
                self._active_actions.discard(topic)
                rospy.logwarn(str(error))
        client.send_goal_async(goal).add_done_callback(accepted)

    def _is_nonzero(self, norm_type, msg_dict):
        twist_dict = msg_dict
        if norm_type == "geometry_msgs/msg/TwistStamped":
            twist_dict = msg_dict.get("twist") or {}
        if norm_type in ("geometry_msgs/msg/Twist",
                          "geometry_msgs/msg/TwistStamped"):
            lin = (twist_dict.get("linear") or {})
            ang = (twist_dict.get("angular") or {})
            for v in (lin.get("x"), lin.get("y"), lin.get("z"),
                      ang.get("x"), ang.get("y"), ang.get("z")):
                if v and float(v) != 0.0:
                    return True
            return False
        return True

    def _publish_watchdog_loop(self):
        zero_twist = {"linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                      "angular": {"x": 0.0, "y": 0.0, "z": 0.0}}
        zero_payloads = {
            "geometry_msgs/msg/Twist": zero_twist,
            "geometry_msgs/msg/TwistStamped": {"twist": zero_twist},
        }
        while True:
            time.sleep(0.1)
            now = time.time()
            stale = []
            with self._client_publishers_lock:
                topic_names = list(self._client_publishers.keys())
            for topic_name in topic_names:
                last = self._client_publish_last_time.get(topic_name, 0.0)
                if last == 0.0:
                    continue
                if (now - last) <= self.publish_watchdog_timeout:
                    continue
                last_nonzero = self._client_publish_last_nonzero.get(topic_name, 0.0)
                if last_nonzero <= last and last != last_nonzero:
                    # we already published the zero after the last non-zero;
                    # nothing more to do until a new command arrives.
                    continue
                stale.append(topic_name)
            for topic_name in stale:
                with self._client_publishers_lock:
                    pub = self._client_publishers.get(topic_name)
                    norm_type = self._client_publisher_types.get(topic_name)
                if pub is None or norm_type not in zero_payloads:
                    continue
                ros_msg = self._dict_to_ros(norm_type, zero_payloads[norm_type])
                if ros_msg is None:
                    continue
                try:
                    pub.publish(ros_msg)
                except Exception:
                    pass
                # mark so we don't keep republishing zeros forever.
                self._client_publish_last_time[topic_name] = now
                self._client_publish_last_nonzero[topic_name] = 0.0


def main(args=None):
    RobocoreVizNode().start()

if __name__ == '__main__':
    main()
