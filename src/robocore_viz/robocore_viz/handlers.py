import json
import math
import socket
import time
import tornado
import tornado.web
import tornado.websocket
import traceback
import types
import uuid

from . import __version__

def json_safe(value):
    """Represent unavailable ROS float measurements as JSON null, not NaN/Infinity."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value

class NoCacheStaticFileHandler(tornado.web.StaticFileHandler):
    def set_extra_headers(self, path):
        # Disable cache
        self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')

class ParametersHandler(tornado.web.RequestHandler):
    async def get(self):
        import asyncio
        from robocore_viz import rospy2
        from rcl_interfaces.srv import ListParameters, GetParameters
        node = rospy2._node
        target = self.get_argument("node")
        known = {(namespace.rstrip("/") + "/" + name)
                 for name, namespace in node.get_node_names_and_namespaces()}
        if target not in known:
            raise tornado.web.HTTPError(404, reason="Node is no longer available")
        clients = []
        async def request(service_type, suffix, request):
            client = node.create_client(service_type, target + "/" + suffix)
            clients.append(client)
            deadline = asyncio.get_running_loop().time() + 5
            while not client.service_is_ready():
                if asyncio.get_running_loop().time() > deadline:
                    raise tornado.web.HTTPError(504, reason="Parameter service unavailable")
                await asyncio.sleep(.05)
            future = client.call_async(request)
            while not future.done():
                if asyncio.get_running_loop().time() > deadline:
                    future.cancel()
                    raise tornado.web.HTTPError(504, reason="Parameter request timed out")
                await asyncio.sleep(.05)
            return future.result()
        try:
            listing = await request(ListParameters, "list_parameters", ListParameters.Request())
            names = sorted(listing.result.names)
            req = GetParameters.Request()
            req.names = names
            values = (await request(GetParameters, "get_parameters", req)).values
            fields = {1: "bool_value", 2: "integer_value", 3: "double_value",
                      4: "string_value", 5: "byte_array_value", 6: "bool_array_value",
                      7: "integer_array_value", 8: "double_array_value", 9: "string_array_value"}
            parameters = []
            for name, value in zip(names, values):
                data = getattr(value, fields[value.type]) if value.type in fields else None
                if value.type >= 5:
                    data = list(data)
                parameters.append({"name": name, "type": int(value.type), "value": json_safe(data)})
            self.set_header('Cache-Control', 'no-store')
            self.write({"node": target, "parameters": parameters})
        finally:
            for client in clients:
                node.destroy_client(client)

class GraphHandler(tornado.web.RequestHandler):
    def initialize(self, node):
        self.node = node

    def get(self):
        from robocore_viz import rospy2
        from rclpy.action import get_action_names_and_types
        node = rospy2._node
        self.set_header('Cache-Control', 'no-store')
        nodes = []
        for name, namespace in sorted(set(node.get_node_names_and_namespaces())):
            try:
                nodes.append({"name": name, "namespace": namespace,
                    "publishers": node.get_publisher_names_and_types_by_node(name, namespace),
                    "subscribers": node.get_subscriber_names_and_types_by_node(name, namespace),
                    "services": node.get_service_names_and_types_by_node(name, namespace)})
            except Exception:
                # A node may disappear during discovery; the next refresh recovers it.
                continue
        self.write({"nodes": nodes, "services": node.get_service_names_and_types(),
                    "actions": get_action_names_and_types(node)})

class JointControlConfigHandler(tornado.web.RequestHandler):
    def initialize(self, node):
        self.node = node
    def get(self):
        self.set_header('Cache-Control', 'no-store')
        self.write(self.node.joint_control_config)

class RobocoreVizSocketHandler(tornado.websocket.WebSocketHandler):
    sockets = set()

    def initialize(self, node):
        # store the instance of the ROS node that created this WebSocketHandler so we can access it later
        self.node = node

    def get_compression_options(self):
        # Non-None enables compression with default options.
        return {}

    def open(self):
        self.id = uuid.uuid4()    # unique socket id
        self.latency = 0          # latency measurement
        self.last_ping_times = [0] * 1024
        self.ping_seq = 0

        self.set_nodelay(True)

        # polyfill of is_closing() method for older versions of tornado
        if not hasattr(self.ws_connection, "is_closing"):
            self.ws_connection.is_closing = types.MethodType(
                lambda self_: self_.stream.closed() or self_.client_terminated or self_.server_terminated,
                self.ws_connection
            )

        self.update_intervals_by_topic = {}  # this socket's throttle rate on each topic
        self.last_data_times_by_topic = {}   # last time this socket received data on each topic

        RobocoreVizSocketHandler.sockets.add(self)

        self.write_message(json.dumps([RobocoreVizSocketHandler.MSG_SYSTEM, {
            "hostname": self.node.title,
            "version": __version__,
        }], separators=(',', ':')))

    def on_close(self):
        RobocoreVizSocketHandler.sockets.remove(self)

        # when socket closes, remove ourselves from all subscriptions
        for topic_name in self.node.remote_subs:
            if self.id in self.node.remote_subs[topic_name]:
                self.node.remote_subs[topic_name].remove(self.id)

    @classmethod
    def send_pings(cls):
        """
        Send pings to all sockets. When pongs are received they will be used for measuring
        latency and clock differences.
        """

        for socket in cls.sockets:
            try:
                socket.last_ping_times[socket.ping_seq % 1024] = time.time() * 1000
                if socket.ws_connection and not socket.ws_connection.is_closing():
                    socket.write_message(json.dumps([RobocoreVizSocketHandler.MSG_PING, {
                        RobocoreVizSocketHandler.PING_SEQ: socket.ping_seq,
                    }], separators=(',', ':')))
                socket.ping_seq += 1
            except Exception as e:
                print("Error sending message: %s" % str(e))

    @classmethod
    def broadcast(cls, message):
        """
        Broadcasts a dict-ified ROS message (message) to all sockets that care about that topic.
        The dict message should contain metadata about what topic it was
        being sent on: message["_topic_name"], message["_topic_type"].
        """

        try:
            if message[0] == RobocoreVizSocketHandler.MSG_TOPICS:
                json_msg = json.dumps(json_safe(message), separators=(',', ':'), allow_nan=False)
                for socket in cls.sockets:
                    if socket.ws_connection and not socket.ws_connection.is_closing():
                        socket.write_message(json_msg)
            elif message[0] == RobocoreVizSocketHandler.MSG_MSG:
                topic_name = message[1]["_topic_name"]
                json_msg = None
                for socket in cls.sockets:
                    if topic_name not in socket.node.remote_subs:
                        continue
                    if socket.id not in socket.node.remote_subs[topic_name]:
                        continue
                    t = time.time()
                    if t - socket.last_data_times_by_topic.get(topic_name, 0.0) < \
                            socket.update_intervals_by_topic.get(topic_name) - 2e-4:
                        continue
                    if socket.ws_connection and not socket.ws_connection.is_closing():
                        if json_msg is None:
                            json_msg = json.dumps(json_safe(message), separators=(',', ':'), allow_nan=False)
                        socket.write_message(json_msg)
                    socket.last_data_times_by_topic[topic_name] = t
        except Exception as e:
            print("Error sending message: %s" % str(e))
            traceback.print_exc()

    def on_message(self, message):
        """
        Message received from the client.
        """

        if self.ws_connection.is_closing():
            return

        # JSON decode it, give up if it isn't valid JSON
        try:
            argv = json.loads(message)
        except (ValueError, TypeError):
            print("error: bad: %s" % message)
            return

        # make sure the received argv is a list and the first element is a string and indicates the type of command
        if type(argv) is not list or len(argv) < 1 or type(argv[0]) is not str:
            print("error: bad: %s" % message)
            return

        # if we got a pong for our own ping, compute latency and clock difference
        elif argv[0] == RobocoreVizSocketHandler.MSG_PONG:
            if len(argv) != 2 or type(argv[1]) is not dict:
                print("error: pong: bad: %s" % message)
                return

            received_pong_time = time.time() * 1000
            self.latency = (received_pong_time - self.last_ping_times[argv[1].get(RobocoreVizSocketHandler.PONG_SEQ, 0) % 1024]) / 2
            if self.latency > 1000.0:
                self.node.logwarn("socket %s has high latency of %.2f ms" % (str(self.id), self.latency))
            
            if self.latency > 10000.0:
                self.node.logerr("socket %s has excessive latency of %.2f ms; closing connection" % (str(self.id), self.latency))
                self.close()

        # client wants to subscribe to topic
        elif argv[0] == RobocoreVizSocketHandler.MSG_SUB:
            if len(argv) != 2 or type(argv[1]) is not dict:
                print("error: sub: bad: %s" % message)
                return

            topic_name = argv[1].get("topicName")
            max_update_rate = float(argv[1].get("maxUpdateRate", 24.0))

            self.update_intervals_by_topic[topic_name] = 1.0 / max_update_rate
            self.node.update_intervals_by_topic[topic_name] = min(
                self.node.update_intervals_by_topic.get(topic_name, 1.),
                self.update_intervals_by_topic[topic_name]
            )

            if topic_name is None:
                print("error: no topic specified")
                return

            if topic_name not in self.node.remote_subs:
                self.node.remote_subs[topic_name] = set()

            self.node.remote_subs[topic_name].add(self.id)
            self.node.sync_subs()
            retained = self.node.retained_messages.get(topic_name)
            if retained is not None:
                self.write_message(json.dumps(json_safe([self.MSG_MSG, retained]),
                                              separators=(",", ":"), allow_nan=False))

        # client wants to unsubscribe from topic
        elif argv[0] == RobocoreVizSocketHandler.MSG_UNSUB:
            if len(argv) != 2 or type(argv[1]) is not dict:
                print("error: unsub: bad: %s" % message)
                return
            topic_name = argv[1].get("topicName")

            if topic_name not in self.node.remote_subs:
                self.node.remote_subs[topic_name] = set()

            try:
                self.node.remote_subs[topic_name].remove(self.id)
            except KeyError:
                print("KeyError trying to remove sub")

        # client wants to publish a message (e.g. teleop Twist).
        # robocore_viz normally only streams ROS -> browser; this opens the reverse
        # direction. The set of acceptable topic types is gated by the node's
        # publish_allowlist to prevent a stray client from publishing to e.g.
        # /rosout. Default allowlist is just geometry_msgs/msg/Twist.
        elif argv[0] == RobocoreVizSocketHandler.MSG_PUB:
            if len(argv) != 2 or type(argv[1]) is not dict:
                print("error: pub: bad: %s" % message)
                return
            topic_name = argv[1].get("topicName")
            topic_type = argv[1].get("topicType")
            msg_dict = argv[1].get("msg")
            if not (topic_name and topic_type and isinstance(msg_dict, dict)):
                print("error: pub: missing fields: %s" % message)
                return
            self.node.publish_from_client(topic_name, topic_type, msg_dict)

RobocoreVizSocketHandler.MSG_PING = "p";
RobocoreVizSocketHandler.MSG_PONG = "q";
RobocoreVizSocketHandler.MSG_MSG = "m";
RobocoreVizSocketHandler.MSG_TOPICS = "t";
RobocoreVizSocketHandler.MSG_SUB = "s";
RobocoreVizSocketHandler.MSG_SYSTEM = "y";
RobocoreVizSocketHandler.MSG_UNSUB = "u";
RobocoreVizSocketHandler.MSG_PUB = "P";

RobocoreVizSocketHandler.PING_SEQ = "s";
RobocoreVizSocketHandler.PONG_SEQ = "s";
RobocoreVizSocketHandler.PONG_TIME = "t";

class DashboardConfigHandler(tornado.web.RequestHandler):
    def initialize(self, node, javascript=False):
        self.node, self.javascript = node, javascript

    def get(self):
        self.set_header('Cache-Control', 'no-store')
        if self.javascript:
            self.set_header('Content-Type', 'application/javascript')
            self.write('window.ROBOCORE_CONFIG = ' + json.dumps(self.node.dashboard_config, allow_nan=False) + ';')
        else:
            self.write(self.node.dashboard_config)
