"""Real ROS messages/DDS with an in-memory serial endpoint; no hardware motion."""

import struct
import time
import unittest
from unittest.mock import patch

try:
    import rclpy
    import serial
except ImportError:
    rclpy = None

from dcaron_bridge.protocol import Parser, frame, subscribe
from test_protocol import odom_packet, velpos_packet


class FakeSerial:
    def __init__(self, *args, **kwargs):
        self.rx = bytearray()
        self.tx = []
        self.closed = False
        self.fail_read = False

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, count):
        if self.fail_read:
            raise serial.SerialException('simulated disconnect')
        result = bytes(self.rx[:count])
        del self.rx[:count]
        return result

    def write(self, data):
        self.tx.append(data)
        return len(data)

    def close(self):
        self.closed = True

    def inject(self, packet, target=151, source=1):
        self.rx.extend(frame(packet.a, packet.b, packet.payload, target, source))


@unittest.skipIf(rclpy is None, 'ROS runtime unavailable; protocol tests still run')
class RosNodeTests(unittest.TestCase):
    def setUp(self):
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        self.context = Context()
        rclpy.init(context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.nodes = []
        self.ports = []

    def tearDown(self):
        self.executor.shutdown()
        for node in reversed(self.nodes):
            node.destroy_node()
        self.context.shutdown()

    def make_bridge(self, **overrides):
        from rclpy.parameter import Parameter
        from dcaron_bridge.node import DcaronBridge

        def factory(*args, **kwargs):
            port = FakeSerial(*args, **kwargs)
            self.ports.append(port)
            return port

        node = DcaronBridge(serial_factory=factory, context=self.context,
                            parameter_overrides=[Parameter(k, value=v) for k, v in overrides.items()])
        self.nodes.append(node)
        self.executor.add_node(node)
        node.tick()
        return node

    def spin_until(self, predicate, timeout=5):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
        self.assertTrue(predicate(), 'ROS discovery/delivery timed out')

    def test_real_ros_topics_and_command_to_wire(self):
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        bridge = self.make_bridge(telemetry='odom', enable_cmd_vel=True)
        peer = Node('test_subscriber', context=self.context)
        self.nodes.append(peer)
        self.executor.add_node(peer)
        odoms, imus = [], []
        odom_sub = peer.create_subscription(Odometry, 'odom', odoms.append, 10)
        imu_sub = peer.create_subscription(Imu, 'imu/data', imus.append, 10)
        commands = peer.create_publisher(Twist, 'cmd_vel', 10)
        self.spin_until(lambda: commands.get_subscription_count() == 1
                        and bridge.odom_pub.get_subscription_count() == 1
                        and bridge.imu_pub.get_subscription_count() == 1)
        self.ports[-1].inject(odom_packet())
        self.spin_until(lambda: len(odoms) > 0 and len(imus) > 0)
        self.assertAlmostEqual(odoms[-1].pose.pose.position.x, 123.456)
        self.assertAlmostEqual(odoms[-1].twist.twist.angular.z, 0.5)
        self.assertAlmostEqual(imus[-1].linear_acceleration.z, 9.81)
        self.assertEqual(imus[-1].orientation_covariance[0], -1.0)
        self.assertEqual(odoms[-1].header.frame_id, 'odom')
        self.assertGreater(odoms[-1].header.stamp.sec, 0)
        self.assertEqual(odoms[-1].header.stamp, imus[-1].header.stamp)
        command = Twist()
        command.linear.x = 0.3
        before = len(self.ports[-1].tx)
        commands.publish(command)
        self.spin_until(lambda: len(self.ports[-1].tx) > before)
        packet = Parser().feed(self.ports[-1].tx[-1])[0]
        self.assertEqual((packet.a, packet.b), (2, 0x62))
        self.assertEqual(struct.unpack('<3i', packet.payload), (3000, 0, 0))
        self.assertIsNotNone(odom_sub)
        self.assertIsNotNone(imu_sub)

    def test_read_only_default_and_address_filter(self):
        from geometry_msgs.msg import Twist
        node = self.make_bridge()
        port = self.ports[-1]
        self.assertEqual(port.tx, [subscribe('velpos')])
        self.assertIsNone(node.cmd_sub)
        self.assertIsNone(node.imu_pub)
        port.inject(velpos_packet(), source=2)
        node.tick()
        self.assertIsNone(node.last_rx)
        port.inject(velpos_packet())
        node.tick()
        self.assertIsNotNone(node.last_rx)
        node.on_command(Twist())
        self.assertEqual(len(port.tx), 1)

    def test_timeout_invalid_command_and_disconnect(self):
        from geometry_msgs.msg import Twist
        node = self.make_bridge(enable_cmd_vel=True)
        port = self.ports[-1]
        port.inject(velpos_packet())
        node.tick()
        command = Twist()
        command.linear.x = 0.2
        node.on_command(command)
        self.assertIsNotNone(node.watchdog.last)
        node.watchdog.last = time.monotonic() - 1
        node.tick()
        self.assertEqual(Parser().feed(port.tx[-1])[0].payload, bytes(12))
        command.linear.x = float('nan')
        node.on_command(command)
        self.assertEqual(Parser().feed(port.tx[-1])[0].payload, bytes(12))
        command.linear.x = 0.2
        node.on_command(command)
        port.fail_read = True
        node.tick()
        self.assertIsNone(node.port)
        self.assertIsNone(node.watchdog.last)
        self.assertTrue(port.closed)
        node.next_connect = 0
        node.tick()
        replacement = self.ports[-1]
        self.assertEqual(len(replacement.tx), 2)  # zero, then subscription; no replay
        self.assertEqual(Parser().feed(replacement.tx[0])[0].payload, bytes(12))
        before = len(replacement.tx)
        node.on_command(command)  # no fresh telemetry after reconnect
        self.assertEqual(len(replacement.tx), before)

    def test_stale_telemetry_and_partial_write(self):
        node = self.make_bridge(enable_cmd_vel=True)
        port = self.ports[-1]
        node.connected_at = time.monotonic() - 3
        node.tick()
        self.assertIsNone(node.port)
        self.assertEqual(Parser().feed(port.tx[-1])[0].payload, bytes(12))
        node.next_connect = 0
        node.tick()
        with patch.object(node.port, 'write', return_value=1):
            with self.assertRaises(serial.SerialException):
                node.write(b'123')

    def test_shutdown_zero_and_stop_subscription(self):
        node = self.make_bridge(enable_cmd_vel=True)
        port = self.ports[-1]
        self.executor.remove_node(node)
        self.nodes.remove(node)
        node.destroy_node()
        self.assertEqual(Parser().feed(port.tx[-2])[0].payload, bytes(12))
        self.assertEqual(port.tx[-1], subscribe('velpos', continuous=False))
        self.assertTrue(port.closed)

    def action_client(self):
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from dcaron_interfaces.action import Motion
        peer = Node('test_motion_client', context=self.context)
        self.nodes.append(peer)
        self.executor.add_node(peer)
        return ActionClient(peer, Motion, 'motion'), Motion

    def send_action(self, client, request):
        response = client.send_goal_async(request)
        self.spin_until(response.done)
        return response.result()

    def test_actions_all_wire_commands_and_results(self):
        node = self.make_bridge(enable_motion_actions=True)
        port = self.ports[-1]
        client, Motion = self.action_client()
        try:
            for command in (0x63, 0x64, 0x65, 0x66):
                port.inject(velpos_packet())
                node.tick()
                goal = Motion.Goal()
                goal.command, goal.x, goal.yaw = command, 0.1, 0.2
                goal.speed, goal.angular_speed, goal.radius = 0.1, 0.2, 0.5
                handle = self.send_action(client, goal)
                self.assertTrue(handle.accepted)
                self.spin_until(lambda: node.actions.active is not None and node.actions.active['sent'])
                self.assertEqual(Parser().feed(port.tx[-1])[0].b, command)
                result = handle.get_result_async()
                port.rx.extend(frame(0x6F, command, b'\x80\x00', 151, 1))
                node.tick()
                self.assertFalse(result.done())
                port.rx.extend(frame(0x6F, command, b'\xff\x00', 151, 1))
                self.spin_until(result.done)
                self.assertTrue(result.result().result.success)
                self.assertEqual(result.result().status, 4)
        finally:
            client.destroy()

    def test_action_cancel_old_reply_quarantine_and_rejection(self):
        node = self.make_bridge(enable_motion_actions=True)
        port = self.ports[-1]
        port.inject(velpos_packet())
        node.tick()
        client, Motion = self.action_client()
        try:
            goal = Motion.Goal()
            goal.command, goal.x, goal.speed = 0x64, 0.1, 0.1
            handle = self.send_action(client, goal)
            self.assertTrue(handle.accepted)
            self.spin_until(lambda: node.actions.active is not None and node.actions.active['sent'])
            busy = self.send_action(client, goal)
            self.assertFalse(busy.accepted)
            result = handle.get_result_async()
            cancel = handle.cancel_goal_async()
            self.spin_until(cancel.done)
            self.spin_until(result.done)
            self.assertEqual(result.result().status, 5)
            self.assertEqual(Parser().feed(port.tx[-1])[0].payload, bytes(12))
            old_pending = self.send_action(client, goal)
            self.assertFalse(old_pending.accepted)
            port.rx.extend(frame(0x6F, 0x64, b'\xff\x62', 151, 1))
            node.tick()
            next_handle = self.send_action(client, goal)
            self.assertTrue(next_handle.accepted)
            self.spin_until(lambda: node.actions.active is not None and node.actions.active['sent'])
            result = next_handle.get_result_async()
            port.rx.extend(frame(0x6F, 0x64, b'\xff\x01', 151, 1))
            self.spin_until(result.done)
            self.assertEqual(result.result().status, 6)
            self.assertEqual(result.result().result.notice, 1)
        finally:
            client.destroy()

    def test_action_deadline_and_reconnect_do_not_replay(self):
        node = self.make_bridge(enable_motion_actions=True)
        port = self.ports[-1]
        port.inject(velpos_packet())
        node.tick()
        client, Motion = self.action_client()
        try:
            goal = Motion.Goal()
            goal.command, goal.yaw, goal.angular_speed = 0x63, 0.2, 0.2
            handle = self.send_action(client, goal)
            self.spin_until(lambda: node.actions.active is not None and node.actions.active['sent'])
            result = handle.get_result_async()
            node.actions.active['deadline'] = time.monotonic() - 1
            self.spin_until(result.done)
            self.assertFalse(result.result().result.success)
            self.assertEqual(result.result().result.notice, 254)
            node.disconnect('test')
            node.next_connect = 0
            node.tick()
            self.assertIn(0x63, node.actions.quarantine.pending)
            self.assertEqual([p.b for data in self.ports[-1].tx for p in Parser().feed(data)], [0x62, 0x81])
        finally:
            client.destroy()

    def test_action_cross_next_and_extended_pose(self):
        node = self.make_bridge(enable_motion_actions=True)
        port = self.ports[-1]
        port.inject(velpos_packet())
        node.tick()
        client, Motion = self.action_client()
        try:
            goal = Motion.Goal()
            goal.command, goal.mode, goal.next_command = 0x81, 1, 0x65
            goal.x, goal.yaw, goal.speed = 0.1, 0.2, 0.1
            handle = self.send_action(client, goal)
            self.assertTrue(handle.accepted)
            self.spin_until(lambda: node.actions.active is not None and node.actions.active['sent'])
            self.assertEqual([p.b for p in Parser().feed(port.tx[-1])], [0x81, 0x65])
            result = handle.get_result_async()
            port.rx.extend(frame(0x6F, 0x65, b'\xff\x00', 151, 1))
            node.tick()
            self.assertFalse(result.done())
            payload = b'\xff\x00\x01' + struct.pack('<4i', 10000, -10000, 0, 2000)
            port.rx.extend(frame(0x6F, 0x81, payload, 151, 1))
            self.spin_until(result.done)
            self.assertTrue(result.result().result.pose_valid)
            self.assertEqual(result.result().result.pose.pose.position.x, 1.0)
        finally:
            client.destroy()
