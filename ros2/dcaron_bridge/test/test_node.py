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
