"""One serial owner, standard ROS topics. All IO runs in one executor thread."""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
import serial

from .protocol import Parser, CommandWatchdog, subscribe, velocity, frame, quaternion
from .telemetry import TelemetryReceiver
from std_msgs.msg import String
import json


class DcaronBridge(Node):
    def __init__(self, *, serial_factory=None, **kwargs):
        super().__init__("dcaron_bridge", **kwargs)
        defaults = {
            "port": "COM3", "baudrate": 460800, "robot_id": 1, "host_id": 151,
            "frequency": 10,
            "enable_cmd_vel": False, "cmd_timeout": 0.5,
            "max_linear_speed": 0.5, "max_angular_speed": 1.0,
            "telemetry_timeout": 2.0, "reconnect_interval": 3.0,
            "odom_frame": "odom", "base_frame": "base_link", "imu_frame": "base_link",
            "publish_tf": False,
            "motion_protocol": "queue", "enable_motion_actions": False, "motion_timeout": 30.0,
            "max_motion_timeout": 120.0, "max_displacement": 2.0, "max_rotation": 6.283185307179586,
        }
        self.cfg = {key: self.declare_parameter(
            key, value, ParameterDescriptor(read_only=True)).value
            for key, value in defaults.items()}
        c = self.cfg
        if c["motion_protocol"] not in ("queue", "legacy"): raise ValueError("motion_protocol must be queue or legacy")
        for key in ("cmd_timeout", "max_linear_speed", "max_angular_speed",
                    "telemetry_timeout", "reconnect_interval", "motion_timeout",
                    "max_motion_timeout", "max_displacement", "max_rotation"):
            if not math.isfinite(c[key]) or c[key] <= 0:
                raise ValueError(f"{key} must be positive and finite")
        for key in ("robot_id", "host_id"):
            if not 0 <= c[key] <= 255:
                raise ValueError(f"{key} must fit one byte")
        if c["robot_id"] == c["host_id"] or c["baudrate"] <= 0:
            raise ValueError("Use distinct device/host addresses and positive baudrate")
        if not c["port"] or any(not c[k] or c[k].startswith("/") for k in
                                ("odom_frame", "base_frame", "imu_frame")):
            raise ValueError("Nonempty port/frame names required; frames cannot start with /")
        if c["odom_frame"] == c["base_frame"]:
            raise ValueError("odom_frame and base_frame must differ")
        self.subscription_frames = [subscribe(name, c["frequency"], target=c["robot_id"], source=c["host_id"])
                                    for name in ('odom', 'velpos')]
        self.subscription_frames.append(frame(0x0B, 0x40, target=c['robot_id'], source=c['host_id']))
        self.pending_subscriptions = []
        self.next_subscription = 0.0
        self.telemetry = TelemetryReceiver()
        self.last_published = None
        self.last_diagnostic = 0.0
        self.rx_bytes = 0
        self.device_state = None
        self.parser = Parser()
        self.watchdog = CommandWatchdog(c["cmd_timeout"])
        self.serial_factory = serial_factory or serial.Serial
        self.serial_error = serial.SerialException
        self.port = None
        self.next_connect = 0.0
        self.last_rx = None
        self.stale_reported = False
        self.invalid_reported = False
        self.connected_at = 0.0
        # Reliable publishers also satisfy best-effort sensor subscribers.
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.imu_pub = self.create_publisher(Imu, "imu/data", 10)
        self.stream_pubs = {name: self.create_publisher(Odometry, 'telemetry/' + name, 10)
                            for name in ('odom', 'velpos')}
        self.diagnostics_pub = self.create_publisher(String, 'telemetry/status', 10)
        self.tf = TransformBroadcaster(self) if c["publish_tf"] else None
        # Keep only the newest command; never use transient-local motion commands.
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.cmd_sub = self.create_subscription(Twist, "cmd_vel", self.on_command, qos) if c["enable_cmd_vel"] else None
        self.actions = None
        if c["motion_timeout"] > c["max_motion_timeout"]:
            raise ValueError('motion_timeout must not exceed max_motion_timeout')
        if c['enable_motion_actions']:
            from .actions import MotionActions
            self.actions = MotionActions(self)
        self.timer = self.create_timer(0.01, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info(
            f"DFLink port={c['port']} baud={c['baudrate']} robot={c['robot_id']} "
            f"streams=odom+velpos frequency={c['frequency']}Hz "
            f"cmd_vel={c['enable_cmd_vel']} actions={c['enable_motion_actions']}")

    def write(self, data):
        if self.port is None:
            raise serial.SerialException("Port is disconnected")
        if self.port.write(data) != len(data):
            raise serial.SerialException("Incomplete DFLink write")

    def stop(self):
        self.write(velocity(0, 0, 0, self.cfg["robot_id"], self.cfg["host_id"]))
        self.watchdog.reset()

    def disconnect(self, error):
        if self.actions is not None:
            self.actions.interrupt(f'Serial failure: {error}', send_stop=False)
        self.get_logger().error(f"DFLink port={self.cfg['port']} failed: {error}")
        if self.port is not None:
            try:
                self.port.close()
            except (OSError, serial.SerialException):
                pass
        self.port = None
        self.parser = Parser()
        self.watchdog.reset()
        self.last_rx = None
        self.next_connect = time.monotonic() + self.cfg["reconnect_interval"]

    def on_command(self, msg):
        if not self.cfg["enable_cmd_vel"] or self.port is None:
            return
        # Telemetry is observational; serial availability and command limits govern sending.
        now = time.monotonic()
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        valid = (all(math.isfinite(v) for v in values)
                 and math.hypot(*values[:2]) <= self.cfg["max_linear_speed"]
                 and abs(values[2]) <= self.cfg["max_angular_speed"]
                 and msg.linear.z == 0.0 and msg.angular.x == 0.0 and msg.angular.y == 0.0)
        try:
            if self.actions is not None:
                self.actions.interrupt('Interrupted by cmd_vel', send_stop=False)
            if not valid:
                self.stop()
                if not self.invalid_reported:
                    self.get_logger().warning("Rejected nonfinite, unsupported-axis or out-of-limit cmd_vel; sent zero")
                self.invalid_reported = True
                return
            self.invalid_reported = False
            self.write(velocity(*values, self.cfg["robot_id"], self.cfg["host_id"]))
            self.watchdog.received(now)
        except (OSError, serial.SerialException) as exc:
            self.disconnect(exc)

    def tick(self):
        now = time.monotonic()
        try:
            if self.port is None:
                if now < self.next_connect:
                    return
                self.port = self.serial_factory(self.cfg["port"], self.cfg["baudrate"],
                                                timeout=0, write_timeout=0.1)
                self.parser = Parser()
                self.watchdog.reset()
                self.last_rx = None
                self.connected_at = now
                self.telemetry = TelemetryReceiver()
                self.last_published = None
                self.device_state = None
                self.rx_bytes = 0
                self.stale_reported = False
                if self.cfg["enable_cmd_vel"] or self.cfg['enable_motion_actions']:
                    self.stop()
                self.pending_subscriptions = list(self.subscription_frames)
                self.next_subscription = now
                self.get_logger().info("Serial connected; independent telemetry/state requests scheduled")
            if self.pending_subscriptions and now >= self.next_subscription:
                self.write(self.pending_subscriptions.pop(0))
                self.next_subscription = now + 0.02  # Independent requests, once per connection.
            # Watchdog precedes RX processing; a noisy stream cannot starve it.
            if self.actions is not None:
                self.actions.tick()
                if self.port is None:
                    return
            if self.watchdog.expired(now):
                self.stop()
                self.get_logger().warning("cmd_vel timeout; sent zero velocity")
            received = self.port.read(min(self.port.in_waiting, 4096))
            self.rx_bytes += len(received)
            for packet in self.parser.feed(received):
                if (packet.target, packet.source) == (self.cfg['host_id'], self.cfg['robot_id']):
                    if packet.a == 0x6F and self.actions is not None:
                        self.actions.receive(packet)
                        continue
                if (packet.target, packet.source) != (self.cfg['host_id'], self.cfg['robot_id']):
                    continue
                if (packet.a, packet.b) == (0x8C, 0x40) and len(packet.payload) >= 12:
                    self.device_state = {'received': True, 'imu_calibrated': packet.payload[0]}
                name = self.telemetry.receive(packet, time.monotonic())
                if name is not None:
                    self.last_rx = time.monotonic()
                    self.publish_state(self.telemetry.streams[name].state,
                                       publisher=self.stream_pubs[name], publish_imu=False, publish_tf=False)
            selected = self.telemetry.preferred(now)
            if selected is not None:
                status = self.telemetry.streams[selected]
                key = (selected, status.ok)
                if key != self.last_published:
                    self.publish_state(status.state)
                    self.last_published = key
            else:
                self.last_published = None
            if now - self.last_diagnostic >= 1.0:
                self.last_diagnostic = now
                report = {'bytes': self.rx_bytes, 'selected': selected, 'state': self.device_state,
                          'streams': {name: {'wire': row.wire, 'ok': row.ok, 'reject': row.rejected,
                                            'len': row.length, 'ver': row.version,
                                            'fresh': row.fresh(now),
                                            'age_s': None if row.last_rx is None else max(0, now-row.last_rx)}
                                      for name, row in self.telemetry.streams.items()}}
                message = String()
                message.data = json.dumps(report)
                self.diagnostics_pub.publish(message)
                # Missing telemetry is reported without reconnecting or interrupting motion.
                stale = self.last_rx is None and now-self.connected_at >= self.cfg['telemetry_timeout']
                stale = stale or (self.last_rx is not None and now-self.last_rx >= self.cfg['telemetry_timeout'])
                if stale and not self.stale_reported:
                    self.get_logger().warning('No fresh telemetry; motion still requires ACK/DONE and its own deadline')
                self.stale_reported = stale
        except (OSError, serial.SerialException) as exc:
            self.disconnect(exc)

    def publish_state(self, state, *, publisher=None, publish_imu=True, publish_tf=True):
        stamp = self.get_clock().now().to_msg()  # Host receive time, not MCU uptime.
        q = quaternion(state.rpy)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.cfg["odom_frame"]
        odom.child_frame_id = self.cfg["base_frame"]
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = state.position
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = state.body_velocity
        # Conservative placeholders, not a calibrated covariance model.
        for index, variance in zip((0, 7, 14, 21, 28, 35), (0.05, 0.05, 1e6, 1e6, 1e6, 0.1)):
            odom.pose.covariance[index] = variance
        for index, variance in zip((0, 7, 14, 21, 28, 35), (0.05, 0.05, 1e6, 1e6, 1e6, 1e6)):
            odom.twist.covariance[index] = variance
        if state.gyro is not None:
            odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z = state.gyro
            for index in (21, 28, 35):
                odom.twist.covariance[index] = 0.1
        (publisher if publisher is not None else self.odom_pub).publish(odom)
        if publish_imu and state.gyro is not None:
            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = self.cfg["imu_frame"]
            # Odom yaw and sensor attitude may use different fusion sources.
            # Do not advertise that combination as a measured IMU orientation.
            imu.orientation.w = 1.0
            imu.orientation_covariance[0] = -1.0
            imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = state.gyro
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = state.acceleration
            # All-zero covariance means unknown per sensor_msgs/Imu.
            self.imu_pub.publish(imu)
        if publish_tf and self.tf is not None:
            transform = TransformStamped()
            transform.header = odom.header
            transform.child_frame_id = odom.child_frame_id
            transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = state.position
            transform.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(transform)

    def destroy_node(self):
        if self.actions is not None:
            self.actions.destroy()
            self.actions = None
        if self.port is not None:
            try:
                if self.cfg["enable_cmd_vel"] or self.cfg['enable_motion_actions']:
                    self.stop()
                for name in ('odom', 'velpos'):
                    self.write(subscribe(name, self.cfg["frequency"], False,
                                         self.cfg["robot_id"], self.cfg["host_id"]))
            except (OSError, serial.SerialException) as exc:
                self.get_logger().error(f"Shutdown write failed: {exc}")
            finally:
                self.port.close()
                self.port = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DcaronBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
