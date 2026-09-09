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

from .protocol import (Parser, STREAMS, CommandWatchdog, subscribe, velocity,
                       decode, quaternion)


class DcaronBridge(Node):
    def __init__(self, *, serial_factory=None, **kwargs):
        super().__init__("dcaron_bridge", **kwargs)
        defaults = {
            "port": "COM3", "baudrate": 460800, "robot_id": 1, "host_id": 151,
            "telemetry": "velpos", "frequency": 10,
            "enable_cmd_vel": False, "cmd_timeout": 0.5,
            "max_linear_speed": 0.5, "max_angular_speed": 1.0,
            "telemetry_timeout": 2.0, "reconnect_interval": 3.0,
            "odom_frame": "odom", "base_frame": "base_link", "imu_frame": "base_link",
            "publish_tf": False,
            "enable_motion_actions": False, "motion_timeout": 30.0,
            "max_motion_timeout": 120.0, "max_displacement": 2.0, "max_rotation": 6.283185307179586,
        }
        self.cfg = {key: self.declare_parameter(
            key, value, ParameterDescriptor(read_only=True)).value
            for key, value in defaults.items()}
        c = self.cfg
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
        self.subscribe_frame = subscribe(c["telemetry"], c["frequency"],
                                         target=c["robot_id"], source=c["host_id"])
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
        self.imu_pub = self.create_publisher(Imu, "imu/data", 10) if c["telemetry"] == "odom" else None
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
            f"stream={c['telemetry']} frequency={c['frequency']}Hz motion={c['enable_cmd_vel']}")

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
            self.actions.interrupt(f'Serial/telemetry failure: {error}', send_stop=False)
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
        # Ignore commands while telemetry is absent/stale. Never replay on reconnect.
        now = time.monotonic()
        if self.last_rx is None or now - self.last_rx >= self.cfg["telemetry_timeout"]:
            return
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
                self.stale_reported = False
                if self.cfg["enable_cmd_vel"] or self.cfg['enable_motion_actions']:
                    self.stop()
                self.write(self.subscribe_frame)  # Exactly once per connection.
                self.get_logger().info("Serial connected; DFLink subscription sent, waiting for telemetry")
            # Watchdog precedes RX processing; a noisy stream cannot starve it.
            if self.actions is not None:
                self.actions.tick()
                if self.port is None:
                    return
            if self.watchdog.expired(now):
                self.stop()
                self.get_logger().warning("cmd_vel timeout; sent zero velocity")
            for packet in self.parser.feed(self.port.read(min(self.port.in_waiting, 4096))):
                if (packet.target, packet.source) == (self.cfg['host_id'], self.cfg['robot_id']):
                    if packet.a == 0x6F and self.actions is not None:
                        self.actions.receive(packet)
                        continue
                if (packet.target, packet.source, packet.a, packet.b) != (
                        self.cfg["host_id"], self.cfg["robot_id"], 0x6C,
                        STREAMS[self.cfg["telemetry"]]):
                    continue
                try:
                    state = decode(packet)
                except ValueError:
                    continue
                if self.last_rx is None or self.stale_reported:
                    self.get_logger().info("Valid telemetry received")
                self.last_rx = time.monotonic()
                self.stale_reported = False
                self.publish_state(state)
            if now - (self.last_rx if self.last_rx is not None else self.connected_at) >= self.cfg["telemetry_timeout"]:
                if not self.stale_reported:
                    if self.cfg["enable_cmd_vel"] or self.cfg['enable_motion_actions']:
                        self.stop()
                    self.get_logger().warning(
                        "No fresh telemetry: check wiring, stream version and license (Odom v4 requires Pro). "
                        "No stale data published; motion input inhibited.")
                    self.stale_reported = True
                # Resubscribe on a new connection, also recovering MCU restarts.
                self.disconnect("Telemetry timeout")
        except (OSError, serial.SerialException) as exc:
            self.disconnect(exc)

    def publish_state(self, state):
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
        self.odom_pub.publish(odom)
        if self.imu_pub is not None and state.gyro is not None:
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
        if self.tf is not None:
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
                self.write(subscribe(self.cfg["telemetry"], self.cfg["frequency"], False,
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
