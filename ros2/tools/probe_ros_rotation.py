"""Installed ROS action -> real serial -> MCU test, with bounded rotation only."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from dcaron_interfaces.action import Motion
from dcaron_bridge.node import DcaronBridge
import serial


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--port', required=True)
    cli.add_argument('--degrees', type=float, default=0)
    cli.add_argument('--rate', type=float, default=0.2)
    cli.add_argument('--log', required=True, type=Path)
    options = cli.parse_args()
    if (not math.isfinite(options.degrees) or abs(options.degrees) > 30
            or not math.isfinite(options.rate) or not 0 < options.rate <= 0.3):
        cli.error('Probe limited to +/-30 degrees and (0, 0.3] rad/s')
    records = []
    start = time.monotonic()
    observed = []
    bridge = peer = client = executor = handle = None
    success = False

    def log(event, **values):
        record = dict(event=event, time=datetime.now(timezone.utc).isoformat(),
                      elapsed=round(time.monotonic() - start, 4), **values)
        records.append(record)
        if event != 'odom':
            print(json.dumps(record), flush=True)

    class LoggedSerial(serial.Serial):
        def write(self, data):
            count = super().write(data)
            log('serial_tx', raw=data.hex(' '), count=count)
            return count

    def odom(msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        p = msg.pose.pose.position
        observed.append((yaw, (p.x, p.y, p.z)))
        log('odom', yaw_deg=math.degrees(yaw), position_m=[p.x, p.y, p.z])

    def spin_until(predicate, seconds):
        deadline = time.monotonic() + seconds
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        if not predicate():
            raise RuntimeError('ROS/hardware response timeout')

    rclpy.init()
    try:
        bridge = DcaronBridge(serial_factory=LoggedSerial, parameter_overrides=[
            Parameter('port', value=options.port),
            Parameter('enable_motion_actions', value=bool(options.degrees)),
            Parameter('telemetry', value='velpos')])
        peer = Node('rotation_probe_client')
        peer.create_subscription(Odometry, 'odom', odom, 10)
        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        executor.add_node(peer)
        spin_until(lambda: len(observed) >= 3, 5)
        before = observed[-1]
        if options.degrees:
            client = ActionClient(peer, Motion, 'motion')
            spin_until(client.server_is_ready, 5)
            goal = Motion.Goal()
            goal.command = Motion.Goal.ROTATE
            goal.yaw, goal.angular_speed, goal.timeout = math.radians(options.degrees), options.rate, 8.0
            log('goal', requested_degrees=options.degrees, angular_speed=options.rate)
            accepted = client.send_goal_async(goal, feedback_callback=lambda message:
                log('feedback', process=message.feedback.process, percent=message.feedback.percent,
                    notice=message.feedback.notice))
            spin_until(accepted.done, 3)
            handle = accepted.result()
            if not handle.accepted:
                raise RuntimeError('Rotation action rejected')
            result = handle.get_result_async()
            spin_until(result.done, 10)
            outcome = result.result()
            log('result', status=outcome.status, success=outcome.result.success,
                notice=outcome.result.notice, message=outcome.result.message)
            success = outcome.status == 4 and outcome.result.success
            handle = None
        else:
            success = True
        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.02)
        after = observed[-1]
        delta = math.atan2(math.sin(after[0] - before[0]), math.cos(after[0] - before[0]))
        log('summary', requested_deg=options.degrees, measured_delta_deg=math.degrees(delta),
            before_position_m=before[1], after_position_m=after[1], success=success,
            odom_messages=len(observed))
    except (RuntimeError, OSError) as exc:
        log('error', message=str(exc))
    finally:
        if handle is not None and handle.accepted:
            try:
                cancel = handle.cancel_goal_async()
                spin_until(cancel.done, 2)
                until = time.monotonic() + 0.2
                while time.monotonic() < until:
                    executor.spin_once(timeout_sec=0.02)
            except Exception as exc:
                log('cancel_failed', message=str(exc))
        if client is not None:
            client.destroy()
        if executor is not None:
            executor.shutdown()
        if bridge is not None:
            bridge.destroy_node()  # Attempts final zero and unsubscribe before closing.
        if peer is not None:
            peer.destroy_node()
        rclpy.shutdown()
        options.log.parent.mkdir(parents=True, exist_ok=True)
        options.log.write_text('\n'.join(json.dumps(row) for row in records) + '\n', encoding='utf-8')
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
