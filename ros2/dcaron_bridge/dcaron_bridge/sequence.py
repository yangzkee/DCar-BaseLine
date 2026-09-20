"""Sequential ROS action client: advances only on a successful MCU terminal reply."""
import argparse
from dataclasses import fields
import json
from pathlib import Path
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from dcaron_interfaces.action import Motion
from nav_msgs.msg import Odometry
import yaml

from .motion import MotionSpec, motion_frames


def main(args=None):
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('file', type=Path)
    cli.add_argument('--execute', action='store_true', help='Without this flag: validate only, no ROS connection')
    cli.add_argument('--action', default='/motion')
    cli.add_argument('--odom', default='/odom')
    cli.add_argument('--log', type=Path)
    options = cli.parse_args(args)
    data = yaml.safe_load(options.file.read_text(encoding='utf-8'))
    if not isinstance(data, list) or not data:
        cli.error('Sequence must be a nonempty YAML list')
    specs = []
    for row in data:
        if not isinstance(row, dict):
            cli.error('Each step must be a mapping')
        row = dict(row)
        for key in ('command', 'next_command'):
            if isinstance(row.get(key), str):
                if row[key] not in ('ROTATE', 'LINEAR', 'LINEAR_WITH_YAW', 'ARC', 'CROSS_RECORD', 'CROSS_LOCALIZE'):
                    cli.error(f'Unknown command {row[key]}')
                row[key] = getattr(Motion.Goal, row[key])
        spec = MotionSpec(**row)
        motion_frames(spec)
        specs.append(spec)
    if not options.execute:
        print(f'Validated {len(specs)} steps. No motion sent; add --execute to run.')
        return
    rclpy.init()
    node = Node('dcaron_motion_sequence')
    client = ActionClient(node, Motion, options.action)
    handle = None
    events = []
    start = time.monotonic()

    def log(event, **values):
        record = dict(event=event, elapsed=round(time.monotonic() - start, 4), **values)
        events.append(record)
        print(json.dumps(record), flush=True)

    def odom(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        log('odom', position=[p.x, p.y, p.z], quaternion=[q.x, q.y, q.z, q.w])

    node.create_subscription(Odometry, options.odom, odom, 10)
    try:
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError('Motion server unavailable; enable_motion_actions must be true')
        for index, spec in enumerate(specs):
            request = Motion.Goal()
            for field in fields(MotionSpec):
                value = getattr(spec, field.name)
                if field.name in ('x', 'y', 'yaw', 'speed', 'angular_speed', 'radius', 'timeout'):
                    value = float(value)
                setattr(request, field.name, value)
            log('goal', step=index, command=spec.command)
            response = client.send_goal_async(request, feedback_callback=lambda message:
                log('feedback', process=message.feedback.process, percent=message.feedback.percent,
                    notice=message.feedback.notice))
            rclpy.spin_until_future_complete(node, response, timeout_sec=5)
            if not response.done():
                raise RuntimeError('Goal response timed out; server may have received the command')
            handle = response.result()
            if not handle.accepted:
                handle = None
                raise RuntimeError(f'Step {index} rejected')
            result_future = handle.get_result_async()
            # Server enforces its own configured deadline, at most 120 s by default.
            rclpy.spin_until_future_complete(node, result_future, timeout_sec=(spec.timeout or 120) + 5)
            if not result_future.done():
                raise RuntimeError('Result timeout; attempting cancellation, no next step')
            response = result_future.result()
            handle = None
            log('result', step=index, status=response.status, success=response.result.success,
                notice=response.result.notice, message=response.result.message)
            if response.status != 4 or not response.result.success:
                raise RuntimeError('Motion did not succeed; sequence stopped')
    finally:
        if handle is not None and handle.accepted:
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel, timeout_sec=2)
            log('cancel_attempt', response_received=cancel.done())
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
        if options.log:
            options.log.parent.mkdir(parents=True, exist_ok=True)
            options.log.write_text('\n'.join(json.dumps(row) for row in events) + '\n', encoding='utf-8')
