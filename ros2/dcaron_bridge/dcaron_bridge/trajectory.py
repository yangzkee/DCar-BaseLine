"""Run one v6 trajectory; validate only unless --execute is explicitly supplied."""
import argparse
import math
from pathlib import Path

import yaml

from .motion import MotionSpec
from .queue import encode


def load_trajectory(path):
    """Validate the YAML and wire values without importing ROS or opening serial."""
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict) or not isinstance(data.get('segments'), list):
        raise ValueError('Trajectory must contain a segments list; use trajectory_sequence YAML format')
    if not 1 <= len(data['segments']) <= 128:
        raise ValueError('Use 1..128 segments')
    timeout = float(data.get('timeout', 60))
    if not math.isfinite(timeout) or timeout < 0:
        raise ValueError('Trajectory timeout must be finite and nonnegative seconds')
    specs = []
    for index, row in enumerate(data['segments']):
        if not isinstance(row, dict):
            raise ValueError(f'Segment {index} must be a mapping')
        try:
            spec = MotionSpec(**({'profile': 18} | row))
            encode(spec, index + 1)
        except TypeError as error:
            raise ValueError(f'Segment {index} has unknown fields or nonnumeric motion values') from error
        specs.append(spec)
    return specs, timeout


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('file', type=Path)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--sequential', action='store_true')
    options = parser.parse_args(args)
    specs, timeout = load_trajectory(options.file)
    if not options.execute:
        print(f'Validated {len(specs)} segments; no serial or ROS connection.')
        return

    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from dcaron_interfaces.action import Trajectory
    from dcaron_interfaces.msg import MotionSegment

    rclpy.init()
    node = Node('dcaron_trajectory_sequence')
    client = ActionClient(node, Trajectory, 'trajectory')
    handle = None
    completed = False
    try:
        if not client.wait_for_server(timeout_sec=5):
            raise RuntimeError('Trajectory server unavailable; enable motion actions on the bridge')
        goal = Trajectory.Goal()
        goal.timeout = timeout
        goal.sequential = options.sequential
        for spec in specs:
            segment = MotionSegment()
            for name in ('command', 'x', 'y', 'yaw', 'speed', 'angular_speed', 'radius', 'profile'):
                value = getattr(spec, name)
                setattr(segment, name, value if name in ('command', 'profile') else float(value))
            goal.segments.append(segment)
        future = client.send_goal_async(goal, feedback_callback=lambda message: print(message.feedback, flush=True))
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        if not future.done():
            raise RuntimeError('Goal acknowledgement lost; inspect bridge before retrying')
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError('Goal rejected; inspect bridge log for the reason')
        result = handle.get_result_async()
        # A zero goal timeout uses the bridge default; allow its default maximum of 120s.
        rclpy.spin_until_future_complete(node, result, timeout_sec=timeout + 10 if timeout else 130)
        if not result.done():
            raise RuntimeError('Result timeout; attempting cancellation')
        completed = True
        print(result.result().result)
        if result.result().status != 4 or not result.result().result.success:
            raise RuntimeError('Trajectory failed')
    finally:
        if handle is not None and handle.accepted and not completed:
            future = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, future, timeout_sec=3)
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
