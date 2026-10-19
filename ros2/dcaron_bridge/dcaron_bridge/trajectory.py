"""Run one Beta trajectory; dry-run by default, explicit --execute to move."""
import argparse
from pathlib import Path
import yaml
from .motion import MotionSpec
from .queue import encode

def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('file',type=Path)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--sequential',action='store_true')
    options=parser.parse_args(args)
    data=yaml.safe_load(options.file.read_text(encoding='utf-8'))
    specs=[MotionSpec(**({'profile':18} | row)) for row in data['segments']]
    if not 1<=len(specs)<=128: raise ValueError('Use 1..128 segments')
    for i,spec in enumerate(specs): encode(spec,i+1)
    if not options.execute:
        print(f'Validated {len(specs)} segments; no serial or ROS connection.');return
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from dcaron_interfaces.action import Trajectory
    from dcaron_interfaces.msg import MotionSegment
    rclpy.init();node=Node('dcaron_trajectory_sequence');client=ActionClient(node,Trajectory,'trajectory')
    handle=None;completed=False
    try:
        if not client.wait_for_server(timeout_sec=5): raise RuntimeError('Trajectory server unavailable')
        goal=Trajectory.Goal();goal.timeout=float(data.get('timeout',60));goal.sequential=options.sequential
        for spec in specs:
            row=MotionSegment()
            for key in ('command','x','y','yaw','speed','angular_speed','radius','profile'):
                value=getattr(spec,key);setattr(row,key,value if key in ('command','profile') else float(value))
            goal.segments.append(row)
        future=client.send_goal_async(goal,feedback_callback=lambda msg:print(msg.feedback,flush=True))
        rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        if not future.done(): raise RuntimeError('Goal acknowledgement lost; check bridge before retrying')
        handle=future.result()
        if not handle.accepted: raise RuntimeError('Goal rejected')
        result=handle.get_result_async();rclpy.spin_until_future_complete(node,result,timeout_sec=goal.timeout+10)
        if not result.done(): raise RuntimeError('Result timeout')
        completed=True;print(result.result().result)
        if not result.result().result.success: raise RuntimeError('Trajectory failed')
    finally:
        if handle is not None and handle.accepted and not completed:
            future=handle.cancel_goal_async();rclpy.spin_until_future_complete(node,future,timeout_sec=3)
        client.destroy();node.destroy_node();rclpy.shutdown()

if __name__=='__main__': main()
