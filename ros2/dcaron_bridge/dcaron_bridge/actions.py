"""ROS action lifecycle. Uses the bridge's single-thread executor and serial owner."""
from dataclasses import fields
import time

from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from dcaron_interfaces.action import Motion, Trajectory
from .queue import QueueClient, validate as validate_queue

from .motion import MotionSpec, motion_frames, validate_limits, decode_progress, ReplyQuarantine
from .protocol import quaternion


class MotionActions:
    def __init__(self, bridge):
        self.bridge = bridge
        self.reserved = False
        self.active = None
        self.quarantine = ReplyQuarantine()
        self.trajectory_server = ActionServer(
            bridge, Trajectory, 'trajectory', execute_callback=self.execute,
            goal_callback=self.admit_trajectory, cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.server = ActionServer(
            bridge, Motion, 'motion', execute_callback=self.execute,
            goal_callback=self.admit, cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

    def connected(self):
        return self.bridge.port is not None

    @staticmethod
    def spec(request):
        return MotionSpec(**{field.name: getattr(request, field.name) for field in fields(MotionSpec)})

    def admit(self, request):
        b = self.bridge
        try:
            spec = self.spec(request)
            if self.use_queue(spec): validate_queue(spec, b.cfg)
            else: validate_limits(spec, b.cfg)
            if self.reserved or self.active is not None:
                raise ValueError('Another motion is active; wait for result or cancel it')
            if not self.connected():
                raise ValueError('Serial disconnected')
            if not self.use_queue(spec) and spec.command in self.quarantine.pending:
                raise ValueError('Old terminal reply unresolved for this command; do not guess completion')
        except ValueError as exc:
            b.get_logger().warning(f'Motion rejected: {exc}')
            return GoalResponse.REJECT
        self.reserved = True
        return GoalResponse.ACCEPT

    def use_queue(self, spec):
        return self.bridge.cfg.get('motion_protocol', 'queue') == 'queue' and spec.command in (0x63,0x64,0x65,0x66)

    def admit_trajectory(self, request):
        try:
            if self.reserved or self.active is not None or not self.connected():
                raise ValueError('Busy or serial disconnected')
            if not 1 <= len(request.segments) <= 128: raise ValueError('Use 1..128 segments')
            import math
            if not math.isfinite(request.timeout) or not 0 <= request.timeout <= self.bridge.cfg['max_motion_timeout']:
                raise ValueError('Invalid trajectory deadline')
            for row in request.segments: validate_queue(self.segment(row), self.bridge.cfg)
        except ValueError as exc:
            self.bridge.get_logger().warning(str(exc));return GoalResponse.REJECT
        self.reserved = True
        return GoalResponse.ACCEPT

    @staticmethod
    def segment(row):
        return MotionSpec(**{name:getattr(row,name) for name in ('command','x','y','yaw','speed','angular_speed','radius','profile')})

    async def execute(self, handle):
        batch = hasattr(handle.request, 'segments')
        specs = [self.segment(row) for row in handle.request.segments] if batch else [self.spec(handle.request)]
        spec = specs[0]
        b = self.bridge
        done = Future()
        self.active = dict(handle=handle, spec=spec, done=done, sent=False, batch=batch, queue=None,
                           deadline=time.monotonic() + (handle.request.timeout or b.cfg['motion_timeout']))
        if batch or self.use_queue(spec):
            self.active['queue'] = QueueClient(specs, b.write, b.cfg['robot_id'], b.cfg['host_id'],
                                                sequential=bool(batch and handle.request.sequential))
        try:
            if handle.is_cancel_requested:
                self.finish('canceled', 254, 'Canceled before transmission')
            elif not self.connected():
                self.finish('aborted', 254, 'Serial disconnected before transmission')
            else:
                b.watchdog.reset()  # Bounded moves are governed by their own deadline.
                self.active['sent'] = True
                try:
                    if self.active['queue'] is not None: self.active['queue'].tick(time.monotonic())
                    else: b.write(motion_frames(spec, b.cfg['robot_id'], b.cfg['host_id']))
                    b.get_logger().info(f'Motion sent: command=0x{spec.command:02x}, goal={bytes(handle.goal_id.uuid).hex()}')
                except (OSError, b.serial_error) as exc:
                    self.interrupt(f'Motion write failed: {exc}', send_stop=False)
                    b.disconnect(exc)
            status, result = await done
            if status == 'succeeded':
                handle.succeed()
            elif status == 'canceled':
                handle.canceled()
            else:
                handle.abort()
            return result
        finally:
            self.active = None
            self.reserved = False

    def finish(self, status, notice, message, progress=None):
        active = self.active
        if active is None or active['done'].done():
            return
        result = Trajectory.Result() if active['batch'] else Motion.Result()
        if active['batch']: result.completed = len(active['queue'].completed)
        result.success = status == 'succeeded'
        result.notice = notice
        result.message = message
        if not active['batch'] and progress is not None and progress.pose is not None:
            result.pose_valid = True
            result.pose.header.frame_id = self.bridge.cfg['odom_frame']
            result.pose.header.stamp = self.bridge.get_clock().now().to_msg()
            result.pose.pose.position.x, result.pose.pose.position.y, result.pose.pose.position.z = progress.pose[:3]
            q = quaternion((0.0, 0.0, progress.pose[3]))
            result.pose.pose.orientation.x, result.pose.pose.orientation.y, result.pose.pose.orientation.z, result.pose.pose.orientation.w = q
        active['done'].set_result((status, result))
        self.bridge.get_logger().info(f'Motion {status}: notice=0x{notice:02x} {message}')

    def interrupt(self, message, *, canceled=False, send_stop=True, notice=254):
        active = self.active
        if active is None or active['done'].done():
            return
        if active['sent'] and active['queue'] is None:
            self.quarantine.abandon(active['spec'].command)
        self.finish('canceled' if canceled else 'aborted', notice, message)
        if (send_stop and active['sent'] and self.bridge.port is not None
                and (active['queue'] is None or active['queue'].motion_started)):
            try:
                if active['queue'] is not None: self.bridge.write(active['queue'].management(4))
                self.bridge.stop()
            except (OSError, self.bridge.serial_error) as exc:
                self.bridge.disconnect(exc)

    def tick(self):
        active = self.active
        if active is None or active['done'].done():
            return
        if active['handle'].is_cancel_requested:
            self.interrupt('Cancellation requested; zero velocity attempted', canceled=True)
        elif time.monotonic() >= active['deadline']:
            self.interrupt('Motion deadline exceeded; zero velocity attempted')
        elif active['queue'] is not None:
            try: active['queue'].tick(time.monotonic())
            except (ValueError, TimeoutError) as exc: self.interrupt(str(exc), notice=getattr(exc,"notice",254))

    def receive(self, packet):
        active = self.active
        if active is not None and not active['done'].done() and active['queue'] is not None:
            if active['handle'].is_cancel_requested:
                self.interrupt('Cancellation requested', canceled=True);return
            queue = active['queue']
            previous = queue.feedback
            try: queue.receive(packet)
            except (ValueError, TimeoutError) as exc:
                self.interrupt(str(exc), notice=getattr(exc,"notice",254));return
            if queue.feedback is not None and queue.feedback != previous:
                task_id, index, process, notice = queue.feedback
                feedback = Trajectory.Feedback() if active['batch'] else Motion.Feedback()
                if active['batch']: feedback.task_id, feedback.segment_index = task_id,index
                feedback.process, feedback.notice = process, notice
                feedback.percent = 100.0 if process == 255 else max(0.0,(process-1)*100.0/254)
                active['handle'].publish_feedback(feedback)
            if queue.done: self.finish('succeeded',queue.final_notice,'All task IDs completed')
            return
        try:
            progress = decode_progress(packet)
        except ValueError:
            return
        if self.quarantine.discard(progress):
            return
        active = self.active
        if active is None or active['done'].done() or not active['sent']:
            return
        if progress.command != active['spec'].command:
            return
        if progress.mode is not None and progress.mode != active['spec'].mode:
            return
        # Cancellation wins if already accepted, even if a terminal arrives in this tick.
        if active['handle'].is_cancel_requested:
            self.interrupt('Cancellation requested; zero velocity attempted', canceled=True)
            self.quarantine.discard(progress)
            return
        feedback = Motion.Feedback()
        feedback.process, feedback.percent, feedback.notice = progress.process, progress.percent, progress.notice
        active['handle'].publish_feedback(feedback)
        if progress.terminal:
            self.finish('succeeded' if progress.notice == 0 else 'aborted', progress.notice,
                        'MCU completed' if progress.notice == 0 else 'MCU rejected or interrupted motion', progress)

    def destroy(self):
        self.interrupt('Bridge shutdown; zero velocity attempted')
        self.server.destroy()
        self.trajectory_server.destroy()
