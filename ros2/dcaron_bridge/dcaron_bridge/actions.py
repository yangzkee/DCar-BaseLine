"""ROS action lifecycle. Uses the bridge's single-thread executor and serial owner."""
from dataclasses import fields
import time

from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future
from dcaron_interfaces.action import Motion

from .motion import MotionSpec, motion_frames, validate_limits, decode_progress, ReplyQuarantine
from .protocol import quaternion


class MotionActions:
    def __init__(self, bridge):
        self.bridge = bridge
        self.reserved = False
        self.active = None
        self.quarantine = ReplyQuarantine()
        self.server = ActionServer(
            bridge, Motion, 'motion', execute_callback=self.execute,
            goal_callback=self.admit, cancel_callback=lambda goal: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

    def fresh(self):
        b = self.bridge
        return (b.port is not None and b.last_rx is not None and
                time.monotonic() - b.last_rx < b.cfg['telemetry_timeout'])

    @staticmethod
    def spec(request):
        return MotionSpec(**{field.name: getattr(request, field.name) for field in fields(MotionSpec)})

    def admit(self, request):
        b = self.bridge
        try:
            spec = self.spec(request)
            validate_limits(spec, b.cfg)
            if self.reserved or self.active is not None:
                raise ValueError('Another motion is active; wait for result or cancel it')
            if not self.fresh():
                raise ValueError('No fresh telemetry')
            if spec.command in self.quarantine.pending:
                raise ValueError('Old terminal reply unresolved for this command; do not guess completion')
        except ValueError as exc:
            b.get_logger().warning(f'Motion rejected: {exc}')
            return GoalResponse.REJECT
        self.reserved = True
        return GoalResponse.ACCEPT

    async def execute(self, handle):
        spec = self.spec(handle.request)
        b = self.bridge
        done = Future()
        self.active = dict(handle=handle, spec=spec, done=done, sent=False,
                           deadline=time.monotonic() + (spec.timeout or b.cfg['motion_timeout']))
        try:
            if handle.is_cancel_requested:
                self.finish('canceled', 254, 'Canceled before transmission')
            elif not self.fresh():
                self.finish('aborted', 254, 'Connection/telemetry lost before transmission')
            else:
                b.watchdog.reset()  # Bounded moves are governed by their own deadline.
                self.active['sent'] = True
                try:
                    b.write(motion_frames(spec, b.cfg['robot_id'], b.cfg['host_id']))
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
        result = Motion.Result()
        result.success = status == 'succeeded'
        result.notice = notice
        result.message = message
        if progress is not None and progress.pose is not None:
            result.pose_valid = True
            result.pose.header.frame_id = self.bridge.cfg['odom_frame']
            result.pose.header.stamp = self.bridge.get_clock().now().to_msg()
            result.pose.pose.position.x, result.pose.pose.position.y, result.pose.pose.position.z = progress.pose[:3]
            q = quaternion((0.0, 0.0, progress.pose[3]))
            result.pose.pose.orientation.x, result.pose.pose.orientation.y, result.pose.pose.orientation.z, result.pose.pose.orientation.w = q
        active['done'].set_result((status, result))
        self.bridge.get_logger().info(f'Motion {status}: notice=0x{notice:02x} {message}')

    def interrupt(self, message, *, canceled=False, send_stop=True):
        active = self.active
        if active is None or active['done'].done():
            return
        if active['sent']:
            self.quarantine.abandon(active['spec'].command)
        self.finish('canceled' if canceled else 'aborted', 254, message)
        if send_stop and active['sent'] and self.bridge.port is not None:
            try:
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

    def receive(self, packet):
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
