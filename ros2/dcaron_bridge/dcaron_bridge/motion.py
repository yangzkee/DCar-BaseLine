"""Bounded DFLink motions and replies; independently testable without ROS."""
from dataclasses import dataclass
import math
import struct

from .protocol import frame

ROTATE, LINEAR, LINEAR_WITH_YAW, ARC = 0x63, 0x64, 0x65, 0x66
CROSS_RECORD, CROSS_LOCALIZE = 0x80, 0x81
COMMANDS = (ROTATE, LINEAR, LINEAR_WITH_YAW, ARC, CROSS_RECORD, CROSS_LOCALIZE)


@dataclass(frozen=True)
class MotionSpec:
    command: int
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    speed: float = 0.0
    angular_speed: float = 0.0
    radius: float = 0.0
    profile: int = 1
    mode: int = 0
    next_command: int = 0
    timeout: float = 0.0


def fixed(*values):
    if any(not math.isfinite(v) or abs(v) > 214748.3647 for v in values):
        raise ValueError('Motion value must be finite and fit signed SI x 10000')
    return struct.pack('<' + 'i' * len(values), *(int(v * 10000) for v in values))


def motion_frames(spec, target=1, source=151):
    """Return one write; NEXT's two frames must be adjacent on the serial wire."""
    if spec.command not in COMMANDS or spec.profile not in (0, 1, 2) or spec.mode not in (0, 1):
        raise ValueError('Unknown motion, profile or mode')
    fixed(spec.x, spec.y, spec.yaw, spec.speed, spec.angular_speed, spec.radius)
    if spec.command == ROTATE:
        if spec.angular_speed <= 0:
            raise ValueError('Rotation angular_speed must be positive')
        payload = fixed(spec.yaw, spec.angular_speed)
    elif spec.command in (LINEAR, LINEAR_WITH_YAW):
        if spec.speed <= 0:
            raise ValueError('Translation speed must be positive')
        payload = (fixed(spec.x, spec.y, spec.speed) if spec.command == LINEAR else
                   fixed(spec.x, spec.y, spec.yaw, spec.speed)) + bytes((spec.profile,))
    elif spec.command == ARC:
        if spec.radius <= 0 or spec.speed <= 0:
            raise ValueError('Arc radius and speed must be positive')
        payload = fixed(spec.radius, spec.yaw, spec.speed) + bytes((spec.profile,))
    else:
        payload = bytes((spec.mode,))
    result = frame(2, spec.command, payload, target, source)
    if spec.command == CROSS_LOCALIZE and spec.mode == 1:
        if spec.next_command not in (LINEAR, LINEAR_WITH_YAW):
            raise ValueError('Localize NEXT requires an immediately bound linear motion')
        bound = MotionSpec(spec.next_command, spec.x, spec.y, spec.yaw, spec.speed,
                           spec.angular_speed, spec.radius, spec.profile)
        result += motion_frames(bound, target, source)
    return result


def validate_limits(spec, cfg):
    """Validate node admission before it sends any bytes."""
    motion_frames(spec)
    if (not math.isfinite(spec.timeout) or spec.timeout < 0
            or spec.timeout > cfg['max_motion_timeout']):
        raise ValueError('Invalid motion timeout')
    if math.hypot(spec.x, spec.y) > cfg['max_displacement'] or abs(spec.yaw) > cfg['max_rotation']:
        raise ValueError('Motion exceeds displacement/rotation limit')
    if spec.speed > cfg['max_linear_speed'] or spec.angular_speed > cfg['max_angular_speed']:
        raise ValueError('Motion exceeds speed limit')
    if spec.command == ARC:
        if spec.radius * abs(spec.yaw) > cfg['max_displacement']:
            raise ValueError('Arc path exceeds displacement limit')
        if spec.speed / spec.radius > cfg['max_angular_speed']:
            raise ValueError('Arc implied yaw rate exceeds angular speed limit')


@dataclass(frozen=True)
class Progress:
    command: int
    process: int
    notice: int
    mode: int | None = None
    pose: tuple | None = None

    @property
    def terminal(self):
        return self.process == 255

    @property
    def percent(self):
        return 100.0 if self.terminal else max(0.0, (self.process - 1) / 254 * 100)


def decode_progress(packet):
    p = packet.payload
    if packet.a != 0x6F or packet.b not in COMMANDS or len(p) not in (2, 19):
        raise ValueError('Not a supported motion reply')
    if not 1 <= p[0] <= 255:
        raise ValueError('Invalid progress byte')
    if len(p) == 19:
        if packet.b not in (CROSS_RECORD, CROSS_LOCALIZE) or p[:2] != b'\xff\x00' or p[2] not in (0, 1):
            raise ValueError('Invalid cross-marker pose reply')
        return Progress(packet.b, p[0], p[1], p[2],
                        tuple(v / 10000 for v in struct.unpack_from('<4i', p, 3)))
    if packet.b in (CROSS_RECORD, CROSS_LOCALIZE) and p[:2] == b'\xff\x00':
        raise ValueError('Cross-marker success must include the captured pose')
    return Progress(packet.b, p[0], p[1])


class ReplyQuarantine:
    """No wire sequence IDs: never attribute a canceled goal's late reply to a new one.

    Keep unresolved command types across serial reconnects. A terminal drains an
    old type. If the terminal was lost, the same type remains blocked until the
    operator verifies stopped state and restarts the bridge. Never guess success.
    """
    def __init__(self):
        self.pending = set()

    def abandon(self, command):
        self.pending.add(command)

    def discard(self, progress):
        if progress.command not in self.pending:
            return False
        if progress.terminal:
            self.pending.remove(progress.command)
        return True
