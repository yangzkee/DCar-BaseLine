"""DFLink V3 wire contract. No ROS or serial dependencies.

Reference: repository DFLink_V3协议手册_对外版.md, sections 2, 5 and 7.
"""

from dataclasses import dataclass
import math
import struct


FREQUENCIES = {10: 1, 50: 5, 100: 10, 200: 20, 250: 25, 500: 50, 1000: 100}
STREAMS = {"velpos": 0x81, "odom": 0x80}


@dataclass(frozen=True)
class Frame:
    target: int
    source: int
    a: int
    b: int
    payload: bytes


def frame(a, b, payload=b"", target=1, source=0x97):
    if len(payload) > 255:
        raise ValueError("DFLink payload exceeds 255 bytes")
    body = bytes((0xDF, target, source, a, b, len(payload))) + payload + b"\xfd"
    return body + struct.pack("<H", sum(body) & 0xFFFF)


def velocity(vx, vy, wz, target=1, source=0x97):
    values = (vx, vy, wz)
    if any(not math.isfinite(v) or abs(v) > 214748.3647 for v in values):
        raise ValueError("Velocity must be finite and fit signed SI x 10000")
    # Match the existing C clients' truncation toward zero.
    return frame(0x02, 0x62, struct.pack("<iii", *(int(v * 10000) for v in values)),
                 target, source)


def subscribe(stream, hz=10, continuous=True, target=1, source=0x97):
    if stream not in STREAMS or hz not in FREQUENCIES:
        raise ValueError("Unsupported stream or DFLink frequency")
    return frame(0x04, STREAMS[stream], bytes((1 if continuous else 2, FREQUENCIES[hz])),
                 target, source)


class Parser:
    """Accept fragmented/coalesced frames and printf noise with bounded storage.

    Scan later candidates too: a corrupt LEN must not indefinitely hide a valid
    following frame. Check tail AND checksum before accepting a candidate.
    """

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while self.buffer:
            incomplete = None
            found = False
            for start, byte in enumerate(self.buffer):
                if byte != 0xDF:
                    continue
                if len(self.buffer) - start < 6:
                    if incomplete is None:
                        incomplete = start
                    continue
                end = start + 9 + self.buffer[start + 5]
                if end > len(self.buffer):
                    if incomplete is None:
                        incomplete = start
                    continue
                packet = self.buffer[start:end]
                if packet[-3] != 0xFD or (sum(packet[:-2]) & 0xFFFF) != int.from_bytes(packet[-2:], "little"):
                    continue
                result.append(Frame(*packet[1:5], bytes(packet[6:-3])))
                del self.buffer[:end]
                found = True
                break
            if not found:
                self.buffer = self.buffer[incomplete:] if incomplete is not None else bytearray()
                break
        return result


@dataclass(frozen=True)
class Telemetry:
    rpy: tuple
    body_velocity: tuple
    position: tuple
    acceleration: tuple | None
    gyro: tuple | None
    device_time: tuple


def decode(packet):
    """Decode exactly Odom v4 / VelPos v2; reject unknown layouts."""
    p = packet.payload
    if packet.a != 0x6C:
        raise ValueError("Not a telemetry frame")
    if packet.b == 0x80 and len(p) == 51 and p[0] == 4:
        shorts = struct.unpack_from("<15h", p, 1)
        return Telemetry(
            tuple(v / 10000 for v in shorts[:3]),
            tuple(v / 5000 for v in shorts[9:12]),
            tuple(v / 1000 for v in struct.unpack_from("<3i", p, 31)),
            tuple(v / 400 for v in shorts[3:6]),
            tuple(v / 1800 for v in shorts[6:9]),
            struct.unpack_from("<2I", p, 43),
        )
    if packet.b == 0x81 and len(p) == 35 and p[0] == 2:
        shorts = struct.unpack_from("<7h", p, 1)
        return Telemetry(
            (0.0, 0.0, shorts[0] / 10000),
            tuple(v / 5000 for v in shorts[1:4]),
            tuple(v / 1000 for v in struct.unpack_from("<3i", p, 15)),
            None, None, struct.unpack_from("<2I", p, 27),
        )
    raise ValueError("Expected Odom v4 (51 bytes) or VelPos v2 (35 bytes)")


def quaternion(rpy):
    roll, pitch, yaw = rpy
    cr, cp, cy = (math.cos(v / 2) for v in (roll, pitch, yaw))
    sr, sp, sy = (math.sin(v / 2) for v in (roll, pitch, yaw))
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class CommandWatchdog:
    """Return a stop once when a command expires; reconnect discards commands."""

    def __init__(self, timeout):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("cmd_timeout must be positive and finite")
        self.timeout = timeout
        self.last = None

    def reset(self):
        self.last = None

    def received(self, now):
        self.last = now

    def expired(self, now):
        if self.last is not None and now - self.last >= self.timeout:
            self.reset()
            return True
        return False
