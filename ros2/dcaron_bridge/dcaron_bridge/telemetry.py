"""Independent real Odom/VelPos reception, diagnostics and freshness selection."""
from dataclasses import dataclass

from .protocol import STREAMS, decode


@dataclass
class StreamStatus:
    wire: int = 0
    ok: int = 0
    rejected: int = 0
    length: int = 0
    version: int = 0
    last_rx: float | None = None
    state: object = None

    def fresh(self, now, timeout=1.0):
        return self.last_rx is not None and now - self.last_rx < timeout


class TelemetryReceiver:
    """Only count checksum-validated, correctly addressed frames passed by caller."""
    def __init__(self):
        self.streams = {name: StreamStatus() for name in STREAMS}

    def receive(self, packet, now):
        if packet.a != 0x6C:
            return None
        name = next((name for name, code in STREAMS.items() if code == packet.b), None)
        if name is None:
            return None
        status = self.streams[name]
        status.wire += 1
        status.length = len(packet.payload)
        status.version = packet.payload[0] if packet.payload else 0
        try:
            state = decode(packet)
        except ValueError:
            status.rejected += 1
            return None
        status.ok += 1
        status.last_rx, status.state = now, state
        return name

    def preferred(self, now):
        for name in ('odom', 'velpos'):
            if self.streams[name].fresh(now):
                return name
        return None
