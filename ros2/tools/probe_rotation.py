"""Direct DFLink test. No ROS required; motion only with explicit --degrees.

Run from repository: python ros2/tools/probe_rotation.py --port COM5 --degrees 15
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import struct
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'dcaron_bridge'))
from dcaron_bridge.protocol import Parser, decode, frame, subscribe, velocity
import serial


def main():
    args = argparse.ArgumentParser(description=__doc__)
    args.add_argument('--port', required=True)
    args.add_argument('--degrees', type=float, default=0.0)
    args.add_argument('--rate', type=float, default=0.2, help='positive rad/s')
    args.add_argument('--timeout', type=float, default=8.0)
    args.add_argument('--log', type=Path, required=True)
    options = args.parse_args()
    if (not all(math.isfinite(v) for v in (options.degrees, options.rate, options.timeout))
            or abs(options.degrees) > 30 or not 0 < options.rate <= 0.3
            or not 0 < options.timeout <= 15):
        args.error('Probe limited to +/-30 deg, (0, 0.3] rad/s and (0, 15] s')
    records = []
    parser = Parser()
    start = time.monotonic()
    before = after = None
    terminal = None
    sent_motion = False
    cleanup_ok = True

    def log(event, **fields):
        record = dict(time=datetime.now(timezone.utc).isoformat(),
                      elapsed=round(time.monotonic() - start, 4), event=event, **fields)
        records.append(record)
        print(json.dumps(record, ensure_ascii=True), flush=True)

    try:
        with serial.Serial(options.port, 460800, timeout=0.02, write_timeout=0.2) as port:
            def send(data, purpose):
                log('tx', purpose=purpose, raw=data.hex(' '))
                if port.write(data) != len(data):
                    raise serial.SerialException('Incomplete write')

            def receive():
                nonlocal after
                for packet in parser.feed(port.read(max(1, min(port.in_waiting, 4096)))):
                    if packet.target != 151 or packet.source != 1:
                        continue
                    fields = dict(a=hex(packet.a), b=hex(packet.b), payload=packet.payload.hex(' '))
                    if packet.a == 0x6C and packet.b == 0x81:
                        state = decode(packet)
                        after = state
                        fields.update(yaw_deg=math.degrees(state.rpy[2]), position_m=state.position,
                                      velocity_mps=state.body_velocity)
                    if packet.a == 0x6F and len(packet.payload) == 2:
                        fields.update(progress=packet.payload[0], notice=packet.payload[1])
                    log('rx', **fields)
                    yield packet

            try:
                log('connected', port=options.port, baudrate=460800)
                send(subscribe('velpos'), 'continuous telemetry 10Hz')
                deadline = time.monotonic() + 2
                while after is None and time.monotonic() < deadline:
                    list(receive())
                if after is None:
                    raise RuntimeError('No valid VelPos v2 received; motion NOT sent')
                before = after
                if options.degrees:
                    payload = struct.pack('<ii', int(math.radians(options.degrees) * 10000),
                                          int(options.rate * 10000))
                    sent_motion = True  # Also stop after a potentially partial write.
                    send(frame(2, 0x63, payload), 'relative rotation')
                    deadline = time.monotonic() + options.timeout
                    while time.monotonic() < deadline and terminal is None:
                        for packet in receive():
                            if packet.a == 0x6F and packet.b == 0x63 and len(packet.payload) == 2:
                                if packet.payload[0] == 255:
                                    terminal = packet.payload[1]
                    if terminal is None:
                        log('timeout', message='No rotation terminal frame; stopping')
                    elif terminal != 0:
                        log('motion_failed', notice=terminal)
                    # Keep observing settling briefly, then force zero in cleanup.
                    deadline = time.monotonic() + 0.5
                    while time.monotonic() < deadline:
                        list(receive())
                delta = math.atan2(math.sin(after.rpy[2] - before.rpy[2]),
                                   math.cos(after.rpy[2] - before.rpy[2]))
                log('summary', requested_deg=options.degrees, measured_delta_deg=math.degrees(delta),
                    terminal_notice=terminal, before_position_m=before.position,
                    after_position_m=after.position)
            finally:
                if sent_motion:
                    try:
                        send(velocity(0, 0, 0), 'final stop')
                    except (OSError, serial.SerialException) as exc:
                        cleanup_ok = False
                        log('stop_failed', message=str(exc))
                try:
                    send(subscribe('velpos', continuous=False), 'stop continuous subscription')
                except (OSError, serial.SerialException) as exc:
                    cleanup_ok = False
                    log('unsubscribe_failed', message=str(exc))
    except (OSError, serial.SerialException, RuntimeError, ValueError) as exc:
        log('error', message=str(exc))
        return 1
    finally:
        options.log.parent.mkdir(parents=True, exist_ok=True)
        options.log.write_text('\n'.join(json.dumps(row, ensure_ascii=False) for row in records) + '\n', encoding='utf-8')
    return 0 if cleanup_ok and (not options.degrees or terminal == 0) else 1


if __name__ == '__main__':
    raise SystemExit(main())
