import math
import struct
import unittest

from dcaron_bridge.protocol import Parser, Frame
from dcaron_bridge.motion import (MotionSpec, motion_frames, decode_progress,
                                   ReplyQuarantine, validate_limits)


class MotionTests(unittest.TestCase):
    def test_rotation_matches_actual_board_probe(self):
        self.assertEqual(motion_frames(MotionSpec(0x63, yaw=math.radians(15), angular_speed=0.2)),
                         bytes.fromhex('df 01 97 02 63 08 39 0a 00 00 d0 07 00 00 fd fb 03'))

    def test_all_trajectory_wire_layouts(self):
        cases = [
            (MotionSpec(0x64, x=0.2, y=-0.1, speed=0.1, profile=2), (2000, -1000, 1000), 2),
            (MotionSpec(0x65, x=0.2, y=-0.1, yaw=-0.5, speed=0.1), (2000, -1000, -5000, 1000), 1),
            (MotionSpec(0x66, radius=0.5, yaw=0.3, speed=0.1, profile=0), (5000, 3000, 1000), 0),
        ]
        for spec, numbers, profile in cases:
            packet = Parser().feed(motion_frames(spec))[0]
            self.assertEqual(packet.b, spec.command)
            self.assertEqual(packet.payload, struct.pack('<' + 'i' * len(numbers), *numbers) + bytes((profile,)))

    def test_localize_next_is_atomic_pair_and_record_modes(self):
        data = motion_frames(MotionSpec(0x81, mode=1, next_command=0x64, x=0.3, speed=0.1))
        packets = Parser().feed(data)
        self.assertEqual([p.b for p in packets], [0x81, 0x64])
        self.assertEqual(packets[0].payload, b'\x01')
        for mode in (0, 1):
            self.assertEqual(Parser().feed(motion_frames(MotionSpec(0x80, mode=mode)))[0].payload, bytes((mode,)))
        with self.assertRaises(ValueError):
            motion_frames(MotionSpec(0x81, mode=1))

    def test_profiles_and_bad_values(self):
        for profile in range(3):
            packet = Parser().feed(motion_frames(MotionSpec(0x64, x=0.1, speed=0.1, profile=profile)))[0]
            self.assertEqual(packet.payload[-1], profile)
        for spec in (MotionSpec(0x62), MotionSpec(0x63, angular_speed=0),
                     MotionSpec(0x64, speed=-1), MotionSpec(0x64, speed=1, profile=3),
                     MotionSpec(0x66, speed=0.1, radius=-1), MotionSpec(0x63, yaw=math.nan, angular_speed=0.1)):
            with self.assertRaises(ValueError):
                motion_frames(spec)

    def test_limits_include_arc_length_and_yaw_rate(self):
        cfg = dict(max_motion_timeout=60, max_displacement=1, max_rotation=3.2,
                   max_linear_speed=0.5, max_angular_speed=1)
        validate_limits(MotionSpec(0x63, yaw=0.2, angular_speed=0.2), cfg)
        for spec in (MotionSpec(0x66, radius=1, yaw=2, speed=0.1),
                     MotionSpec(0x66, radius=0.01, yaw=1, speed=0.1),
                     MotionSpec(0x63, angular_speed=0.2, timeout=math.inf)):
            with self.assertRaises(ValueError):
                validate_limits(spec, cfg)

    def test_actual_progress_and_result(self):
        packet = Parser().feed(bytes.fromhex('DF 97 01 6F 64 02 80 00 FD C9 03'))[0]
        progress = decode_progress(packet)
        self.assertEqual(progress.percent, 50.0)
        self.assertFalse(progress.terminal)
        final = decode_progress(Frame(151, 1, 0x6F, 0x63, b'\xff\x00'))
        self.assertTrue(final.terminal)
        self.assertEqual(final.notice, 0)
        self.assertEqual(final.percent, 100.0)

    def test_cross_pose_only_valid_for_extended_success(self):
        data = b'\xff\x00\x01' + struct.pack('<4i', 1234, -5678, 0, 30000)
        result = decode_progress(Frame(151, 1, 0x6F, 0x81, data))
        self.assertEqual(result.pose, (0.1234, -0.5678, 0.0, 3.0))
        self.assertEqual(result.mode, 1)
        for command, payload in ((0x81, b'\xff\x00'), (0x64, data), (0x81, b'\xff\x01' + data[2:])):
            with self.assertRaises(ValueError):
                decode_progress(Frame(151, 1, 0x6F, command, payload))

    def test_old_replies_quarantined_until_terminal(self):
        quarantine = ReplyQuarantine()
        quarantine.abandon(0x64)
        progress = decode_progress(Frame(151, 1, 0x6F, 0x64, b'\x80\x00'))
        self.assertTrue(quarantine.discard(progress))
        self.assertIn(0x64, quarantine.pending)
        terminal = decode_progress(Frame(151, 1, 0x6F, 0x64, b'\xff\x62'))
        self.assertTrue(quarantine.discard(terminal))
        self.assertNotIn(0x64, quarantine.pending)
        self.assertFalse(quarantine.discard(terminal))
