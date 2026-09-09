import math
import random
import struct
import unittest

from dcaron_bridge.protocol import (Parser, Frame, frame, subscribe, velocity,
                                    decode, quaternion, CommandWatchdog)


def odom_packet():
    # Independent fixture: distinct values expose offsets and scale mix-ups.
    payload = b'\x04' + struct.pack('<15h3i2I',
        1000, -2000, 3000, 400, -800, 3924, 180, -360, 900,
        500, -1000, 1500, 2000, -2500, 3000,
        123456, -234567, 345678, 1234, 567)
    return Frame(151, 1, 0x6C, 0x80, payload)


def velpos_packet():
    payload = b'\x02' + struct.pack('<7h3i2I',
        -10000, 500, -1000, 1500, 2000, -2500, 3000,
        123456, -234567, 345678, 1234, 567)
    return Frame(151, 1, 0x6C, 0x81, payload)


class ProtocolTests(unittest.TestCase):
    def test_published_manual_golden_frame(self):
        self.assertEqual(frame(4, 0x6A, bytes((1, 10))),
                         bytes.fromhex('DF 01 97 04 6A 02 01 0A FD EF 02'))

    def test_subscription_codes(self):
        self.assertEqual(subscribe('velpos', 10), bytes.fromhex('DF 01 97 04 81 02 01 01 FD FD 02'))
        self.assertEqual(subscribe('odom', 100)[6:8], b'\x01\x0a')
        self.assertEqual(subscribe('odom', continuous=False)[6], 2)
        with self.assertRaises(ValueError):
            subscribe('odom', 20)

    def test_velocity_wire_units_and_sign(self):
        packet = velocity(0.3, -0.2, 1.0)
        self.assertEqual(packet[:6], bytes.fromhex('DF 01 97 02 62 0C'))
        self.assertEqual(packet[6:18], bytes.fromhex('B8 0B 00 00 30 F8 FF FF 10 27 00 00'))
        for bad in (math.nan, math.inf, -math.inf, 300000):
            with self.assertRaises(ValueError):
                velocity(bad, 0, 0)

    def test_noise_corruption_fragmentation_and_coalescing(self):
        good = frame(0x6C, 0x81, velpos_packet().payload, 151, 1)
        corrupted = bytearray(good)
        corrupted[-1] ^= 0x80
        parser = Parser()
        stream = b'printf\r\n' + bytes(corrupted) + b'\xdf\x97\x01\x6c\x81\xff' + good + good
        rng = random.Random(7)
        packets = []
        while stream:
            size = rng.randint(1, 12)
            packets.extend(parser.feed(stream[:size]))
            stream = stream[size:]
        self.assertEqual(packets, [velpos_packet(), velpos_packet()])
        self.assertEqual(len(parser.buffer), 0)

    def test_payload_with_frame_markers_and_max_length(self):
        payload = bytes(range(255))
        packet = frame(3, 4, payload)
        parser = Parser()
        self.assertEqual(parser.feed(packet[:100]), [])
        self.assertEqual(parser.feed(packet[100:])[0].payload, payload)
        parser.feed(b'\xdf' * 10000)
        self.assertLessEqual(len(parser.buffer), 263)

    def test_tail_required_even_with_valid_checksum(self):
        body = frame(3, 4)[:-3] + b'\x00'
        self.assertEqual(Parser().feed(body + struct.pack('<H', sum(body))), [])

    def test_odom_layout_and_scales(self):
        state = decode(odom_packet())
        self.assertEqual(state.rpy, (0.1, -0.2, 0.3))
        self.assertEqual(state.acceleration, (1.0, -2.0, 9.81))
        self.assertEqual(state.gyro, (0.1, -0.2, 0.5))
        self.assertEqual(state.body_velocity, (0.1, -0.2, 0.3))
        self.assertEqual(state.position, (123.456, -234.567, 345.678))
        self.assertEqual(state.device_time, (1234, 567))

    def test_velpos_missing_imu_is_not_zero_measurement(self):
        state = decode(velpos_packet())
        self.assertEqual(state.rpy, (0.0, 0.0, -1.0))
        self.assertEqual(state.body_velocity, (0.1, -0.2, 0.3))
        self.assertEqual(state.position, (123.456, -234.567, 345.678))
        self.assertIsNone(state.gyro)
        self.assertIsNone(state.acceleration)

    def test_wrong_versions_lengths_and_classes_rejected(self):
        original = odom_packet()
        for payload in (b'\x03' + original.payload[1:], original.payload[:-1], original.payload + b'\x00', b''):
            with self.assertRaises(ValueError):
                decode(Frame(151, 1, 0x6C, 0x80, payload))
        with self.assertRaises(ValueError):
            decode(Frame(151, 1, 0x6F, 0x80, original.payload))

    def test_quaternion(self):
        self.assertEqual(quaternion((0, 0, 0)), (0.0, 0.0, 0.0, 1.0))
        q = quaternion((0, 0, math.pi / 2))
        self.assertAlmostEqual(q[2], math.sqrt(0.5))
        self.assertAlmostEqual(sum(v * v for v in q), 1.0)

    def test_watchdog_boundary_and_reconnect(self):
        watchdog = CommandWatchdog(0.5)
        self.assertFalse(watchdog.expired(100))
        watchdog.received(10)
        self.assertFalse(watchdog.expired(10.499))
        self.assertTrue(watchdog.expired(10.5))
        self.assertFalse(watchdog.expired(11))
        watchdog.received(20)
        watchdog.reset()
        self.assertFalse(watchdog.expired(100))


if __name__ == '__main__':
    unittest.main()
