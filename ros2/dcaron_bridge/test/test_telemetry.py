import unittest
from dcaron_bridge.protocol import Frame, Parser, frame
from dcaron_bridge.telemetry import TelemetryReceiver
from test_protocol import odom_packet, velpos_packet


class TelemetryTests(unittest.TestCase):
    def test_independent_counts_both_orders_and_byte_fragments(self):
        for packets in [(odom_packet(), velpos_packet()), (velpos_packet(), odom_packet())]:
            receiver, parser = TelemetryReceiver(), Parser()
            wire = b''.join(frame(p.a, p.b, p.payload, 151, 1) for p in packets)
            for byte in wire:
                for packet in parser.feed(bytes([byte])):
                    receiver.receive(packet, 10)
            self.assertEqual([s.ok for s in receiver.streams.values()], [1, 1])
            self.assertNotEqual(receiver.streams['odom'].state, receiver.streams['velpos'].state)
            self.assertEqual(receiver.preferred(10), 'odom')

    def test_absence_stale_recovery_and_rejected_layout(self):
        r = TelemetryReceiver()
        self.assertIsNone(r.preferred(0))
        r.receive(velpos_packet(), 1)
        self.assertEqual(r.preferred(1), 'velpos')
        r.receive(odom_packet(), 1.1)
        self.assertEqual(r.preferred(1.2), 'odom')
        r.receive(velpos_packet(), 2.2)
        self.assertEqual(r.preferred(2.2), 'velpos')
        self.assertIsNone(r.preferred(3.3))
        r.receive(odom_packet(), 4)
        self.assertEqual(r.preferred(4), 'odom')
        bad = Frame(151, 1, 0x6C, 0x81, b'\x04')
        self.assertIsNone(r.receive(bad, 5))
        self.assertEqual(r.streams['velpos'].wire, 3)
        self.assertEqual(r.streams['velpos'].ok, 2)
        self.assertEqual(r.streams['velpos'].rejected, 1)
        self.assertEqual(r.streams['velpos'].version, 4)
