import unittest
from dcaron_bridge.protocol import Frame, Parser, frame
from dcaron_bridge.telemetry import TelemetryReceiver
from test_protocol import odom_packet


class TelemetryTests(unittest.TestCase):
    def test_fragmented_and_coalesced_odom(self):
        r, parser = TelemetryReceiver(), Parser()
        p=odom_packet()
        for byte in frame(p.a,p.b,p.payload,151,1)*2:
            for packet in parser.feed(bytes([byte])):
                r.receive(packet,10)
        self.assertEqual(set(r.streams), {'odom'})
        self.assertEqual(r.streams['odom'].ok,2)
        self.assertEqual(r.preferred(10),'odom')

    def test_absence_stale_recovery_and_rejected_layout(self):
        r=TelemetryReceiver()
        self.assertIsNone(r.preferred(0))
        r.receive(odom_packet(),1)
        self.assertEqual(r.preferred(1),'odom')
        self.assertIsNone(r.preferred(2))
        self.assertIsNone(r.receive(Frame(151,1,0x6C,0x81,bytes(35)),2))
        self.assertEqual(r.streams['odom'].wire,1)
        r.receive(odom_packet(),3)
        self.assertEqual(r.preferred(3),'odom')
        r.receive(Frame(151,1,0x6C,0x80,b'\x04'),4)
        self.assertEqual(r.streams['odom'].ok,2)
        self.assertEqual(r.streams['odom'].rejected,1)
        self.assertIsNone(r.preferred(4))
