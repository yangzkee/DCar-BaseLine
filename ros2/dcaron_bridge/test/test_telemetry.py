import unittest
from dcaron_bridge.telemetry import TelemetryReceiver
from dcaron_bridge.protocol import Frame
from test_protocol import odom_packet, velpos_packet
class TelemetryTests(unittest.TestCase):
 def test_unified_stream_and_stale_recovery(self):
  r=TelemetryReceiver();self.assertIsNone(r.preferred(0))
  self.assertIsNone(r.receive(velpos_packet(),1))
  self.assertEqual(r.receive(odom_packet(),1),'odom')
  self.assertEqual(r.preferred(1.1),'odom');self.assertIsNone(r.preferred(2.1))
  r.receive(odom_packet(),3);self.assertEqual(r.preferred(3),'odom')
  self.assertIsNone(r.receive(Frame(151,1,0x6C,0x80,b'\x03'),4))
  self.assertEqual(r.streams['odom'].ok,2);self.assertEqual(r.streams['odom'].rejected,1)
