import unittest,struct,math
from dcaron_bridge.motion import MotionSpec
from dcaron_bridge.protocol import Parser,frame
from dcaron_bridge.queue import QueueClient,encode,validate

def reply(p,n,i):return Parser().feed(frame(0x6f,0x85,struct.pack('<BBH',p,n,i),151,1))[0]
class QueueTests(unittest.TestCase):
    def test_unified_line_yaw_and_all_grades(self):
        # Both public command names must use the same 0x64 wire contract.
        # Keep grade outside the signed yaw field, including a full reverse turn.
        for grade in range(16,21):
            for yaw in (0.0, -math.pi/2, 2*math.pi):
                packets=[encode(MotionSpec(command,x=.3,y=-.1,speed=.15,
                                          yaw=yaw,profile=grade),42)
                         for command in (100,101)]
                self.assertEqual(packets[0],packets[1])
                f=Parser().feed(packets[0])[0]
                expected=struct.pack('<iii',3000,-1000,1500)
                if yaw: expected+=struct.pack('<i',int(yaw*10000))
                self.assertEqual((f.b,f.payload),(0x64,expected+struct.pack('<BH',grade,42)))

    def test_successor_admitted_before_first_done(self):
        q,tx=self.ready([MotionSpec(100,x=.3,speed=.15,profile=18),
                        MotionSpec(100,y=.3,yaw=math.pi/2,speed=.15,profile=18)])
        q.tick(.02);q.receive(reply(0,0x10,65535));q.tick(.03)
        self.assertEqual(q.cursor,2)
        self.assertFalse(q.completed)
        self.assertEqual(set(q.pending),{65535,1})
        self.assertEqual(Parser().feed(tx[-1])[0].b,0x64)

    def ready(self,specs=None,sequential=False):
        tx=[];q=QueueClient(specs or [MotionSpec(100,x=.1,speed=.1,profile=18)]*3,tx.append,sequential=sequential)
        q.tick(0);q.receive(reply(16,0x21,6));q.tick(.01);q.receive(reply(16,0x28,65535));return q,tx
    def test_wire(self):
        f=Parser().feed(encode(MotionSpec(100,x=.1,speed=.06,profile=18),7))[0]
        self.assertEqual(f.payload,struct.pack('<iiiBH',1000,0,600,18,7))
        f=Parser().feed(encode(MotionSpec(101,x=-.6,speed=.15,yaw=2*math.pi,profile=18),8))[0]
        self.assertEqual((f.b,len(f.payload)),(100,19));self.assertEqual(struct.unpack('<iiiiBH',f.payload),(-6000,0,1500,62831,18,8))
        f=Parser().feed(encode(MotionSpec(102,radius=.3,yaw=math.pi,speed=-.15,profile=18),9))[0]
        self.assertEqual(struct.unpack('<iiiBH',f.payload)[2],-1500)
    def test_rollover_duplicate_and_trim(self):
        q,tx=self.ready();q.tick(.02);first=tx[-1];q.tick(.6);self.assertEqual(tx[-1],first)
        q.receive(reply(1,0,65535));q.tick(.61);self.assertIn(1,q.pending)
        q.receive(reply(255,0,65535));q.receive(reply(255,0,65535));self.assertEqual(len(q.completed),1)
        q.receive(reply(0,0x10,1));q.tick(.62);q.receive(reply(0,0x10,2))
        q.receive(reply(254,0x25,1));self.assertFalse(q.done)
        q.receive(reply(255,0x27,1));q.receive(reply(255,0,2));self.assertTrue(q.done);self.assertEqual(q.final_notice,0x27)
    def test_sequential_and_result_query(self):
        q,tx=self.ready(sequential=True);q.tick(.02);q.receive(reply(0,0x10,65535));q.tick(.2);self.assertEqual(q.cursor,1)
        q.tick(1.1);f=Parser().feed(tx[-1])[0];self.assertEqual(f.payload,b'\x05\xff\xff')
        q.receive(reply(255,0,65535));q.tick(1.2);self.assertEqual(q.cursor,2)
    def test_full_fault_and_capability(self):
        q,tx=self.ready();q.tick(.02);first=tx[-1];q.receive(reply(255,0x11,65535));q.tick(.6);self.assertEqual(first,tx[-1]);self.assertEqual(q.cursor,1)
        with self.assertRaises(ValueError):q.receive(reply(255,0x26,65535))
        q=QueueClient([MotionSpec(100,x=.1,speed=.1,profile=18)],lambda _:None)
        with self.assertRaises(ValueError):q.receive(reply(16,0x21,5))
    def test_wrong_address_and_id_ignored(self):
        q,tx=self.ready();q.tick(.02);q.receive(reply(255,0,7));q.receive(reply(255,0,0));self.assertFalse(q.completed)
        packet=Parser().feed(frame(0x6f,0x85,struct.pack('<BBH',255,0,65535),151,2))[0];q.receive(packet);self.assertFalse(q.completed)
    def test_validation_and_no_response(self):
        for spec in [MotionSpec(100,speed=math.nan,profile=18),MotionSpec(100,speed=.1,profile=2),MotionSpec(102,radius=-1,speed=.1,profile=18)]:
            with self.assertRaises(ValueError):encode(spec,1)
        q=QueueClient([MotionSpec(100,x=.1,speed=.1,profile=18)],lambda _:None)
        q.tick(0);q.tick(.5);q.tick(1)
        with self.assertRaises(TimeoutError):q.tick(1.5)
    def test_v7_warnings_continue_and_survive_final_success(self):
        for notice in (0x24,0x26):
            tx=[];q=QueueClient([MotionSpec(100,x=.1,speed=.1,profile=18)]*2,tx.append)
            q.tick(0);q.receive(reply(16,0x21,7));q.tick(.01);q.receive(reply(16,0x28,1));q.tick(.02)
            q.receive(reply(255,notice,1)) # terminal can arrive before admission ACK
            self.assertEqual(q.feedback,(1,0,255,notice))
            q.receive(reply(0,0x10,1));q.tick(.03)
            self.assertIn(2,q.pending)
            q.receive(reply(255,0x27,2))
            self.assertTrue(q.done);self.assertEqual(q.final_notice,notice)
    def test_v7_nonterminal_warning_and_other_fault_still_fail(self):
        for process,notice in ((254,0x24),(254,0x26),(255,0x22),(255,0x23)):
            q=QueueClient([MotionSpec(100,x=.1,speed=.1,profile=18)],lambda _:None)
            q.tick(0);q.receive(reply(16,0x21,7));q.tick(.01);q.receive(reply(16,0x28,1));q.tick(.02)
            with self.assertRaises(ValueError):q.receive(reply(process,notice,1))
if __name__=='__main__':unittest.main()
