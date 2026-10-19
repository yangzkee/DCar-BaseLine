"""DFLink queue v6: bounded lookahead, automatic u16 IDs, idempotent retry."""
import math
import struct
from .protocol import frame
from .motion import fixed

GRADES = (16, 17, 18, 19, 20)
SUCCESS = (0, 0x27)

class QueueFault(ValueError):
    def __init__(self, message, notice):
        super().__init__(message)
        self.notice = notice

def encode(spec, task_id, target=1, source=151):
    if not 1 <= task_id <= 65535 or spec.profile not in GRADES:
        raise ValueError('Queue requires nonzero u16 ID and grade 16..20')
    fixed(spec.x, spec.y, spec.yaw, spec.speed, spec.angular_speed, spec.radius)
    command = spec.command
    if command == 0x63:
        if spec.angular_speed <= 0: raise ValueError('Positive angular speed required')
        payload = fixed(spec.yaw, spec.angular_speed)
    elif command in (0x64, 0x65):
        if spec.speed <= 0: raise ValueError('Positive translation speed required')
        command = 0x64
        payload = fixed(spec.x, spec.y, spec.speed)
        if spec.yaw != 0: payload += fixed(spec.yaw)
        payload += bytes((spec.profile,))
    elif command == 0x66:
        if spec.radius <= 0 or spec.speed == 0: raise ValueError('Positive radius and nonzero signed speed required')
        payload = fixed(spec.radius, spec.yaw, spec.speed) + bytes((spec.profile,))
    else: raise ValueError('Only rotate, line and arc support queued motion')
    return frame(2, command, payload + struct.pack('<H', task_id), target, source)

def validate(spec, cfg):
    encode(spec, 1)
    if (not math.isfinite(spec.timeout) or not 0 <= spec.timeout <= cfg['max_motion_timeout']
        or math.hypot(spec.x, spec.y) > cfg['max_displacement']
        or abs(spec.yaw) > cfg['max_rotation']
        or abs(spec.speed) > cfg['max_linear_speed']
        or spec.angular_speed > cfg['max_angular_speed']):
        raise ValueError('Motion exceeds configured limits')
    if spec.command == 0x66 and (spec.radius * abs(spec.yaw) > cfg['max_displacement']
                                 or abs(spec.speed) / spec.radius > cfg['max_angular_speed']):
        raise ValueError('Arc exceeds configured limits')

class QueueClient:
    """Caller owns serial, deadlines, cancellation and monotonic clock; no IO threads."""
    def __init__(self, specs, send, target=1, source=151, sequential=False):
        if not 1 <= len(specs) <= 128: raise ValueError('Use 1..128 segments')
        for spec in specs: encode(spec, 1, target, source)
        self.specs, self.send, self.target, self.source = list(specs), send, target, source
        self.window = 1 if sequential else 8
        self.state, self.next_id, self.cursor = 'probe', 0, 0
        self.pending, self.completed = {}, set()
        self.last, self.attempts, self.final_notice = -1e9, 0, 0
        self.feedback = None
        self.motion_started = False

    @property
    def done(self): return self.state == 'done'

    def management(self, op, task_id=None):
        return frame(2, 0x85, bytes((op,)) + (struct.pack('<H', task_id) if task_id is not None else b''), self.target, self.source)

    def tick(self, now):
        if self.state in ('probe', 'sync'):
            if now-self.last >= .5:
                if self.attempts >= 3: raise TimeoutError('Queue capability/ID query unanswered')
                self.send(self.management(0 if self.state == 'probe' else 7))
                self.last, self.attempts = now, self.attempts+1
            return
        if self.done: return
        # Admission retries reuse the exact bytes and ID; accepted tasks use result queries.
        for task_id, item in list(self.pending.items()):
            if now-item['last'] < (.5 if not item['accepted'] else 1.0): continue
            if not item['accepted'] and item['attempts'] >= 3:
                raise TimeoutError('Motion admission unanswered')
            self.send(self.management(5, task_id) if item['accepted'] else item['frame'])
            item['last'], item['attempts'] = now, item['attempts']+1
        # Keep at most one admission unconfirmed; prevents consuming an ID after Full.
        if any(not x['accepted'] for x in self.pending.values()): return
        if self.cursor < len(self.specs) and len(self.pending) < self.window:
            task_id = self.next_id
            payload = encode(self.specs[self.cursor], task_id, self.target, self.source)
            self.pending[task_id] = dict(frame=payload, accepted=False, attempts=1, last=now, index=self.cursor)
            self.next_id = task_id % 65535 + 1
            self.cursor += 1
            self.motion_started = True
            self.send(payload)

    def receive(self, packet):
        if (packet.target, packet.source, packet.a, packet.b, len(packet.payload)) != (self.source, self.target, 0x6f, 0x85, 4): return
        process, notice, task_id = struct.unpack('<BBH', packet.payload)
        if self.state == 'probe':
            if notice == 0x21:
                if task_id < 6: raise ValueError('Beta requires queue v6 firmware; no legacy fallback')
                self.state, self.last, self.attempts = 'sync', -1e9, 0
            return
        if self.state == 'sync':
            if notice == 0x28 and task_id:
                if process != 16: raise ValueError('Device queue is occupied by another sender')
                self.next_id, self.state = task_id, 'run'
            return
        if self.state != 'run' or task_id not in self.pending: return
        item = self.pending[task_id]
        if notice == 0x11:
            # Full is not terminal. Retain this ID and retry with bounded deadline at caller.
            item['attempts'] = 0
            return
        if notice not in (0, 0x10, 0x25, 0x27):
            raise QueueFault(f'Task {task_id} rejected: notice=0x{notice:02x}', notice)
        item['accepted'] = True
        self.feedback = (task_id, item['index'], process, notice)
        if process == 255:
            if notice not in SUCCESS: raise ValueError('Invalid terminal notice')
            self.completed.add(task_id)
            del self.pending[task_id]
            if notice == 0x27: self.final_notice = notice
            if len(self.completed) == len(self.specs): self.state = 'done'
