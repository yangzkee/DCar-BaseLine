"""Browser key-release teleoperation with a short server-side command lease."""

import json
import math
import secrets
import threading
import time
from collections import deque
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


KEYS = {'w': (1, 0, 0), 's': (-1, 0, 0), 'a': (0, 1, 0),
        'd': (0, -1, 0), 'q': (0, 0, 1), 'e': (0, 0, -1)}


def combined_direction(keys):
    if not isinstance(keys, str) or len(keys) > 6 or any(key not in KEYS for key in keys) or len(set(keys)) != len(keys):
        raise ValueError('未知或重复按键')
    x, y, z = (sum(KEYS[key][axis] for key in keys) for axis in range(3))
    magnitude = math.hypot(x, y)
    if magnitude > 1:
        x, y = x / magnitude, y / magnitude
    return x, y, z


def measured_hz(samples, now):
    recent = [stamp for stamp in samples if now - stamp <= 2.0]
    return (len(recent) - 1) / (recent[-1] - recent[0]) if len(recent) > 1 and recent[-1] > recent[0] else 0.0


def valid_speeds(linear, angular):
    if isinstance(linear, bool) or isinstance(angular, bool):
        raise ValueError('速度必须是数值')
    linear, angular = float(linear), float(angular)
    if not (math.isfinite(linear) and 0.01 <= linear <= 0.5 and math.isfinite(angular) and 0.01 <= angular <= 1.0):
        raise ValueError('线速度范围：0.01～0.5 m/s；角速度范围：0.01～1.0 rad/s')
    return linear, angular


class HoldState:
    def __init__(self, timeout=0.25):
        self.lock = threading.Lock()
        self.timeout = timeout
        self.owner = None
        self.key = ''
        self.deadline = 0.0
        self.sequence = -1

    def update(self, owner, sequence, key, now):
        combined_direction(key)
        with self.lock:
            if self.owner != owner:
                if self.key and now < self.deadline:
                    raise ValueError('已有其他页面正在控制，请先停车')
                self.owner, self.sequence = owner, -1
            if sequence <= self.sequence:
                return  # delayed requests must never restart motion after release
            self.sequence = sequence
            self.key = key
            self.deadline = now + self.timeout

    def current(self, now):
        with self.lock:
            if now >= self.deadline:
                self.key = ''
            return self.key


class WebConsole(Node):
    def __init__(self):
        super().__init__('dcaron_console')
        self.linear = float(self.declare_parameter('linear_speed', 0.15).value)
        self.angular = float(self.declare_parameter('angular_speed', 0.4).value)
        self.http_port = int(self.declare_parameter('http_port', 8088).value)
        if not (math.isfinite(self.linear) and 0 < self.linear <= 0.5
                and math.isfinite(self.angular) and 0 < self.angular <= 1
                and 1 <= self.http_port <= 65535):
            raise ValueError('平移速度应为 0～0.5 m/s，旋转速度为 0～1 rad/s，端口为 1～65535')
        self.state = HoldState()
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
        self.odom = None
        self.odom_time = 0.0
        self.feedback_lock = threading.Lock()
        self.speed_lock = threading.Lock()
        self.control_times = deque(maxlen=150)
        self.odom_times = deque(maxlen=100)
        self.sub = self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.timer = self.create_timer(0.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_odom(self, message):
        p, v = message.pose.pose, message.twist.twist
        q = p.orientation
        yaw = math.degrees(math.atan2(2 * (q.w*q.z + q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
        with self.feedback_lock:
            self.odom_time = time.monotonic()
            self.odom_times.append(self.odom_time)
            self.odom = {'x': p.position.x, 'y': p.position.y, 'yaw': yaw,
                         'vx': v.linear.x, 'vy': v.linear.y, 'wz': v.angular.z}

    def tick(self):
        key = self.state.current(time.monotonic())
        # Never continue motion without fresh robot feedback or a command subscriber.
        with self.feedback_lock:
            fresh = self.odom is not None and time.monotonic() - self.odom_time < 1.0
        if not fresh or self.pub.get_subscription_count() == 0:
            key = ''
            with self.state.lock:
                self.state.key = ''
        self.publish(key)

    def publish(self, key):
        unit = combined_direction(key)
        message = Twist()
        with self.speed_lock:
            message.linear.x = float(unit[0] * self.linear)
            message.linear.y = float(unit[1] * self.linear)
            message.angular.z = float(unit[2] * self.angular)
        self.pub.publish(message)
        with self.feedback_lock:
            self.control_times.append(time.monotonic())

    def set_speed(self, linear, angular):
        linear, angular = valid_speeds(linear, angular)
        with self.state.lock:
            if self.state.key and time.monotonic() < self.state.deadline:
                raise ValueError('请先松开方向控制，再调整速度')
            with self.speed_lock:
                self.linear, self.angular = linear, angular

    def feedback(self):
        with self.feedback_lock:
            now = time.monotonic()
            data = {'odom': self.odom, 'age': now - self.odom_time if self.odom else None,
                    'control_hz': measured_hz(self.control_times, now),
                    'odom_hz': measured_hz(self.odom_times, now), 'control_target_hz': 50.0,
                    'heartbeat_target_hz': 1000.0 / 60.0}
        with self.speed_lock:
            data.update(linear=self.linear, angular=self.angular)
        data.update(subscribers=self.pub.get_subscription_count(), key=self.state.current(now))
        return data


def make_handler(node, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def respond(self, code, data, kind='application/json; charset=utf-8'):
            body = data.encode('utf-8')
            self.send_response(code)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Frame-Options', 'DENY')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/':
                self.respond(200, PAGE.replace('__TOKEN__', token), 'text/html; charset=utf-8')
            elif self.path == '/status':
                self.respond(200, json.dumps(node.feedback()))
            else:
                self.respond(404, '{}')

        def do_POST(self):
            try:
                if self.path not in ('/command', '/speed'):
                    self.respond(404, '{}')
                    return
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 2048:
                    raise ValueError('请求长度错误')
                data = json.loads(self.rfile.read(length))
                if not isinstance(data, dict):
                    raise ValueError('请求格式错误')
                if not secrets.compare_digest(str(data.get('token', '')), token):
                    self.respond(403, '{}')
                    return
                if self.path == '/speed':
                    node.set_speed(data['linear'], data['angular'])
                    self.respond(200, json.dumps(node.feedback()))
                    return
                owner, seq = data['owner'], data['sequence']
                if not isinstance(owner, str) or not isinstance(seq, int) or seq < 0:
                    raise ValueError('请求格式错误')
                node.state.update(owner, seq, data['key'], time.monotonic())
                self.respond(200, '{}')
            except (ValueError, KeyError, TypeError) as exc:
                self.respond(409, json.dumps({'error': str(exc)}, ensure_ascii=False))
    return Handler


PAGE = Path(__file__).with_name('web_panel.html').read_text(encoding='utf-8')


def main(args=None):
    # Keep the ROS context alive through Ctrl+C so cleanup can publish zero.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = WebConsole()
    server = None
    try:
        server = ThreadingHTTPServer(('0.0.0.0', node.http_port), make_handler(node, secrets.token_hex(24)))
        server.daemon_threads = True
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        print(f'中文按住控制面板已启动，请在电脑浏览器打开 http://<ROS 主机 IP>:{node.http_port}', flush=True)
        print('按住才运动，松开/失焦发零速度；空格/X/Esc 停车。Ctrl+C 停止服务。', flush=True)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except OSError as exc:
        node.get_logger().error(f'控制面板端口无法打开：{exc}；可用 --ros-args -p http_port:=8089 更换端口。')
    finally:
        if server:
            server.shutdown()
            server.server_close()
        for _ in range(3):
            node.publish('')
            time.sleep(0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
