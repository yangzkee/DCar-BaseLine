"""Keyboard teleoperation and telemetry dashboard for DcarON."""

import curses
import math
import time
import unicodedata

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


MOTION_KEYS = {
    curses.KEY_UP: (1.0, 0.0, 0.0), ord("w"): (1.0, 0.0, 0.0), ord("W"): (1.0, 0.0, 0.0),
    curses.KEY_DOWN: (-1.0, 0.0, 0.0), ord("s"): (-1.0, 0.0, 0.0), ord("S"): (-1.0, 0.0, 0.0),
    curses.KEY_LEFT: (0.0, 1.0, 0.0), ord("a"): (0.0, 1.0, 0.0), ord("A"): (0.0, 1.0, 0.0),
    curses.KEY_RIGHT: (0.0, -1.0, 0.0), ord("d"): (0.0, -1.0, 0.0), ord("D"): (0.0, -1.0, 0.0),
    ord("q"): (0.0, 0.0, 1.0), ord("Q"): (0.0, 0.0, 1.0),
    ord("e"): (0.0, 0.0, -1.0), ord("E"): (0.0, 0.0, -1.0),
}


def quaternion_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class ConsoleNode(Node):
    def __init__(self):
        super().__init__("dcaron_console")
        self.declare_parameter("linear_speed", 0.20)
        self.declare_parameter("angular_speed", 0.60)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        if any(not math.isfinite(value) or value <= 0 for value in (
                self.linear_speed, self.angular_speed)):
            raise ValueError("速度参数必须为有限正数")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel", qos)
        self.odom_sub = self.create_subscription(Odometry, "odom", self.on_odom, 10)
        self.odom = None
        self.odom_received_at = None
        self.command = (0.0, 0.0, 0.0)
        self.last_motion = (0.0, 0.0, 0.0)
        self.topics = []
        self.last_topic_refresh = 0.0

    def on_odom(self, message):
        self.odom = message
        self.odom_received_at = time.monotonic()

    def accept_key(self, key, now):
        # Like teleop_twist_keyboard: publish on input, never on idle.
        if key == -1 or key == curses.KEY_RESIZE:
            return
        if key in MOTION_KEYS:
            self.last_motion = MOTION_KEYS[key]
        elif key in (ord("+"), ord("=")):
            self.linear_speed = min(0.5, self.linear_speed + 0.05)
            self.angular_speed = min(1.0, self.angular_speed + 0.10)
        elif key in (ord("-"), ord("_")):
            self.linear_speed = max(0.05, self.linear_speed - 0.05)
            self.angular_speed = max(0.10, self.angular_speed - 0.10)
        else:
            self.last_motion = (0.0, 0.0, 0.0)
            if key in (ord('r'), ord('R')):
                self.last_topic_refresh = 0.0
        unit = self.last_motion
        self.command = (unit[0] * self.linear_speed, unit[1] * self.linear_speed,
                        unit[2] * self.angular_speed)
        self.publish_command(now)

    def publish_command(self, now):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = self.command
        self.cmd_pub.publish(message)

    def refresh_topics(self, now):
        if now - self.last_topic_refresh < 1.0:
            return
        self.last_topic_refresh = now
        rows = []
        for name, types in self.get_topic_names_and_types():
            if name in ("/rosout", "/parameter_events"):
                continue
            rows.append((name, ",".join(types), self.count_publishers(name),
                         self.count_subscribers(name)))
        self.topics = sorted(rows)

    def send_stop(self):
        self.command = (0.0, 0.0, 0.0)
        self.last_motion = (0.0, 0.0, 0.0)
        for _ in range(3):
            self.publish_command(time.monotonic())
            time.sleep(0.02)


def clip_columns(text, columns):
    """Clip by terminal cells, including full-width Chinese characters."""
    result, used = [], 0
    for character in text:
        size = 0 if unicodedata.combining(character) else (
            2 if unicodedata.east_asian_width(character) in ('W', 'F') else 1)
        if used + size > columns:
            break
        result.append(character)
        used += size
    return ''.join(result)


def add_line(screen, row, text, style=0):
    height, width = screen.getmaxyx()
    if row < height:
        try:
            screen.addstr(row, 0, clip_columns(text, max(0, width - 1)), style)
        except curses.error:
            pass


def cell(text, width):
    text = clip_columns(str(text), width)
    used = sum(0 if unicodedata.combining(c) else
               2 if unicodedata.east_asian_width(c) in ('W', 'F') else 1 for c in text)
    return text + ' ' * (width - used)


def draw(screen, node, now):
    screen.erase()
    height, screen_width = screen.getmaxyx()
    width = min(100, screen_width - 1)
    if width < 70 or height < 24:
        add_line(screen, 0, '请将终端放大至至少 71 列 × 24 行。', curses.A_BOLD)
        add_line(screen, 1, '窗口过小：已停止控制；空格停车，Esc 退出。')
        screen.refresh()
        return
    inner = width - 4
    row = 0

    def line(text, style=0):
        nonlocal row
        add_line(screen, row, '| ' + cell(text, inner) + ' |', style)
        row += 1

    def border(title=''):
        nonlocal row
        label = (' ' + title + ' ') if title else ''
        label_width = sum(2 if unicodedata.east_asian_width(c) in ('W', 'F') else 1 for c in label)
        add_line(screen, row, '+' + label + '-' * max(0, width - 2 - label_width) + '+', curses.A_BOLD)
        row += 1

    def columns(values):
        available = inner - 3 * (len(values) - 1)
        sizes = [available // len(values)] * len(values)
        sizes[-1] += available % len(values)
        line(' | '.join(cell(value, size) for value, size in zip(values, sizes)))

    border('DcarON 小车控制台')
    subscribers = node.count_subscribers('cmd_vel')
    line(f'控制接收节点：{subscribers}    模式：按键触发发送（标准程序方式）')
    line('松键不发停车指令；由底盘超时处理。空格/X：主动停车。')
    border('按键操作')
    columns(['W / ↑：前进', 'S / ↓：后退', '空格 / X：停车'])
    columns(['A / ←：左移', 'D / →：右移', 'Esc：停车退出'])
    columns(['Q：左转', 'E：右转', '+ / -：调节速度'])
    line('R：停车并刷新话题；其他字符：停车。鼠标不用于运动控制。')
    border('速度控制')
    columns(['项目', '前后 / 左右 (m/s)', '转向 (rad/s)'])
    columns(['上次发送', f'{node.command[0]:+.3f} / {node.command[1]:+.3f}', f'{node.command[2]:+.3f}'])
    columns(['设定速度', f'{node.linear_speed:.2f}', f'{node.angular_speed:.2f}'])
    border('底盘反馈 /odom')
    if node.odom is None:
        line('等待里程计数据……')
        line('位置：--    朝向：--')
        line('反馈速度：--')
    else:
        pose, twist = node.odom.pose.pose, node.odom.twist.twist
        age = now - node.odom_received_at
        line(f'数据更新时间：{age:.2f} 秒前' + ('（数据过期）' if age >= 1.0 else ''))
        line(f'位置：X {pose.position.x:+.3f}  Y {pose.position.y:+.3f} 米；朝向 {math.degrees(quaternion_yaw(pose.orientation)):+.1f} 度')
        line(f'速度：前后 {twist.linear.x:+.3f}  左右 {twist.linear.y:+.3f} m/s；转向 {twist.angular.z:+.3f} rad/s')
    border('ROS 话题')
    columns(['话题名称', '发布数 / 订阅数', '消息类型'])
    limit = max(0, height - row - 2)
    for name, types, publishers, subscribers in node.topics[:limit]:
        columns([name, f'{publishers} / {subscribers}', types])
    if len(node.topics) > limit:
        line(f'另有 {len(node.topics) - limit} 个话题；放大终端查看更多。')
    border()
    screen.refresh()


def run_console(screen, node):
    curses.curs_set(0)
    screen.keypad(True)
    screen.nodelay(True)
    curses.set_escdelay(25)
    too_small_before = False
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.0)
        now = time.monotonic()
        key = screen.getch()
        if key in (27, 3):
            break
        height, width = screen.getmaxyx()
        too_small = height < 24 or width < 71
        if too_small:
            if not too_small_before:
                node.send_stop()
            curses.flushinp()
        else:
            node.accept_key(key, now)
        too_small_before = too_small
        node.refresh_topics(now)
        draw(screen, node, now)
        time.sleep(0.005)




def terminal_main(args=None):
    rclpy.init(args=args)
    node = ConsoleNode()
    try:
        curses.wrapper(run_console, node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.send_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    from .web_console import main as web_main
    web_main(args)
