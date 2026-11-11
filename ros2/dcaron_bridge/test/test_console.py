from types import SimpleNamespace
import pytest

pytest.importorskip("curses")
pytest.importorskip("rclpy")

from dcaron_bridge.console import ConsoleNode, cell, draw


def fake_console():
    messages = []
    node = SimpleNamespace(linear_speed=0.2, angular_speed=0.6,
        command=(0.0, 0.0, 0.0), last_motion=(0.0, 0.0, 0.0),
        cmd_pub=SimpleNamespace(publish=messages.append))
    node.publish_command = lambda now: ConsoleNode.publish_command(node, now)
    return node, messages


def test_one_message_per_key_none_on_idle():
    node, messages = fake_console()
    ConsoleNode.accept_key(node, ord('w'), 10.0)
    for now in (10.1, 10.3, 10.8, 12.0):
        ConsoleNode.accept_key(node, -1, now)
    assert len(messages) == 1
    assert messages[-1].linear.x == 0.2
    ConsoleNode.accept_key(node, ord('w'), 12.1)
    assert len(messages) == 2


def test_stop_unknown_and_refresh():
    node, messages = fake_console()
    for key in (' ', 'x', 'k', '?', 'r'):
        ConsoleNode.accept_key(node, ord('q'), 10.0)
        assert messages[-1].angular.z == 0.6
        ConsoleNode.accept_key(node, ord(key), 10.1)
        assert messages[-1].angular.z == 0.0
        assert messages[-1].linear.x == 0.0
    assert node.last_topic_refresh == 0.0


def test_speed_change_republishes_direction():
    node, messages = fake_console()
    ConsoleNode.accept_key(node, ord('w'), 10.0)
    ConsoleNode.accept_key(node, ord('+'), 10.1)
    assert len(messages) == 2
    assert messages[-1].linear.x == 0.25
    ConsoleNode.accept_key(node, ord('s'), 10.2)
    assert messages[-1].linear.x == -0.25


def test_exit_sends_zero():
    node, messages = fake_console()
    ConsoleNode.accept_key(node, ord('w'), 10.0)
    ConsoleNode.send_stop(node)
    assert all(m.linear.x == 0.0 for m in messages[-3:])


def test_table():
    node, _ = fake_console()
    node.count_subscribers = lambda name: 1
    node.odom = None
    node.topics = []
    rows = []
    screen = SimpleNamespace(getmaxyx=lambda: (24, 80), erase=lambda: None,
        refresh=lambda: None, addstr=lambda row, col, text, style: rows.append((row, text)))
    draw(screen, node, 10.0)
    assert all(row < 24 for row, text in rows)
    assert any('上次发送' in text for row, text in rows)
    assert cell('停车', 6) == '停车  '
