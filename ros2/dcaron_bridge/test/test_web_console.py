import json
import threading
from collections import deque
from http.server import ThreadingHTTPServer
from types import SimpleNamespace
from urllib.request import Request, urlopen
from urllib.error import HTTPError

import pytest

pytest.importorskip("rclpy")

from dcaron_bridge.web_console import HoldState, WebConsole, make_handler, valid_speeds, measured_hz, combined_direction


def test_release_and_delayed_heartbeat():
    state = HoldState()
    state.update('a', 1, 'w', 10)
    assert state.current(10.1) == 'w'
    state.update('a', 3, '', 10.12)
    state.update('a', 2, 'w', 10.13)
    assert state.current(10.14) == ''


def test_hold_heartbeat_and_disconnect():
    state = HoldState()
    for i in range(20):
        state.update('a', i, 'w', 10 + i * 0.06)
        assert state.current(10 + i * 0.06 + 0.04) == 'w'
    assert state.current(12) == ''


def test_only_one_controller():
    state = HoldState()
    state.update('a', 1, 'w', 10)
    with pytest.raises(ValueError):
        state.update('b', 1, 's', 10.1)
    state.update('b', 1, 's', 11)
    assert state.current(11.1) == 's'


def test_wire_velocity_and_zero():
    messages = []
    node = SimpleNamespace(linear=0.15, angular=0.4,
                           pub=SimpleNamespace(publish=messages.append),
                           speed_lock=threading.Lock(), feedback_lock=threading.Lock(), control_times=deque())
    WebConsole.publish(node, 'w')
    WebConsole.publish(node, '')
    assert messages[0].linear.x == 0.15
    assert messages[1].linear.x == 0.0
    assert messages[1].angular.z == 0.0


def test_http_authentication_release_and_sequence():
    node = SimpleNamespace(state=HoldState(), feedback=lambda: {'odom': None})
    server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(node, 'secret'))
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f'http://127.0.0.1:{server.server_port}'
    def send(sequence, key, token='secret'):
        data = json.dumps(dict(owner='test', sequence=sequence, key=key, token=token)).encode()
        with urlopen(Request(base + '/command', data=data), timeout=2) as response:
            return response.status
    try:
        with pytest.raises(HTTPError) as exc:
            send(1, 'w', 'wrong')
        assert exc.value.code == 403
        assert send(1, 'w') == 200
        assert send(3, '') == 200
        assert send(2, 'w') == 200
        assert node.state.key == ''
        with urlopen(base, timeout=2) as response:
            page = response.read().decode()
        for event in ('keyup', 'pointerup', 'blur', 'pagehide', 'visibilitychange'):
            assert event in page
    finally:
        server.shutdown()
        server.server_close()


def test_speed_limits_and_pause_before_change():
    assert valid_speeds(0.25, 0.6) == (0.25, 0.6)
    assert valid_speeds(0.01, 0.01) == (0.01, 0.01)
    for linear, angular in [(0, 0.4), (0.005, 0.4), (0.1, 0.005), (0.6, 0.4), (0.1, 2), (float('nan'), 0.4), (True, 0.4)]:
        with pytest.raises(ValueError):
            valid_speeds(linear, angular)
    node = SimpleNamespace(state=HoldState(), speed_lock=threading.Lock(), linear=0.1, angular=0.2)
    WebConsole.set_speed(node, 0.25, 0.6)
    assert node.linear == 0.25
    import time
    node.state.update('a', 1, 'w', time.monotonic())
    with pytest.raises(ValueError):
        WebConsole.set_speed(node, 0.4, 0.8)


def test_measured_frequencies_and_stale_window():
    assert measured_hz([1, 1.02, 1.04, 1.06], 1.1) == pytest.approx(50)
    assert measured_hz([1, 1.1, 1.2], 1.3) == pytest.approx(10)
    assert measured_hz([1, 1.1], 5) == 0


def test_combinations_cancel_and_normalize():
    assert combined_direction('qw') == (1, 0, 1)
    assert combined_direction('ew') == (1, 0, -1)
    assert combined_direction('wsqe') == (0, 0, 0)
    x, y, z = combined_direction('awq')
    assert x == pytest.approx(2 ** -0.5)
    assert y == pytest.approx(2 ** -0.5)
    assert z == 1
    for keys in ('ww', 'bad', None, ['w']):
        with pytest.raises(ValueError):
            combined_direction(keys)


def test_wire_translation_and_rotation_together():
    messages = []
    node = SimpleNamespace(linear=0.15, angular=0.4,
                           pub=SimpleNamespace(publish=messages.append),
                           speed_lock=threading.Lock(), feedback_lock=threading.Lock(), control_times=deque())
    WebConsole.publish(node, 'qw')
    assert messages[-1].linear.x == 0.15
    assert messages[-1].angular.z == 0.4
    WebConsole.publish(node, 'w')
    assert messages[-1].linear.x == 0.15
    assert messages[-1].angular.z == 0.0
    WebConsole.publish(node, '')
    assert messages[-1].linear.x == messages[-1].angular.z == 0.0
