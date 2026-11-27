"""Tests for rm_dashboard.web_server — the HTTP/SSE surface.

Pure stdlib (http.client against a real loopback server on an ephemeral port),
so this runs in the workstation tier with no ROS and no browser.

The properties under test are the ones that matter for a diagnostic attached
to a robot with NO PHYSICAL E-STOP:

  * a command reaches the dispatcher EXACTLY as sent, and nothing else does;
  * a dispatcher failure becomes an error response, never a server crash —
    a dashboard that dies when a service is missing is dead when you need it;
  * a stalled or vanished client cannot block the broadcast for anyone else,
    and cannot grow memory on the robot without bound.
"""

from __future__ import annotations

import http.client
import json
import os
import socket
import sys
import threading
import time

import pytest

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(_REPO_ROOT, "src", "rm_dashboard"))

from rm_dashboard.web_server import DashboardServer  # noqa: E402
from rm_dashboard import web_server as ws  # noqa: E402


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def server():
    """A running server plus the record of what reached its dispatcher."""
    state = {"topics": {}, "controls_enabled": True, "uptime_s": 0.0}
    calls = []

    def dispatch(command, payload):
        calls.append((command, payload))
        if command == "boom":
            raise RuntimeError("dispatcher exploded")
        if command == "refuse":
            return {"ok": False, "error": "refused by the node"}
        return {"ok": True, "message": f"{command} ok"}

    port = _free_port()
    srv = DashboardServer(
        snapshot_fn=lambda: state,
        dispatch_fn=dispatch,
        host="127.0.0.1",
        port=port,
        stream_hz=50.0,
    )
    srv.start()
    try:
        yield srv, port, state, calls
    finally:
        srv.stop()


def _post(port, path, body):
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request(
        "POST", path, json.dumps(body).encode(),
        {"Content-Type": "application/json"},
    )
    resp = conn.getresponse()
    out = (resp.status, json.loads(resp.read()))
    conn.close()
    return out


def _get(port, path):
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request("GET", path)
    resp = conn.getresponse()
    out = (resp.status, resp.read())
    conn.close()
    return out


def test_index_is_served_and_is_self_contained(server) -> None:
    """No CDN reference may creep in: bernard-1 has no WAN uplink, so an
    external script or stylesheet would silently not load on the robot."""
    _, port, _, _ = server
    status, body = _get(port, "/")
    assert status == 200
    text = body.decode()
    assert "<canvas" in text
    for marker in ("http://", "https://", "//cdn"):
        assert f'src="{marker}' not in text
        assert f'href="{marker}' not in text


def test_state_endpoint_returns_the_live_snapshot(server) -> None:
    _, port, state, _ = server
    state["topics"] = {"/scan": {"count": 3}}
    status, body = _get(port, "/api/state")
    assert status == 200
    assert json.loads(body)["topics"]["/scan"]["count"] == 3


def test_only_the_read_only_state_is_readable_cross_origin(server) -> None:
    """The robot wall (another origin) reads /api/state; a cross-origin page
    must never be able to POST a command or read anything else."""
    import http.client
    _srv, port, _state, _calls = server
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request("GET", "/api/state"); resp = conn.getresponse(); resp.read()
    assert resp.getheader("Access-Control-Allow-Origin") == "*"
    conn.request("GET", "/api/map"); resp = conn.getresponse(); resp.read()
    assert resp.getheader("Access-Control-Allow-Origin") is None
    conn.request("POST", "/api/kill", body=b"{}", headers={"Content-Type": "application/json"})
    resp = conn.getresponse(); resp.read()
    assert resp.getheader("Access-Control-Allow-Origin") is None
    conn.close()


def test_post_reaches_the_dispatcher_verbatim(server) -> None:
    _, port, _, calls = server
    status, body = _post(port, "/api/goal", {"x": 2.5, "y": -0.25})
    assert status == 200 and body["ok"] is True
    assert calls == [("goal", {"x": 2.5, "y": -0.25})]


def test_a_refused_command_is_a_400_with_the_node_s_own_reason(server) -> None:
    """The node's rejection reason must survive to the browser unaltered —
    a UI that reports success for a rejected parameter set is the exact bug
    this project already hit once with command_source."""
    _, port, _, _ = server
    status, body = _post(port, "/api/refuse", {})
    assert status == 400
    assert body["ok"] is False
    assert body["error"] == "refused by the node"


def test_a_throwing_dispatcher_becomes_an_error_not_a_crash(server) -> None:
    srv, port, _, _ = server
    status, body = _post(port, "/api/boom", {})
    assert status == 400 and body["ok"] is False
    assert "exploded" in body["error"]
    # And the server is still serving afterwards.
    assert _get(port, "/api/state")[0] == 200


def test_malformed_body_is_rejected_without_dispatching(server) -> None:
    """Garbage must not reach the robot-facing dispatcher at all."""
    _, port, _, calls = server
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request("POST", "/api/kill", b"{not json", {"Content-Type": "application/json"})
    resp = conn.getresponse()
    assert resp.status == 400
    conn.close()
    assert calls == []


def test_non_api_paths_are_404_and_never_dispatch(server) -> None:
    _, port, _, calls = server
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request("POST", "/kill", b"{}", {"Content-Type": "application/json"})
    assert conn.getresponse().status == 404
    conn.close()
    assert calls == []


def test_sse_stream_delivers_snapshots(server) -> None:
    _, port, state, _ = server
    state["marker"] = "hello"
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request("GET", "/events")
    resp = conn.getresponse()
    assert resp.status == 200
    assert resp.getheader("Content-Type") == "text/event-stream"

    deadline = time.time() + 5.0
    buf = b""
    while time.time() < deadline:
        buf += resp.read(1)
        if b"data:" in buf and buf.endswith(b"\n\n"):
            break
    line = [b for b in buf.split(b"\n") if b.startswith(b"data:")][0]
    assert json.loads(line[len("data:"):])["marker"] == "hello"
    conn.close()


def test_a_stalled_client_queue_is_bounded(server) -> None:
    """A backgrounded tab must not grow memory on the robot, and the newest
    frame is the one worth keeping — a live view has no use for a backlog."""
    client = ws._Client()
    for i in range(ws._CLIENT_QUEUE_MAX * 10):
        client.push(f"frame{i}")
    with client.lock:
        assert len(client.pending) == ws._CLIENT_QUEUE_MAX
        assert client.pending[-1] == f"frame{ws._CLIENT_QUEUE_MAX * 10 - 1}"


def test_a_vanished_client_does_not_stop_the_broadcast(server) -> None:
    """Open an SSE connection, abandon it, and confirm the server keeps
    serving everyone else."""
    srv, port, _, _ = server
    # A raw socket, so the disconnect can be as rude as a closed laptop lid:
    # http.client tidies up after itself, which is not the case under test.
    sock = socket.create_connection(("127.0.0.1", port), timeout=5)
    sock.sendall(b"GET /events HTTP/1.1\r\nHost: localhost\r\n\r\n")
    sock.recv(64)  # let the response start, so a client is registered
    sock.close()
    time.sleep(0.3)
    assert _get(port, "/api/state")[0] == 200
    assert _post(port, "/api/kill", {})[0] == 200


def test_a_failing_snapshot_does_not_kill_the_broadcast_thread() -> None:
    """A bad frame is skipped; the stream survives to show the next one."""
    calls = {"n": 0}

    def snapshot():
        calls["n"] += 1
        if calls["n"] < 3:
            raise ValueError("bad frame")
        return {"ok": True}

    port = _free_port()
    srv = DashboardServer(
        snapshot_fn=snapshot,
        dispatch_fn=lambda c, p: {"ok": True},
        host="127.0.0.1", port=port, stream_hz=100.0,
    )
    srv.start()
    try:
        time.sleep(0.3)
        assert calls["n"] > 3, "broadcast thread died on the first bad frame"
    finally:
        srv.stop()


def test_stop_is_idempotent_enough_to_be_safe_in_a_finally(server) -> None:
    srv, port, _, _ = server
    srv.stop()
    srv.stop()  # must not raise — destroy_node() calls it on every exit path
