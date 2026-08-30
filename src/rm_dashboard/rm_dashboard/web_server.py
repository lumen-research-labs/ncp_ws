"""
web_server.py — the dashboard's HTTP surface: one page, one event stream, a
handful of command endpoints. ROS-free.

What it does:
    Serves `static/index.html` at `/`, a Server-Sent Events stream of state
    snapshots at `/events`, a one-shot snapshot at `/api/state`, the SLAM
    occupancy grid at `/api/map`, and POSTs to `/api/<command>` which it
    forwards to a caller-supplied dispatch function. Runs on
    `http.server.ThreadingHTTPServer` in a daemon thread.

    THE MAP IS A SEPARATE ENDPOINT, not part of the snapshot, and that split
    is deliberate. The snapshot goes out at 10 Hz; a SLAM occupancy grid is
    hundreds of kilobytes and changes every couple of seconds. Putting it on
    the stream would multiply the robot's outbound traffic by two orders of
    magnitude to redraw a picture that had not changed. The browser polls this
    on its own slow schedule and skips the fetch entirely when the sequence
    number in the snapshot has not moved.

    STANDARD LIBRARY ONLY, and that is a hard requirement rather than a
    preference: bernard-1 has no internet (RobotLab has no WAN uplink), so
    anything needing `pip install` cannot be deployed to the robot at all.
    That rules out the obvious choices — flask, websockets, rosbridge — and
    leaves SSE over plain HTTP, which is a good fit anyway: the traffic is
    one-way state at a fixed rate, browsers reconnect dropped SSE streams on
    their own, and there is no handshake to get wrong.

    The page is served from the robot rather than the workstation on purpose.
    It only needs TCP to reach the browser, so it works whenever SSH does —
    whereas anything workstation-side would depend on DDS crossing the lab
    network, which is the thing this project has repeatedly had to debug (the
    multicast/`maxInitialPeersRange` traps). A dashboard that fails in the
    conditions you would open it in is not worth having.

Subscribes to:  (nothing — ROS-free; reads snapshots from a StateStore)
Publishes:      (nothing — commands go out through the injected dispatcher,
                 which is the ONLY path from this module to the robot)

Failure mode:
    Fail-quiet toward the robot, loud toward the browser. A dropped or slow
    client is dropped from the broadcast list and never blocks the robot: the
    ROS thread does not touch this module, and every socket write is guarded.
    An exception inside a command dispatch is caught and returned to the
    browser as an error payload rather than propagating into the server
    thread. The server can therefore stop working without the robot noticing,
    which is the correct direction for a diagnostic to fail — the reverse,
    where a browser tab can wedge the control stack, is what this arrangement
    exists to prevent.
"""

from __future__ import annotations

import json
import pathlib
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Callable, Dict, Optional

# Snapshot broadcast rate. 10 Hz is above the 5 Hz control loop, so no policy
# tick is ever missed, and well below the 20 Hz safety loop — a browser does
# not need every safety tick, and at 20 Hz the JSON (which carries a ~2000
# point scan) becomes a meaningful fraction of the Jetson's time for no
# diagnostic gain.
DEFAULT_STREAM_HZ = 10.0

# Bound on the queue of pending snapshots per connected client. A browser tab
# that stops reading (backgrounded, throttled, laptop asleep) must not grow
# this without limit on the robot. When it overflows, the OLDEST snapshots are
# dropped: a live view wants the newest state, and delivering a backlog of
# stale frames after a stall is worse than skipping them.
_CLIENT_QUEUE_MAX = 4

_STATIC_DIR = pathlib.Path(__file__).parent / "static"


class _Client:
    """One connected SSE consumer."""

    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.pending: list = []
        self.event = threading.Event()
        self.closed = False

    def push(self, payload: str) -> None:
        with self.lock:
            if self.closed:
                return
            self.pending.append(payload)
            # Drop from the FRONT — keep the newest.
            while len(self.pending) > _CLIENT_QUEUE_MAX:
                self.pending.pop(0)
        self.event.set()

    def close(self) -> None:
        with self.lock:
            self.closed = True
        self.event.set()


class DashboardServer:
    """Owns the HTTP server thread and the SSE broadcast loop."""

    def __init__(
        self,
        snapshot_fn: Callable[[], Dict[str, Any]],
        dispatch_fn: Callable[[str, Dict[str, Any]], Dict[str, Any]],
        host: str = "0.0.0.0",
        port: int = 8080,
        stream_hz: float = DEFAULT_STREAM_HZ,
        logger: Optional[Callable[[str], None]] = None,
        map_fn: Optional[Callable[[], Dict[str, Any]]] = None,
    ) -> None:
        """
        map_fn:
            Returns the current occupancy-grid payload for GET /api/map.
            Optional: omitted, the endpoint reports the map as unavailable
            rather than 404ing, so a page written against a newer node still
            renders correctly against an older one.
        snapshot_fn:
            Returns the current JSON-serializable state. Called on the
            broadcast thread at `stream_hz`.
        dispatch_fn:
            `(command, payload) -> result dict`. The only route from a browser
            to the robot. Everything it is allowed to do is decided by the
            node that supplies it, not here — this module deliberately holds
            no list of permitted commands, so the security-relevant decision
            lives in one place next to the ROS handles rather than being
            duplicated in an HTTP router that could drift out of step.
        """
        self._snapshot_fn = snapshot_fn
        self._dispatch_fn = dispatch_fn
        self._map_fn = map_fn
        self._host = host
        self._port = port
        self._stream_period = 1.0 / stream_hz
        self._log = logger or (lambda msg: None)

        self._clients: list = []
        self._clients_lock = threading.Lock()
        self._stop = threading.Event()

        self._httpd: Optional[ThreadingHTTPServer] = None
        self._http_thread: Optional[threading.Thread] = None
        self._broadcast_thread: Optional[threading.Thread] = None

    # -- lifecycle ---------------------------------------------------------

    def start(self) -> None:
        handler = _make_handler(self)
        # ThreadingHTTPServer: SSE holds a connection open for the life of the
        # tab, so a single-threaded server would serve exactly one client and
        # then appear hung to the second.
        self._httpd = ThreadingHTTPServer((self._host, self._port), handler)
        self._httpd.daemon_threads = True
        self._http_thread = threading.Thread(
            target=self._httpd.serve_forever, name="dashboard-http", daemon=True
        )
        self._http_thread.start()
        self._broadcast_thread = threading.Thread(
            target=self._broadcast_loop, name="dashboard-sse", daemon=True
        )
        self._broadcast_thread.start()
        self._log(f"dashboard serving on http://{self._host}:{self._port}/")

    def stop(self) -> None:
        self._stop.set()
        with self._clients_lock:
            for client in self._clients:
                client.close()
            self._clients.clear()
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()

    # -- SSE ---------------------------------------------------------------

    def _broadcast_loop(self) -> None:
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                payload = json.dumps(self._snapshot_fn())
            except Exception as exc:  # never kill the thread over one bad frame
                self._log(f"dashboard: snapshot failed: {exc!r}")
                payload = json.dumps({"error": f"snapshot_failed:{exc!r}"})
            with self._clients_lock:
                clients = list(self._clients)
            for client in clients:
                client.push(payload)
            # Sleep the REMAINDER of the period, so serialization time does
            # not slow the stream below the requested rate.
            elapsed = time.monotonic() - started
            self._stop.wait(max(0.0, self._stream_period - elapsed))

    def _add_client(self) -> _Client:
        client = _Client()
        with self._clients_lock:
            self._clients.append(client)
        return client

    def _remove_client(self, client: _Client) -> None:
        client.close()
        with self._clients_lock:
            if client in self._clients:
                self._clients.remove(client)

    # -- commands ----------------------------------------------------------

    def dispatch(self, command: str, payload: Dict[str, Any]) -> Dict[str, Any]:
        try:
            return self._dispatch_fn(command, payload)
        except Exception as exc:
            self._log(f"dashboard: command {command!r} failed: {exc!r}")
            return {"ok": False, "error": repr(exc)}


def _make_handler(server: DashboardServer):
    """Build a request-handler class bound to one DashboardServer."""

    class Handler(BaseHTTPRequestHandler):
        # Quiet: BaseHTTPRequestHandler logs every request to stderr, which at
        # one SSE connection per tab plus periodic polls would bury the ROS
        # log this dashboard exists to complement.
        def log_message(self, fmt, *args):  # noqa: A003 - stdlib signature
            return

        protocol_version = "HTTP/1.1"

        # -- helpers -------------------------------------------------------

        def _send_json(self, obj, status=200):
            body = json.dumps(obj).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            # No caching anywhere: every response here is live state, and a
            # cached dashboard showing a stopped robot as running is the exact
            # failure this whole thing is meant to prevent.
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_bytes(self, body: bytes, content_type: str, status=200):
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        # -- GET -----------------------------------------------------------

        def do_GET(self):  # noqa: N802 - stdlib signature
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                return self._serve_index()
            if path == "/api/state":
                return self._send_json(server._snapshot_fn())
            if path == "/api/map":
                return self._serve_map()
            if path == "/events":
                return self._serve_events()
            return self._send_json({"error": "not found"}, status=404)

        def _serve_index(self):
            index = _STATIC_DIR / "index.html"
            try:
                body = index.read_bytes()
            except Exception as exc:
                return self._send_json(
                    {"error": f"cannot read {index}: {exc!r}"}, status=500
                )
            return self._send_bytes(body, "text/html; charset=utf-8")

        def _serve_map(self):
            """The occupancy grid, or an explicit statement that there is none.

            A 200 carrying `available: false` rather than a 404: the browser
            needs to distinguish "SLAM is not running" (a normal state worth
            saying out loud) from "this dashboard is too old to have a map
            endpoint", and a 404 conflates them.
            """
            if server._map_fn is None:
                return self._send_json(
                    {"available": False, "reason": "map endpoint not wired up"}
                )
            try:
                return self._send_json(server._map_fn())
            except Exception as exc:
                server._log(f"dashboard: map render failed: {exc!r}")
                return self._send_json(
                    {"available": False, "reason": f"map render failed: {exc!r}"}
                )

        def _serve_events(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            # Chunked/streaming: the length is unknown and unbounded.
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()

            client = server._add_client()
            try:
                while not server._stop.is_set():
                    client.event.wait(timeout=1.0)
                    with client.lock:
                        if client.closed:
                            break
                        batch = client.pending
                        client.pending = []
                        client.event.clear()
                    if not batch:
                        # Comment frame doubles as a keep-alive: it proves the
                        # socket is alive during a snapshot stall, so the page
                        # can tell "robot quiet" from "connection dead".
                        self.wfile.write(b": keepalive\n\n")
                        self.wfile.flush()
                        continue
                    # Only the newest matters for a live view.
                    self.wfile.write(f"data: {batch[-1]}\n\n".encode())
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass  # tab closed; entirely normal
            except Exception as exc:
                server._log(f"dashboard: SSE client dropped: {exc!r}")
            finally:
                server._remove_client(client)

        # -- POST ----------------------------------------------------------

        def do_POST(self):  # noqa: N802 - stdlib signature
            path = self.path.split("?", 1)[0]
            if not path.startswith("/api/"):
                return self._send_json({"error": "not found"}, status=404)
            command = path[len("/api/"):]

            try:
                length = int(self.headers.get("Content-Length") or 0)
                raw = self.rfile.read(length) if length else b""
                payload = json.loads(raw) if raw else {}
                if not isinstance(payload, dict):
                    raise ValueError("payload must be a JSON object")
            except Exception as exc:
                return self._send_json(
                    {"ok": False, "error": f"bad request body: {exc!r}"}, status=400
                )

            result = server.dispatch(command, payload)
            status = 200 if result.get("ok", False) else 400
            return self._send_json(result, status=status)

    return Handler
