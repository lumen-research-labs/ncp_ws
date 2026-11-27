"""The robot wall (scripts/robot_wall): a viewer that frames each robot's own
dashboard and re-points the frame when that robot comes back. It never sends
a command. The installer writes the robot list and an autostart entry; the
launcher opens a kiosk browser or fails loudly. Workstation tier.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys

import pytest

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_WALL = os.path.join(_REPO_ROOT, "scripts", "robot_wall")
_INDEX = os.path.join(_REPO_ROOT, "src", "rm_dashboard", "rm_dashboard", "static", "index.html")


def _read(name: str) -> str:
    with open(os.path.join(_WALL, name), encoding="utf-8") as fh:
        return fh.read()


def test_the_page_frames_each_robot_and_probes_its_state() -> None:
    html = _read("wall.html")
    assert '<script src="robots.js"></script>' in html
    assert "window.ROBOTS" in html
    assert 'fetch(url, { cache: "no-store" })' in html
    assert 'mode: "no-cors"' in html, "an old dashboard without the CORS header still counts as up"
    assert '"/api/state"' in html
    assert "setInterval(() => tick(r), PROBE_MS)" in html


def test_the_page_never_commands_anything() -> None:
    html = _read("wall.html")
    assert "POST" not in html and "method:" not in html
    for path in ("/api/kill", "/api/reset", "/api/source", "/api/goal", "/api/roam", "/api/jog", "/api/waypoint"):
        assert path not in html, path


def test_a_frame_is_repointed_only_on_a_down_to_up_edge_or_a_restart() -> None:
    html = _read("wall.html")
    tick = html[html.index("async function tick(r)"):html.index("build();")]
    assert "if(up && !r.up){ r.up = true; showFrame(r); }" in tick
    assert "else if(!up && r.up){ r.up = false; showDown(r); }" in tick
    assert "snap.uptime_s < r.uptime - 2) showFrame(r)" in tick, "a stack restart under a live frame reloads it"
    assert 'r.el.frame.src = r.url + "/?wall=" + Date.now()' in html, "fresh load, never a cached page"


def test_the_launcher_tries_each_browser_in_kiosk_mode_and_fails_loudly() -> None:
    sh = _read("robot_wall.sh")
    for b in ("chromium", "google-chrome", "firefox"):
        assert b in sh
    assert sh.count("--kiosk") >= 3
    assert 'log "robot_wall: no browser found' in sh and "exit 1" in sh
    assert "launch.log" in sh
    # served from localhost, not opened as a file: a localhost page may fetch
    # and frame the robots' http dashboards in every browser
    assert 'python3 -m http.server --bind 127.0.0.1' in sh and 'URL="http://127.0.0.1:$PORT/' in sh
    assert "falling back to $URL" in sh, "no python3: file:// still opens"
    assert "trap 'kill $SERVER 2>/dev/null' EXIT" in sh, "the server dies with the browser"
    # with no browser on PATH it exits 1 and says what it looked for: a PATH
    # holding only the coreutils the script needs, and no browser
    import shutil, tempfile
    with tempfile.TemporaryDirectory() as d:
        bindir = os.path.join(d, "bin"); os.mkdir(bindir)
        for tool in ("date", "tee", "mkdir", "dirname", "basename", "sleep"):
            os.symlink(shutil.which(tool), os.path.join(bindir, tool))
        env = dict(os.environ, PATH=bindir, HOME=d)
        r = subprocess.run(["/bin/bash", os.path.join(_WALL, "robot_wall.sh")], env=env, capture_output=True, text=True)
    assert r.returncode == 1 and "no browser found" in r.stderr + r.stdout


def test_the_installer_writes_the_robot_list_and_the_autostart_entry(tmp_path) -> None:
    env = dict(os.environ, HOME=str(tmp_path), XDG_DATA_HOME=str(tmp_path / "share"))
    r = subprocess.run(["bash", os.path.join(_WALL, "install.sh"), "b1=http://10.0.0.1:8080", "b3=http://10.0.0.3:8080"],
                       env=env, capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    dest = tmp_path / "share" / "robot-wall"
    assert (dest / "wall.html").is_file() and os.access(dest / "robot_wall.sh", os.X_OK)
    robots = (dest / "robots.js").read_text()
    assert 'name: "b1", url: "http://10.0.0.1:8080"' in robots and 'name: "b3", url: "http://10.0.0.3:8080"' in robots
    desktop = (tmp_path / ".config" / "autostart" / "robot-wall.desktop").read_text()
    assert f"Exec={dest}/robot_wall.sh" in desktop and "X-GNOME-Autostart-enabled=true" in desktop
    assert "Automatic Login" in r.stdout, "the one step the installer cannot do is named"


def test_the_installer_defaults_to_both_robots_and_refuses_a_bad_pair(tmp_path) -> None:
    env = dict(os.environ, HOME=str(tmp_path), XDG_DATA_HOME=str(tmp_path / "share"))
    r = subprocess.run(["bash", os.path.join(_WALL, "install.sh")], env=env, capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    robots = (tmp_path / "share" / "robot-wall" / "robots.js").read_text()
    assert "192.168.4.100:8080" in robots and "192.168.4.102:8080" in robots
    bad = subprocess.run(["bash", os.path.join(_WALL, "install.sh"), "bernard-9"], env=dict(env, HOME=str(tmp_path / "other")),
                         capture_output=True, text=True)
    assert bad.returncode == 1 and "expected name=http://host:port" in bad.stderr
    assert not (tmp_path / "other").exists(), "a refused install writes nothing"


def test_the_wall_computer_setup_works_from_a_clone_or_a_stick() -> None:
    sh = _read("setup_wall_computer.sh")
    assert 'if [ -d "$HERE/ncp_ws/.git" ]; then' in sh and 'elif [ -d "$HERE/../../.git" ]; then' in sh
    assert sh.index('say "2. chrony') < sh.index('say "3. the robot network'), "chrony needs the internet; joining RobotLab ends it"
    assert "sudo apt-get install -y chrony" in sh and 'ls "$HERE"/deb/chrony_*.deb' in sh
    assert "allow 192.168.4.0/24" in sh and "local stratum 10" in sh
    assert 'read -r -s -p "   RobotLab WiFi password: " PSK' in sh, "the key is asked, never committed"
    assert "ipv4.addresses \"$ADDRESS/24\"" in sh and 'ADDRESS="192.168.4.251"' in sh
    assert 'bash "$DEST/scripts/robot_wall/install.sh"' in sh
    for name in ("setup_wall_computer.sh", "make_bundle.sh"):
        r = subprocess.run(["bash", "-n", os.path.join(_WALL, name)], capture_output=True, text=True)
        assert r.returncode == 0, r.stderr
    bundle = _read("make_bundle.sh")
    assert "apt-get download chrony" in bundle and "robotlab.psk" in bundle and "umask 077" in bundle
    assert "git clone -q --branch" in bundle


def test_the_dashboard_reloads_itself_when_the_stack_restarts() -> None:
    html = open(_INDEX, encoding="utf-8").read()
    assert "latest.uptime_s < lastUptime - 2){ location.reload(); return; }" in html
    assert "lastUptime = latest.uptime_s;" in html
