# Plan: the robot wall, a second computer that always shows both dashboards

> Status: APPROVED 2026-11-26 (Daniel: "I like the way of setting it up in such a way that it boots with the computer"; "I am fine for the robot to launch its own stack"); BUILT 2026-11-26 (352e15c, 890 workstation tests): wall page, launcher, installer, dashboard reload rule, CORS on GET state, boot unit and scripts, two how-tos. PENDING on the robots (both off the network tonight): deploy, `robot_boot/install.sh`, a watched power cycle; on the wall computer: clone, install, chrony, automatic login. The previous plans (robot profiles; zones as clearances) are built and deployed; git history keeps them. Written to `~/lumen/optomotive/CODING_PROMPT.md` (2026-10-22).

## Context

Daniel wants both dashboards on a second Pop!_OS computer, open from the moment it boots, surviving robot reboots without anyone touching it. The dashboard page half-does this today: an open tab reconnects on its own when a stack comes back (the event stream retries), but it never reloads, so after a stack restart it shows stale state (the map with no robot, 2026-11-18), and a tab opened while the robot is off shows the browser's error page forever. A rebooted robot is off the network for about two minutes.

## Existing code reused

`rm_dashboard/web_server.py` serves `/api/state` (JSON, read-only) and the page; `index.html` `connect()` is the reconnect; `docs/workstation_setup.md` has the wired-link recipe; `scripts/setup_workstation.sh` is the pattern for an installer that explains itself.

## Design

**The wall page**, `scripts/robot_wall/wall.html`, a local static page: one column per robot, each an iframe on that robot's dashboard under a status strip (name, domain, battery, safety state). Every 3 s it probes each robot with `GET /api/state`; while a robot is down its strip says so and the frame is blank; when the probe succeeds after failures the frame is re-pointed, so a rebooted robot comes back on its own. The page never POSTs: it is a viewer, like the dashboard it frames. Robot list from a generated `robots.js` beside it (name + URL), because a `file://` page cannot read a sibling JSON.

**The dashboard page** gains one rule: if the snapshot's `uptime_s` goes backwards, the stack restarted, reload. **The server** adds `Access-Control-Allow-Origin: *` on `GET /api/state` only (read-only JSON), so the wall can read the strip's numbers; commands keep no CORS.

**The launcher**, `scripts/robot_wall/robot_wall.sh`: serves the wall directory on `127.0.0.1:8099` with `python3 -m http.server` (a localhost page may fetch and frame http dashboards in every browser; a `file://` page is subject to each browser's own rules, so the file path is only the fallback without python3), picks the browser (chromium, google-chrome, then firefox, kiosk flag each), opens the page, logs to `~/.local/share/robot-wall/launch.log`, exits non-zero naming the browsers it looked for if none exists.

**The installer**, `scripts/robot_wall/install.sh [URL ...]` (default: both known robots): copies the page, writes `robots.js`, writes the XDG autostart entry `~/.config/autostart/robot-wall.desktop` (GNOME and COSMIC both run it at login). `--systemd` installs a user service on `graphical-session.target` instead, for a session that ignores XDG autostart. Booting straight into the wall also needs automatic login, a Settings toggle the how-to names.

## Failure modes

| Failure | Surfaces as |
|---|---|
| A robot off at boot, or rebooted | its strip reads "down, waiting"; the frame returns when the probe succeeds |
| A stack restart with the robot up | the framed dashboard reloads itself (uptime went backwards) |
| No supported browser | the launcher exits non-zero, names what it looked for, in the log |
| The old server build without the CORS header | the probe still works (opaque response = up); the strip shows name and URL only |
| The second computer not on the robot network | both strips stay "down"; the how-to's first section is the network |

## Changes

New: `scripts/robot_wall/{wall.html,robot_wall.sh,install.sh}`, `docs/robot_wall.md`, `tests/unit/test_robot_wall.py`. Changed: `rm_dashboard/static/index.html` (reload on restart), `web_server.py` (CORS on GET `/api/state`), `tests/unit/test_dashboard_web_server.py` (header present on GET state, absent on POST), `tests/unit/test_dashboard_sources.py` (reload pin), `docs/README.md`, `README.md` layout, `CLAUDE.md`, `src/rm_dashboard/README.md` (one line).

## Tests

The wall page: one frame per robot from `robots.js`, the probe hits `/api/state`, re-points a frame only on a down-to-up edge, contains no POST and no command path. The installer: writes the autostart entry with the launcher's path and `X-GNOME-Autostart-enabled=true`; `--systemd` writes a unit on `graphical-session.target`. The launcher: kiosk flag per browser, non-zero exit with no browser. The dashboard: `uptime_s` decreasing calls `location.reload()`. The server: CORS header only on GET `/api/state`. `test_docs` indexes the how-to under the cap.

## Verification

Workstation: suite green; `bash scripts/robot_wall/install.sh` here, log out and in, the wall appears in Firefox kiosk with both robots; stop bernard-1's stack and restart it, its column reloads on its own; pull its power, the column says down, and after the robot is back and launched it returns. Second computer: clone the branch, run the installer, enable automatic login, reboot.

## Phase 2: the robots launch their own stacks at boot

**Why.** With the wall alone, a power-cycled robot sits at "down, waiting" until someone runs the prep and the launch over SSH. With this, the sequence after a power cycle is: wait about three minutes, the column comes back by itself, the robot KILLED on teleop and ready to arm. An unattended launch moves nothing: rm_safety starts KILLED and `command_source` is `teleop` with nothing publishing it, the same two gates as a hand launch.

**The clock comes from the wall computer** (Daniel: "can we just set its clock to the computer one?"). Nothing in the stack needs the calendar date (every node reads the robot's one clock), but recordings and logs do, and the robots have no NTP and lose time on every power cut. So: the wall computer runs `chrony` as a time server for the robot network (`allow 192.168.4.0/24`, `local stratum 10` so it serves its own clock even without internet), each robot's `systemd-timesyncd` is pointed at the wall computer's fixed address by `install.sh --time-server <ip>`, and `boot_stack.sh` waits up to 30 s for `timedatectl` to report a sync BEFORE launching, so the stack never starts and then jumps. Wall computer off: the robot launches after the 30 s with the time it has. A sync landing later under a running stack is the known one-tick `pose_stale` blip (2026-11-18), fail-closed and harmless; it happens only when the wall computer comes up after the robot.

**Design.** `scripts/robot_boot/ncp-stack.service`, a SYSTEM unit (the Yahboom image auto-logs the desktop in, but a system unit needs no session): `After=network-online.target`, `User=jetson`, `Restart=no` (ncp.launch.py already refuses to stack a second copy on a running one, and a crash loop on a robot is worse than a stopped stack), `ExecStart=.../scripts/robot_boot/boot_stack.sh`, `ExecStop=.../scripts/stack_down.sh`. `boot_stack.sh` sources the four ROS setup files, waits up to 60 s for both serial by-id paths (the lidar's CP2102, the base board's CH340), runs `prep_robot.sh`, then `run_stack.sh stop_zone_enabled:=true slow_zone_enabled:=true with_waypoint:=true` (today's hand launch), logging to `~/stack_boot.log`. `scripts/robot_boot/install.sh` (run on the robot, once, `sudo -n`): installs and enables the unit, and DISABLES the factory autostart (`rosmaster_main.py`), the only thing that still races for the base port, by whatever mechanism starts it (to be read off the robot before writing this script: desktop autostart entry, rc.local or a unit); the disabled file is kept beside the original with `.disabled`, never deleted. `systemctl stop ncp-stack` is a clean teardown; `systemctl start` a clean bring-up; a hand `run_stack.sh` while the service runs is refused by the launch as today.

**Failure modes.** No serial device after 60 s: the script refuses, names the missing path, the service is `failed`, the log says why (this is the two-cable lidar on bernard-1). The launch refuses (profile, domain, a running stack): same. The service is stopped by hand and the robot rebooted: it starts again (enabled); use `systemctl disable` to keep a robot down. The factory app is wanted again: re-enable its entry.

**Tests.** The unit's pins (system unit, `Restart=no`, ExecStop is stack_down, after network-online, user jetson); the boot script waits for both by-id paths, runs prep then run_stack with the zones on, logs to a file; the installer keeps the disabled autostart beside the original. Robot verification: install on bernard-1, `systemctl start`, start lines as from a hand launch; reboot it from the wall's point of view: column down, then back within three minutes, KILLED on teleop, one /cmd_vel publisher.

## Not in this plan

Remote power control. Internet for the robots.
