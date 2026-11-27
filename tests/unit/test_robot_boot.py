"""scripts/robot_boot: the robot launches its own stack at boot. The unit is
a system unit without a restart loop; the boot script waits for the serial
devices and the clock, preps, then launches today's hand launch; the
installer disables the factory autostart by renaming, never deleting.
Workstation tier (text pins and bash -n).
"""

from __future__ import annotations

import os
import subprocess

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_BOOT = os.path.join(_REPO_ROOT, "scripts", "robot_boot")


def _read(name: str) -> str:
    with open(os.path.join(_BOOT, name), encoding="utf-8") as fh:
        return fh.read()


def test_the_unit_is_a_supervised_system_service_without_a_crash_loop() -> None:
    unit = _read("ncp-stack.service")
    assert "User=jetson" in unit and "WantedBy=multi-user.target" in unit
    assert "After=network-online.target time-sync.target" in unit
    assert "Restart=no" in unit, "ncp.launch.py refuses a second copy; a loop on a robot is worse than a stopped stack"
    assert "ExecStart=/bin/bash /home/jetson/ncp_ws/scripts/robot_boot/boot_stack.sh" in unit
    assert "ExecStop=/bin/bash /home/jetson/ncp_ws/scripts/stack_down.sh" in unit, "stop is the clean teardown: sources first, safety last"


def test_the_boot_script_waits_for_the_devices_and_the_clock_then_preps_and_launches() -> None:
    sh = _read("boot_stack.sh")
    order = [sh.index(s) for s in (
        "source /home/jetson/ncp_ws/install/setup.bash",
        'for dev in "$LIDAR" "$BASE"',
        "boot_stack: REFUSING to launch",
        "NTPSynchronized",
        "bash scripts/prep_robot.sh",
        "exec bash scripts/run_stack.sh stop_zone_enabled:=true slow_zone_enabled:=true with_waypoint:=true",
    )]
    assert order == sorted(order), "devices, clock, prep, launch, in that order"
    assert "usb-Silicon_Labs_CP2102" in sh and "usb-1a86_USB_Serial" in sh
    assert 'exec >>"$LOG" 2>&1' in sh and "stack_boot.log" in sh
    assert "date -s" not in sh and "ntpdate" not in sh, "the clock is timesyncd's job, never a jump from this script"


def test_the_installer_renames_the_autostart_and_never_deletes() -> None:
    sh = _read("install.sh")
    assert 'systemctl enable ncp-stack.service' in sh
    assert 'mv "$f" "$f.disabled"' in sh and "rm " not in sh
    assert "rosmaster_main" in sh
    assert 'grep -lZE "rosmaster_main|Rosmaster/rosmaster|start_app\\.sh"' in sh, \
        "the shipped entry's Exec is start_app.sh; rosmaster_main is one level down"
    assert "timesyncd.conf.d/robot-wall.conf" in sh and "NTP=%s" in sh
    assert "run as jetson on the robot" in sh and "needs passwordless sudo" in sh


def test_the_scripts_parse() -> None:
    for name in ("boot_stack.sh", "install.sh"):
        r = subprocess.run(["bash", "-n", os.path.join(_BOOT, name)], capture_output=True, text=True)
        assert r.returncode == 0, r.stderr
