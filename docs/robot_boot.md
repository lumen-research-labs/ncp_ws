# The robot launches its own stack at boot

With `ncp-stack.service` installed, a robot that is switched on comes up on
its own: devices, clock, cleanup, launch, about three minutes from power to
a dashboard that says KILLED on teleop. Nothing about the safety model
changes. An unattended launch passes the same two gates as a hand launch:
`rm_safety` starts KILLED, and `command_source` is `teleop` with nothing
publishing it. Arming is still a person's act.

## Install, once per robot

On the robot, after `deploy_to_robot.sh` has put the branch there:

```bash
bash scripts/robot_boot/install.sh --time-server 192.168.4.251
sudo systemctl start ncp-stack      # now, or just reboot
```

The installer enables the system unit, disables Yahboom's factory autostart
(`rosmaster_main.py`, the one thing that still raced our stack for the
base-board port; its entry is renamed `.disabled` beside the original, never
deleted) and, with `--time-server`, points the robot's `systemd-timesyncd` at
that address. Needs passwordless sudo for `jetson`, which both units have.

## What the boot script does, in order

1. Waits up to 60 s for both serial devices (the lidar's CP2102, the base
   board's CH340). A missing one is a refusal naming the path: on bernard-1
   that is the lidar's second cable.
2. Waits up to 30 s for the clock to sync. The stack does not need the date,
   but recordings and logs do, and a clock set later under a running stack
   makes every policy node stop for a tick. If the time server is off, it
   launches with the time it has.
3. Runs `prep_robot.sh`, then `run_stack.sh stop_zone_enabled:=true
   slow_zone_enabled:=true with_waypoint:=true`, today's hand launch. Output
   goes to `~/stack_boot.log`.

## Time from the wall computer

The robots have no NTP and lose the time on every power cut. The wall
computer serves its own clock to the robot network:

```bash
sudo apt install chrony                      # while it has internet
echo 'allow 192.168.4.0/24
local stratum 10' | sudo tee -a /etc/chrony/chrony.conf
sudo systemctl restart chrony
```

Give it a fixed address on that network (192.168.4.251 in the example) and
pass that to the robots' `install.sh --time-server`. `timedatectl` on a
robot shows `System clock synchronized: yes` once it has taken the time.

## Operating it

| want | do |
|---|---|
| stop the stack cleanly | `sudo systemctl stop ncp-stack` (runs `stack_down.sh`: sources first, safety last) |
| start it again | `sudo systemctl start ncp-stack` |
| keep a robot down across reboots | `sudo systemctl disable ncp-stack` |
| see why it did not come up | `~/stack_boot.log`, then `systemctl status ncp-stack` |
| launch by hand while it runs | refused by `ncp.launch.py` (a second stack), as always |

The unit has `Restart=no` on purpose: a crash loop on a robot is worse than a
stopped stack, and the log says what happened. The factory app can be had
back by renaming its `.disabled` entry.
