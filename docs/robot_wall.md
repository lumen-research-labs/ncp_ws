# The robot wall

A second computer that shows both dashboards from the moment it boots and
keeps showing them through robot reboots and battery swaps. It is a viewer:
each column is that robot's own dashboard, framed, so every button in it
(ARM, KILL, the source buttons, ROAM, goals, jog) works exactly as in a tab.
What the wall adds is persistence: it probes each robot every three seconds,
shows "down, waiting" while a robot is off, and re-points the frame the moment
the robot's dashboard answers again. The dashboard itself now reloads when the
stack under it restarts, so a column never shows a previous run's map.

## 1. Put the computer on the robot network

The robots live on the RobotLab router (192.168.4.0/24, no internet), which
overlaps the lab WiFi's addresses. Pick one:

- **Dedicated wall:** join RobotLab directly (its WiFi, or a cable into the
  router). Both robots are reachable with no configuration; no internet.
- **Also needs the internet:** keep the lab WiFi and add a wired link with
  host routes to the two robots only, the recipe in
  [`workstation_setup.md`](workstation_setup.md).

Give the wall computer a fixed address on that network if the robots are to
take their clock from it (see [`robot_boot.md`](robot_boot.md)).

## 2. Install

Clone the repository on that computer (branch `feat/dynabarn-profile` until
merged), then:

```bash
bash scripts/robot_wall/install.sh          # bernard-1 and bernard-3 by default
bash scripts/robot_wall/install.sh bernard-1=http://192.168.4.100:8080   # or your own list
```

It copies the page and the launcher to `~/.local/share/robot-wall`, writes
the robot list to `robots.js` there, and registers an autostart entry in
`~/.config/autostart`, which GNOME and COSMIC run at login. If the session
ignores XDG autostart, `install.sh --systemd` installs a user service on
`graphical-session.target` instead. Try it at once with
`~/.local/share/robot-wall/robot_wall.sh`; it opens the first browser it
finds (Chromium, Chrome, Firefox) in kiosk mode and logs to
`~/.local/share/robot-wall/launch.log`.

## 3. Boot into it

Settings, Users, **Automatic Login** on, and the display's blank-screen
timer off (Settings, Power). Reboot: the wall appears without a keyboard.

## What you see after a robot power cycle

| moment | the robot's column |
|---|---|
| power off | "down, waiting"; the other column is unaffected |
| power on, about a minute | still waiting: nothing answers until the stack is up |
| the stack is up ([`robot_boot.md`](robot_boot.md), or a hand launch) | the dashboard loads fresh, KILLED on teleop, ready to arm |
| the stack restarts with the robot up | the column reloads itself |

## Limits

The wall cannot power a robot or start a stack: a robot that is off is off,
and the dashboard is served by the stack. With the boot service installed on
the robots, a power cycle needs no one. Both robots in one browser means both
KILL buttons are on one screen; each stops only its own robot (they are on
different ROS domains), so read the column header before pressing.
