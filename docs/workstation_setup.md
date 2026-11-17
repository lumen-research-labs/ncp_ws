# Set up a workstation

The workstation is where you edit code, run the unit and replay tests, build `rm_interfaces`, deploy to the robot over SSH and open the dashboard in a browser. The robot stack itself runs on the Jetson (ROS 2 Humble, native). Nothing here talks DDS to the robot: the dashboard is plain HTTP and deploys go over SSH.

Verified 2026-10-23 on Pop!_OS 24.04 (an Ubuntu 24.04 base) with ROS 2 Jazzy, Python 3.12.3, numpy 1.26.4, PyYAML 6.0.1, pytest 7.4.4 and colcon 0.3.0, all from apt. Jazzy on the workstation and Humble on the robot have not been tested together over DDS, and nothing in this workflow needs them to be.

## The short way

With ROS 2 installed and this repo cloned (step 2), one script does steps 1, 2, 3 and 4 and checks step 5:

```bash
bash scripts/setup_workstation.sh          # install, clone the upstream beside, test, build
bash scripts/setup_workstation.sh --check  # report only
```

It stops at the first thing it cannot do and names it. The steps below are what it does, for doing them by hand.

## 1. Install the packages

```bash
sudo apt install ros-jazzy-ros-base python3-colcon-common-extensions \
  python3-pytest python3-numpy python3-yaml
```

`torch` and `ncps` are optional. Without them the checkpoint tests skip (6 of 617).

## 2. Clone the two repositories side by side

```bash
mkdir -p ~/lumen && cd ~/lumen
git clone git@github.com:lumen-research-labs/ncp_ws.git
git clone git@github.com:lumen-research-labs/NCP-on-Rosmaster.git
```

The launch, `scripts/deploy_upstream.sh` and `tests/unit/test_vendor_is_pristine.py` expect the upstream checkout at `../NCP-on-Rosmaster`, next to `ncp_ws`. Check out the branch the robot runs, named in `CLAUDE.md` section 4 (today `feat/roam-mode` in `ncp_ws` and `feat/720-beam-270fov` in `NCP-on-Rosmaster`, which carries the trained weights).

## 3. Run the tests

```bash
cd ~/lumen/ncp_ws
python3 -m pytest tests/
```

Expected: `611 passed, 6 skipped in 15.60s`. No ROS runtime is needed. The robot tier runs more, because the ROS node modules cannot be imported here.

## 4. Build the workstation target

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select rm_interfaces
```

Expected: `Summary: 1 package finished [1min 40s]`. Robot-side packages are built on the robot. Do not co-install them here.

After a ROS upgrade a build directory from the old install fails with `No rule to make target '/opt/ros/jazzy/lib/libfastcdr.so.2.2.7'` or similar. Delete `build/` and `install/` and build again.

## 5. Reach the robots

Put this in `~/.ssh/config`. The key `~/.ssh/lumen` comes from Daniel.

```
Host bernard-1
  HostName 192.168.4.100
  User jetson
  IdentityFile ~/.ssh/lumen
  IdentitiesOnly yes
  ServerAliveInterval 30
  ConnectTimeout 8
```

bernard-1 sits on the RobotLab router (192.168.4.0/24, no internet), which overlaps the lab WiFi. Plug an Ethernet cable from the workstation into that router and give the port a host route to the robot only, so WiFi keeps the internet. Replace `enp1s0` with your interface name.

```bash
nmcli con add type ethernet ifname enp1s0 con-name robotlab-wired \
  ipv4.method manual ipv4.addresses 192.168.4.250/32 ipv4.routes "192.168.4.100/32, 192.168.4.102/32" \
  ipv4.never-default yes connection.autoconnect-priority 10
```

Test with `ssh bernard-1 echo up`. A silent robot is powered off, not a routing problem. The traps that cost sessions (the clock, the base-board serial port, the lidar symlink race, the `pkill -f` self-match) are in `.claude/skills/robot-ssh/SKILL.md`. Read it before the first remote command.

## 6. Deploy

```bash
bash scripts/deploy_to_robot.sh     # this repo -> ~/ncp_ws on the robot, git push over SSH
bash scripts/deploy_upstream.sh     # NCP-on-Rosmaster with the weights, tar over SSH
```

Then build and start the stack on the robot as [`operating_guide.md`](operating_guide.md) describes.

## 7. Open the dashboard

With the stack up, open `http://192.168.4.100:8080/` in a browser. Nothing is installed on the workstation for it.
