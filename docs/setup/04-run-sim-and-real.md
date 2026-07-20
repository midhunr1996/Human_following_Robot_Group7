# 04 — Run the follower in Simulation and on the Real Robot

Command reference for launching the Gazebo sim demo and the real TurtleBot3
Burger from the Ubuntu VM.

Key facts for this setup:
- VM: ROS 2 Humble, workspace at `~/ros2_ws` (built with `colcon --symlink-install`).
- Real robot: Raspberry Pi at **192.168.137.202**, SSH `ubuntu` / `turtlebot`,
  model **burger**, LiDAR **LDS-02**, **ROS_DOMAIN_ID=7**.
- The VM's `~/.bashrc` already exports `ROS_DOMAIN_ID=7` and `ROS_LOCALHOST_ONLY=0`
  (so, by default, a fresh VM terminal talks to the **real robot**).
- VM NIC is bridged to the Windows hotspot ("Microsoft Wi-Fi Direct Virtual Adapter #2").

---

## 0. See the VM desktop (to watch Gazebo)

Gazebo needs a visible desktop. From the **Windows host** (PowerShell):

```powershell
& "C:\Program Files\Oracle\VirtualBox\VBoxManage.exe" startvm "ros2-tb3-follower" --type gui
```

RAM note: run the VM at **6 GB** for the GUI (8 GB crashes on this 16 GB host; see
`01-create-vm.md`). If the host is low on free RAM, close apps first, or run headless
(`--type headless`) and drive it over SSH/scripts.

Every VM terminal below assumes you first source ROS + the workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

## 1. Simulation only (Gazebo follow-nearest demo)

The sim uses a **waffle_pi** model (it has a camera) in the 20×20 m world with 3 walking
people; the robot follows the nearest one.

**One-liner (3-pane tmux):**

```bash
bash /mnt/host_project/scripts/05-run-demo.sh
```

**Or manually — three terminals**, each first running the two `source` lines above plus:

```bash
export TURTLEBOT3_MODEL=waffle_pi
export DISPLAY=:0
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
```

```bash
# Terminal 1 — Gazebo + world + spawn robot
ros2 launch tb3_follower_bringup sim.launch.py
# Terminal 2 — YOLO person detector (nearest-person)
ros2 launch tb3_follower_bringup perception.launch.py
# Terminal 3 — behavior-tree follower
ros2 launch tb3_follower_bringup behavior.launch.py
```

Watch it work: `ros2 topic echo /person/detection` (distance to nearest) and
`ros2 topic echo /cmd_vel` (drive commands).

Stop the sim: `tmux kill-session -t tb3-follower` (or Ctrl+C in each terminal).

---

## 2. Real Burger robot

### 2a. On the Pi — start the robot bringup

```bash
ssh ubuntu@192.168.137.202          # password: turtlebot
# (on the Pi)
export ROS_DOMAIN_ID=7 TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02
ros2 launch turtlebot3_bringup robot.launch.py
```

Leave it running. (To start it detached so it survives logout:
`setsid nohup ros2 launch turtlebot3_bringup robot.launch.py </dev/null >/tmp/bringup.log 2>&1 &`)

### 2b. On the VM — see and drive the robot

```bash
export ROS_DOMAIN_ID=7 ROS_LOCALHOST_ONLY=0 TURTLEBOT3_MODEL=burger
ros2 topic list                       # /scan /odom /cmd_vel /imu ...
ros2 topic echo /scan --once          # confirm LiDAR data
ros2 run turtlebot3_teleop teleop_keyboard   # drive it (give it floor clearance!)
```

If the VM can't see the robot: confirm the NIC is bridged (host PowerShell):
```powershell
& "C:\Program Files\Oracle\VirtualBox\VBoxManage.exe" controlvm "ros2-tb3-follower" nic1 bridged "Microsoft Wi-Fi Direct Virtual Adapter #2"
```
and that `ping 192.168.137.202` works from the VM.

### 2c. Real vision-follow (needs the RealSense camera up)

The RealSense D435i currently browns out the Pi (under-voltage) — put it on a **powered
USB-3 hub** first. Once `/camera/camera/color/image_raw` is publishing:

```bash
# On the Pi: start the color stream
export ROS_DOMAIN_ID=7
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=false pointcloud.enable:=false \
  rgb_camera.color_profile:=640x480x15

# On the VM: run the follower against the real camera (remap to the RealSense topics)
export ROS_DOMAIN_ID=7 ROS_LOCALHOST_ONLY=0
ros2 run tb3_follower_perception person_detector_node --ros-args \
  --params-file ~/ros2_ws/src/tb3_follower_perception/config/detector_params.yaml \
  -p image_topic:=/camera/camera/color/image_raw \
  -p camera_info_topic:=/camera/camera/color/camera_info
# (separate terminal)
ros2 launch tb3_follower_bringup behavior.launch.py
```

The behavior node publishes `/cmd_vel`, which the real Burger subscribes to — so the
robot will physically follow the nearest person. Keep an e-stop / clear space ready.

---

## 3. Running sim AND real at the same time

They share topic names (`/cmd_vel`, `/scan`), so keep them on separate DDS scopes or
the sim will fight the real robot. Simplest: **isolate the sim to the VM** with
`ROS_LOCALHOST_ONLY=1` (loopback only), and leave the real robot on the network.

```bash
# Real robot: as in section 2 (ROS_LOCALHOST_ONLY=0, domain 7).

# Sim, isolated inside the VM — prefix each sim launch:
ROS_LOCALHOST_ONLY=1 ros2 launch tb3_follower_bringup sim.launch.py
ROS_LOCALHOST_ONLY=1 ros2 launch tb3_follower_bringup perception.launch.py
ROS_LOCALHOST_ONLY=1 ros2 launch tb3_follower_bringup behavior.launch.py
```

With `ROS_LOCALHOST_ONLY=1`, the sim's traffic never leaves the VM, so it can't reach
or disturb the real robot even though both use domain 7.

---

## 4. Stop everything

```bash
# Sim (VM):
tmux kill-session -t tb3-follower

# Real robot (from the VM, over SSH):
sshpass -p turtlebot ssh ubuntu@192.168.137.202 "pkill -f robot.launch; pkill -f realsense2_camera_node"
```
