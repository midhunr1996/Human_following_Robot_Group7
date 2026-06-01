# TurtleBot3 YOLO + Behavior Tree Follower — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a ROS 2 Humble case study where a TurtleBot3 Waffle Pi in Gazebo follows a walking human, with perception via YOLOv8 and control via a py_trees behavior tree.

**Architecture:** Two-node split. `person_detector_node` runs YOLO on the camera feed and publishes `PersonDetection` messages. `follower_bt_node` runs a py_trees behavior tree that consumes detections and publishes `/cmd_vel`. Stack runs inside an Ubuntu 22.04 VirtualBox VM on the D: drive; sources live in the Windows host directory and are mounted in via shared folder + symlinked into `~/ros2_ws`.

**Tech Stack:** Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11, TurtleBot3 Waffle Pi, Python 3.10, Ultralytics YOLOv8n, py_trees + py_trees_ros2, cv_bridge, colcon (ament_python + ament_cmake).

**Two execution environments:**
- **Host (Windows):** scaffolding files, writing scripts, running pure-Python unit tests on functions that don't import ROS.
- **VM (Ubuntu):** install ROS, `colcon build`, run Gazebo, run the live demo, run integration tests.

Tasks 0–11 are 100% host-side (Windows). Tasks 12–17 are mostly host-side authoring but include VM execution checkpoints. Task 17 is the live demo done entirely in the VM.

---

## Code organization principle

To get a fast TDD loop on Windows without needing ROS installed:

- **Pure-math modules** (`geometry.py`, `control.py`) import only `numpy` and `dataclasses`. Tested on host with `pytest`.
- **ROS-dependent modules** (`person_detector_node.py`, `behaviours/*.py`, `follower_bt_node.py`) import `rclpy`/`py_trees`. Tested only inside the VM after `colcon build`.

Each task says explicitly whether it runs on host or VM.

---

## Task 0: Project scaffold + .gitignore

**Environment:** Host (Windows, PowerShell or bash)

**Files:**
- Create: `d:/ros case study/.gitignore`
- Create: `d:/ros case study/.gitattributes`
- Create: `d:/ros case study/README.md`

- [ ] **Step 0.1: Create `.gitignore`**

File: `d:/ros case study/.gitignore`

```
# Python
__pycache__/
*.py[cod]
*.egg-info/
.pytest_cache/
.venv/
venv/

# ROS 2 / colcon
build/
install/
log/

# Editor / OS
.vscode/
.idea/
*.swp
.DS_Store
Thumbs.db

# Models (downloaded at install time, not checked in)
*.pt
*.onnx
models/

# Logs
*.log
```

- [ ] **Step 0.2: Create `.gitattributes` to keep shell scripts LF**

File: `d:/ros case study/.gitattributes`

```
# Keep LF on shell scripts and world/launch files regardless of host
*.sh           text eol=lf
*.bash         text eol=lf
*.py           text eol=lf
*.yaml         text eol=lf
*.yml          text eol=lf
*.world        text eol=lf
*.launch.py    text eol=lf
*.xml          text eol=lf
*.msg          text eol=lf
*.srv          text eol=lf
```

- [ ] **Step 0.3: Create `README.md`**

File: `d:/ros case study/README.md`

````markdown
# TurtleBot3 Human Follower — ROS 2 + YOLO + Behavior Tree

A ROS 2 Humble case study: a simulated TurtleBot3 Waffle Pi follows a walking human in Gazebo, with perception by YOLOv8 and control by a py_trees behavior tree. Runs inside an Ubuntu 22.04 VirtualBox VM on Windows.

## Quick start

1. **Host (Windows):** Read [docs/setup/01-create-vm.md](docs/setup/01-create-vm.md) and create the Ubuntu 22.04 VM in VirtualBox.
2. **VM (Ubuntu):** Run the three install scripts in order:
   ```bash
   bash /mnt/host_project/scripts/02-install-ros.sh
   bash /mnt/host_project/scripts/03-install-yolo.sh
   bash /mnt/host_project/scripts/04-build-workspace.sh
   ```
3. **VM:** Launch the demo:
   ```bash
   bash /mnt/host_project/scripts/05-run-demo.sh
   ```

## Architecture

See [docs/architecture.md](docs/architecture.md) and [docs/superpowers/specs/2026-06-01-tb3-yolo-bt-follower-design.md](docs/superpowers/specs/2026-06-01-tb3-yolo-bt-follower-design.md).

## Layout

- `ros2_ws/src/tb3_follower_msgs/` — custom `PersonDetection.msg`
- `ros2_ws/src/tb3_follower_perception/` — YOLOv8 detector node
- `ros2_ws/src/tb3_follower_behavior/` — py_trees behavior tree node
- `ros2_ws/src/tb3_follower_bringup/` — launch files, Gazebo world, params, RViz config
- `scripts/` — install + demo runner scripts (executed inside the VM)
- `docs/setup/` — VM creation, install, and demo walkthrough docs
````

- [ ] **Step 0.4: Commit**

```bash
cd "d:/ros case study"
git add .gitignore .gitattributes README.md
git commit -m "chore: project scaffold (gitignore, gitattributes, README)"
```

---

## Task 1: VM creation doc

**Environment:** Host (Windows)

**Files:**
- Create: `d:/ros case study/docs/setup/01-create-vm.md`

- [ ] **Step 1.1: Write VM creation walkthrough**

File: `d:/ros case study/docs/setup/01-create-vm.md`

````markdown
# 01 — Create the Ubuntu 22.04 VM in VirtualBox

## What you need before starting

- VirtualBox 7.x already installed on Windows.
- At least 16 GB host RAM (VM uses 8 GB; OS uses the rest).
- ~50 GB free on the **D: drive**.
- Ubuntu 22.04.4 LTS Desktop ISO. Download from:
  https://releases.ubuntu.com/22.04/ubuntu-22.04.4-desktop-amd64.iso

Save the ISO anywhere — e.g. `D:\ISOs\ubuntu-22.04.4-desktop-amd64.iso`.

## Step 1 — Set VirtualBox default machine folder to D:

This makes new VMs land on D: by default.

1. Open VirtualBox.
2. **File → Preferences → General**.
3. Set **Default Machine Folder** to `D:\VirtualBox VMs`.
4. Click **OK**.

## Step 2 — Create the VM

1. **Machine → New**.
2. Settings:
   - **Name:** `ros2-tb3-follower`
   - **Folder:** `D:\VirtualBox VMs` (should already be the default after Step 1)
   - **ISO Image:** the Ubuntu 22.04.4 ISO you downloaded
   - **Type:** Linux
   - **Version:** Ubuntu (64-bit)
   - **Skip Unattended Installation:** **CHECK THIS BOX** (we want manual install for shared-folder setup)
3. **Hardware:**
   - **Base Memory:** 8192 MB (8 GB)
   - **Processors:** 4 CPUs
4. **Hard Disk:**
   - **Create a Virtual Hard Disk Now**
   - **Size:** 40 GB
   - **Format:** VDI, dynamically allocated
5. Click **Finish**.

## Step 3 — Tune VM settings before first boot

Select `ros2-tb3-follower` → **Settings**:

- **System → Processor:** confirm 4 CPUs, **enable** `Enable PAE/NX` and `Enable Nested VT-x/AMD-V` if greyed-on.
- **Display → Screen:**
  - **Video Memory:** 128 MB
  - **Graphics Controller:** VMSVGA
  - **Enable 3D Acceleration:** ON (best effort; Gazebo will fall back to software rendering if this is unreliable)
- **Network → Adapter 1:** NAT (default, no change needed).
- **Shared Folders → Add:**
  - **Folder Path:** `D:\ros case study`
  - **Folder Name:** `host_project`
  - **Auto-mount:** ON
  - **Make Permanent:** ON
  - **Read-only:** OFF
- **General → Advanced:** Shared Clipboard → Bidirectional, Drag'n'Drop → Bidirectional.

Click **OK**.

## Step 4 — Install Ubuntu

1. **Start** the VM.
2. Boot into Ubuntu installer → choose:
   - Language: English
   - Keyboard: your layout
   - **Normal installation** (NOT minimal)
   - **Download updates while installing:** ON
   - **Install third-party software:** ON
   - **Erase disk and install Ubuntu** (this only erases the VM's virtual disk, not your real one)
   - User: pick whatever username/password you want; remember them
   - **Log in automatically:** ON (convenience for a dev VM)
3. Wait for install (~10 min). Reboot when prompted. Press Enter to eject the ISO.

## Step 5 — Install VirtualBox Guest Additions

Inside the running Ubuntu VM:

1. Open a terminal (Ctrl+Alt+T).
2. Run:
   ```bash
   sudo apt update
   sudo apt install -y build-essential dkms linux-headers-$(uname -r)
   ```
3. From the VirtualBox menu bar (top of the VM window): **Devices → Insert Guest Additions CD image**.
4. When the file manager opens, right-click in the CD window → **Open in Terminal**, then:
   ```bash
   sudo ./VBoxLinuxAdditions.run
   ```
5. After it finishes, add your user to the `vboxsf` group so you can read the shared folder:
   ```bash
   sudo usermod -aG vboxsf $USER
   ```
6. Reboot the VM:
   ```bash
   sudo reboot
   ```

## Step 6 — Confirm the shared folder is mounted

After reboot, open a terminal and run:

```bash
ls /media/sf_host_project
```

You should see `README.md`, `docs/`, `scripts/`, etc. — the contents of `D:\ros case study`.

The default mount path is `/media/sf_host_project`. We'll create a symlink to make it `/mnt/host_project` for shorter paths:

```bash
sudo ln -s /media/sf_host_project /mnt/host_project
ls /mnt/host_project
```

You should see the same contents. **You are done with VM setup.** Proceed to `02-install-ros.md`.

## Troubleshooting

- **Shared folder shows "Permission denied":** you forgot Step 5.5 (`usermod -aG vboxsf`). Re-run it and reboot.
- **VM boots to a black screen after install:** disable 3D acceleration in Settings → Display, then restart.
- **`ls /media/` is empty:** Guest Additions install failed. Re-do Step 5, watching for errors.
````

- [ ] **Step 1.2: Commit**

```bash
cd "d:/ros case study"
git add docs/setup/01-create-vm.md
git commit -m "docs: VM creation walkthrough"
```

---

## Task 2: ROS 2 install script

**Environment:** Host (writing), VM (will execute later)

**Files:**
- Create: `d:/ros case study/scripts/02-install-ros.sh`

- [ ] **Step 2.1: Write the install script**

File: `d:/ros case study/scripts/02-install-ros.sh`

```bash
#!/usr/bin/env bash
# 02-install-ros.sh — Install ROS 2 Humble + TurtleBot3 + Gazebo on Ubuntu 22.04.
# Idempotent: safe to re-run after partial failure.

set -euo pipefail

log()  { echo -e "\033[1;34m[install-ros]\033[0m $*"; }
warn() { echo -e "\033[1;33m[install-ros]\033[0m $*"; }
die()  { echo -e "\033[1;31m[install-ros]\033[0m $*"; exit 1; }

# ----- Sanity checks -----
[[ "$(lsb_release -cs)" == "jammy" ]] || die "This script requires Ubuntu 22.04 (jammy). Got: $(lsb_release -cs)"
[[ $EUID -ne 0 ]] || die "Do NOT run as root. Run as your normal user; sudo is invoked per-step."

log "Ubuntu 22.04 confirmed. Starting install."

# ----- Locale -----
log "Setting UTF-8 locale..."
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ----- ROS 2 apt repo -----
log "Adding ROS 2 apt repository..."
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository -y universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

ROS_REPO_LINE="deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main"
echo "$ROS_REPO_LINE" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

# ----- Install ROS 2 Humble desktop + dev tools -----
log "Installing ros-humble-desktop and dev tools (~5-10 min)..."
sudo apt install -y \
    ros-humble-desktop \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete

# ----- rosdep init -----
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    log "Initializing rosdep..."
    sudo rosdep init
fi
rosdep update

# ----- TurtleBot3 + Gazebo packages -----
log "Installing TurtleBot3 + Gazebo Classic..."
sudo apt install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3-gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-vision-msgs

# ----- tmux for the demo runner -----
sudo apt install -y tmux

# ----- Env vars in ~/.bashrc -----
log "Configuring ~/.bashrc..."
BASHRC="$HOME/.bashrc"
MARKER="# >>> tb3-follower env >>>"
if ! grep -q "$MARKER" "$BASHRC"; then
    cat >> "$BASHRC" <<EOF

$MARKER
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash
[[ -f \$HOME/ros2_ws/install/setup.bash ]] && source \$HOME/ros2_ws/install/setup.bash
# <<< tb3-follower env <<<
EOF
    log "Added env block to ~/.bashrc."
else
    log "~/.bashrc env block already present, skipping."
fi

log "DONE. Open a NEW terminal (or 'source ~/.bashrc') and run scripts/03-install-yolo.sh next."
```

- [ ] **Step 2.2: Mark executable + commit**

```bash
cd "d:/ros case study"
git add scripts/02-install-ros.sh
git update-index --chmod=+x scripts/02-install-ros.sh
git commit -m "feat: ROS 2 Humble + TurtleBot3 install script"
```

---

## Task 3: YOLO install script

**Environment:** Host (writing), VM (will execute later)

**Files:**
- Create: `d:/ros case study/scripts/03-install-yolo.sh`

- [ ] **Step 3.1: Write the YOLO install script**

File: `d:/ros case study/scripts/03-install-yolo.sh`

```bash
#!/usr/bin/env bash
# 03-install-yolo.sh — Install Ultralytics YOLOv8 + dependencies and pre-download yolov8n.pt.
# Idempotent.

set -euo pipefail

log()  { echo -e "\033[1;34m[install-yolo]\033[0m $*"; }
die()  { echo -e "\033[1;31m[install-yolo]\033[0m $*"; exit 1; }

[[ $EUID -ne 0 ]] || die "Do NOT run as root."

log "Installing Python dev tools..."
sudo apt update
sudo apt install -y python3-pip python3-venv libgl1 libglib2.0-0

log "Upgrading pip..."
python3 -m pip install --upgrade pip

log "Installing Ultralytics (CPU-only torch) and runtime deps..."
# CPU-only torch keeps the install small and avoids pulling CUDA toolchain
python3 -m pip install --user \
    --index-url https://download.pytorch.org/whl/cpu \
    torch==2.2.2 torchvision==0.17.2
python3 -m pip install --user \
    ultralytics==8.2.31 \
    opencv-python-headless==4.10.0.84 \
    numpy==1.26.4

log "Installing py_trees + py_trees_ros for ROS 2 Humble..."
sudo apt install -y \
    ros-humble-py-trees \
    ros-humble-py-trees-ros \
    ros-humble-py-trees-ros-interfaces

log "Pre-downloading yolov8n.pt to ~/models/..."
mkdir -p "$HOME/models"
if [[ ! -f "$HOME/models/yolov8n.pt" ]]; then
    python3 - <<'PY'
from ultralytics import YOLO
import shutil, os
m = YOLO("yolov8n.pt")  # downloads to current dir
shutil.move("yolov8n.pt", os.path.expanduser("~/models/yolov8n.pt"))
print("Downloaded yolov8n.pt ->", os.path.expanduser("~/models/yolov8n.pt"))
PY
else
    log "yolov8n.pt already present at ~/models/, skipping."
fi

log "Sanity check: torch + ultralytics import..."
python3 - <<'PY'
import torch, ultralytics, cv2, numpy
print("torch     :", torch.__version__)
print("ultralytics:", ultralytics.__version__)
print("cv2       :", cv2.__version__)
print("numpy     :", numpy.__version__)
PY

log "DONE. Next: scripts/04-build-workspace.sh"
```

- [ ] **Step 3.2: Commit**

```bash
cd "d:/ros case study"
git add scripts/03-install-yolo.sh
git update-index --chmod=+x scripts/03-install-yolo.sh
git commit -m "feat: YOLOv8 + py_trees install script"
```

---

## Task 4: Workspace build script

**Environment:** Host (writing), VM (will execute later)

**Files:**
- Create: `d:/ros case study/scripts/04-build-workspace.sh`

- [ ] **Step 4.1: Write the build script**

File: `d:/ros case study/scripts/04-build-workspace.sh`

```bash
#!/usr/bin/env bash
# 04-build-workspace.sh — Symlink the workspace from the shared folder into $HOME
# and run colcon build. Idempotent.

set -euo pipefail

log()  { echo -e "\033[1;34m[build-ws]\033[0m $*"; }
die()  { echo -e "\033[1;31m[build-ws]\033[0m $*"; exit 1; }

HOST_WS="/mnt/host_project/ros2_ws"
LOCAL_WS="$HOME/ros2_ws"

[[ -d "$HOST_WS/src" ]] || die "Shared folder workspace not found at $HOST_WS/src. Is the VirtualBox shared folder mounted?"

# ----- Symlink or refresh -----
if [[ -L "$LOCAL_WS" ]]; then
    log "Symlink $LOCAL_WS already exists, leaving in place."
elif [[ -e "$LOCAL_WS" ]]; then
    die "$LOCAL_WS exists but is not a symlink. Remove it manually and re-run."
else
    log "Creating symlink $LOCAL_WS -> $HOST_WS"
    ln -s "$HOST_WS" "$LOCAL_WS"
fi

# ----- Source ROS env -----
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash

cd "$LOCAL_WS"

# ----- rosdep deps -----
log "Resolving package dependencies with rosdep..."
rosdep install --from-paths src --ignore-src -r -y || true

# ----- colcon build -----
log "Running colcon build --symlink-install (this can take a few minutes)..."
colcon build --symlink-install

log "DONE. Open a NEW terminal and run: source ~/ros2_ws/install/setup.bash"
log "Then: bash /mnt/host_project/scripts/05-run-demo.sh"
```

- [ ] **Step 4.2: Commit**

```bash
cd "d:/ros case study"
git add scripts/04-build-workspace.sh
git update-index --chmod=+x scripts/04-build-workspace.sh
git commit -m "feat: workspace symlink + colcon build script"
```

---

## Task 5: `tb3_follower_msgs` package — custom `PersonDetection` msg

**Environment:** Host (writing), VM (will build later)

**Files:**
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_msgs/package.xml`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_msgs/CMakeLists.txt`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_msgs/msg/PersonDetection.msg`

- [ ] **Step 5.1: `package.xml`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_msgs/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>tb3_follower_msgs</name>
  <version>0.1.0</version>
  <description>Custom messages for the TurtleBot3 YOLO + BT follower case study.</description>
  <maintainer email="midhunr2015@gmail.com">Midhun</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>builtin_interfaces</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 5.2: `CMakeLists.txt`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_msgs/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(tb3_follower_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PersonDetection.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
```

- [ ] **Step 5.3: `PersonDetection.msg`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_msgs/msg/PersonDetection.msg`

```
# Single highest-confidence person detection (single-target follow).
# All bbox coords are normalized [0,1] relative to image dimensions.
bool detected
float32 bbox_cx
float32 bbox_cy
float32 bbox_w
float32 bbox_h
float32 distance      # meters, -1.0 if unknown
float32 confidence    # [0,1]
builtin_interfaces/Time stamp
```

- [ ] **Step 5.4: Commit**

```bash
cd "d:/ros case study"
git add ros2_ws/src/tb3_follower_msgs/
git commit -m "feat(msgs): PersonDetection custom msg package"
```

---

## Task 6: Perception — pure-math geometry module + tests (TDD on host)

**Environment:** Host (Windows, Python 3.10+, pip install numpy pytest)

**Files:**
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_perception/tb3_follower_perception/__init__.py`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_perception/tb3_follower_perception/geometry.py`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_perception/test/test_geometry.py`

This is the first TDD task. Goal: prove the distance math works before wiring it into a ROS node.

- [ ] **Step 6.1: Set up Python venv on host for fast TDD (one-time)**

In PowerShell:

```powershell
cd "d:/ros case study"
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install numpy pytest
```

(If `Activate.ps1` is blocked: `Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass`.)

- [ ] **Step 6.2: Write the failing test**

File: `d:/ros case study/ros2_ws/src/tb3_follower_perception/test/test_geometry.py`

```python
"""Pure-math tests for tb3_follower_perception.geometry — no ROS deps."""
import math
import pytest

from tb3_follower_perception.geometry import (
    visual_distance,
    bbox_cx_to_lidar_index,
    fuse_distance,
)


class TestVisualDistance:
    def test_far_person_small_bbox(self):
        # 1.7m person, bbox 60px tall in a 480-px image, focal 525px (Waffle Pi default-ish)
        d = visual_distance(person_height_m=1.7, bbox_h_px=60, focal_px=525)
        assert d == pytest.approx(14.875, rel=1e-3)

    def test_close_person_large_bbox(self):
        d = visual_distance(person_height_m=1.7, bbox_h_px=400, focal_px=525)
        assert d == pytest.approx(2.23125, rel=1e-3)

    def test_zero_bbox_raises(self):
        with pytest.raises(ValueError):
            visual_distance(person_height_m=1.7, bbox_h_px=0, focal_px=525)

    def test_negative_focal_raises(self):
        with pytest.raises(ValueError):
            visual_distance(person_height_m=1.7, bbox_h_px=100, focal_px=-1)


class TestBboxToLidarIndex:
    def test_center_maps_to_zero_yaw(self):
        # 360-beam scan, angle_min=-pi, angle_increment=2pi/360
        # bbox_cx=0.5 (image center) should map to forward-facing beam.
        # Camera HFOV assumed 1.085 rad (62.2 deg, Waffle Pi Raspberry Pi cam default).
        idx = bbox_cx_to_lidar_index(
            bbox_cx=0.5,
            camera_hfov_rad=1.085,
            scan_angle_min=-math.pi,
            scan_angle_increment=(2 * math.pi) / 360,
            num_beams=360,
        )
        # Forward = yaw 0 = scan index where angle_min + i*incr == 0 => i = pi / incr
        assert idx == 180

    def test_left_edge_negative_yaw(self):
        idx = bbox_cx_to_lidar_index(
            bbox_cx=0.0,
            camera_hfov_rad=1.085,
            scan_angle_min=-math.pi,
            scan_angle_increment=(2 * math.pi) / 360,
            num_beams=360,
        )
        # bbox_cx=0 means person at left edge of frame -> +HFOV/2 yaw (object to robot's left)
        # +0.5425 rad ≈ +31 deg ≈ index 180 + 31 = 211
        assert idx == pytest.approx(211, abs=1)

    def test_right_edge_positive_yaw(self):
        idx = bbox_cx_to_lidar_index(
            bbox_cx=1.0,
            camera_hfov_rad=1.085,
            scan_angle_min=-math.pi,
            scan_angle_increment=(2 * math.pi) / 360,
            num_beams=360,
        )
        # bbox_cx=1 -> person on robot's right -> -31 deg -> index 180 - 31 = 149
        assert idx == pytest.approx(149, abs=1)


class TestFuseDistance:
    def test_lidar_valid_overrides_visual(self):
        d = fuse_distance(visual_d=5.0, lidar_d=2.0, lidar_range_max=10.0)
        assert d == 2.0

    def test_lidar_above_max_falls_back_to_visual(self):
        d = fuse_distance(visual_d=5.0, lidar_d=10.5, lidar_range_max=10.0)
        assert d == 5.0

    def test_lidar_nan_falls_back_to_visual(self):
        d = fuse_distance(visual_d=5.0, lidar_d=float("nan"), lidar_range_max=10.0)
        assert d == 5.0

    def test_lidar_inf_falls_back_to_visual(self):
        d = fuse_distance(visual_d=5.0, lidar_d=float("inf"), lidar_range_max=10.0)
        assert d == 5.0
```

- [ ] **Step 6.3: Create empty `__init__.py` so tests can import the module**

File: `d:/ros case study/ros2_ws/src/tb3_follower_perception/tb3_follower_perception/__init__.py`

(empty file)

- [ ] **Step 6.4: Run test to verify it fails**

```powershell
cd "d:/ros case study/ros2_ws/src/tb3_follower_perception"
$env:PYTHONPATH = "."; pytest test/test_geometry.py -v
```

Expected: `ModuleNotFoundError: No module named 'tb3_follower_perception.geometry'` (or all tests collect as errors).

- [ ] **Step 6.5: Write minimal implementation**

File: `d:/ros case study/ros2_ws/src/tb3_follower_perception/tb3_follower_perception/geometry.py`

```python
"""Pure geometry helpers for the perception node. No ROS imports.

Tested by test_geometry.py — runnable on any machine with numpy + pytest.
"""
from __future__ import annotations

import math


def visual_distance(*, person_height_m: float, bbox_h_px: float, focal_px: float) -> float:
    """Estimate distance to a person using pinhole projection.

    d = (H * f) / h_px

    where H is the assumed person height in meters, f is camera focal length in
    pixels, and h_px is the bounding box height in pixels.
    """
    if bbox_h_px <= 0:
        raise ValueError("bbox_h_px must be > 0")
    if focal_px <= 0:
        raise ValueError("focal_px must be > 0")
    return (person_height_m * focal_px) / bbox_h_px


def bbox_cx_to_lidar_index(
    *,
    bbox_cx: float,
    camera_hfov_rad: float,
    scan_angle_min: float,
    scan_angle_increment: float,
    num_beams: int,
) -> int:
    """Map a normalized bbox center x ([0,1]) to a LaserScan beam index.

    Convention: bbox_cx=0.5 maps to yaw 0 (straight ahead). bbox_cx=0 (left edge
    of image) maps to +HFOV/2 yaw (object to robot's LEFT, positive yaw in ROS).
    bbox_cx=1 maps to -HFOV/2 (right side).
    """
    yaw = (0.5 - bbox_cx) * camera_hfov_rad
    raw_idx = (yaw - scan_angle_min) / scan_angle_increment
    idx = int(round(raw_idx))
    return idx % num_beams


def fuse_distance(*, visual_d: float, lidar_d: float, lidar_range_max: float) -> float:
    """Prefer LiDAR if it returned a valid in-range reading, else fall back to visual."""
    if math.isnan(lidar_d) or math.isinf(lidar_d):
        return visual_d
    if lidar_d <= 0 or lidar_d >= lidar_range_max:
        return visual_d
    return lidar_d
```

- [ ] **Step 6.6: Run test to verify it passes**

```powershell
cd "d:/ros case study/ros2_ws/src/tb3_follower_perception"
$env:PYTHONPATH = "."; pytest test/test_geometry.py -v
```

Expected: all 10 tests pass.

- [ ] **Step 6.7: Commit**

```bash
cd "d:/ros case study"
git add ros2_ws/src/tb3_follower_perception/tb3_follower_perception/__init__.py \
        ros2_ws/src/tb3_follower_perception/tb3_follower_perception/geometry.py \
        ros2_ws/src/tb3_follower_perception/test/test_geometry.py
git commit -m "feat(perception): pure-math geometry helpers + tests"
```

---

## Task 7: Perception node + package metadata

**Environment:** Host (writing), VM (will build + run later)

**Files:**
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_perception/package.xml`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_perception/setup.py`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_perception/setup.cfg`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_perception/resource/tb3_follower_perception`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_perception/tb3_follower_perception/person_detector_node.py`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_perception/config/detector_params.yaml`

- [ ] **Step 7.1: `package.xml`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_perception/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>tb3_follower_perception</name>
  <version>0.1.0</version>
  <description>YOLOv8-based person detection node for the TB3 follower case study.</description>
  <maintainer email="midhunr2015@gmail.com">Midhun</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>tb3_follower_msgs</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 7.2: `setup.py`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_perception/setup.py`

```python
from setuptools import setup
from glob import glob
import os

package_name = "tb3_follower_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Midhun",
    maintainer_email="midhunr2015@gmail.com",
    description="YOLOv8 person detector node",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "person_detector_node = tb3_follower_perception.person_detector_node:main",
        ],
    },
)
```

- [ ] **Step 7.3: `setup.cfg`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_perception/setup.cfg`

```ini
[develop]
script_dir=$base/lib/tb3_follower_perception
[install]
install_scripts=$base/lib/tb3_follower_perception
```

- [ ] **Step 7.4: `resource/tb3_follower_perception` (empty marker file)**

File: `d:/ros case study/ros2_ws/src/tb3_follower_perception/resource/tb3_follower_perception`

(empty file — its existence is what matters for ament index)

- [ ] **Step 7.5: Params YAML**

File: `d:/ros case study/ros2_ws/src/tb3_follower_perception/config/detector_params.yaml`

```yaml
person_detector_node:
  ros__parameters:
    model_path: "~/models/yolov8n.pt"
    conf_threshold: 0.5
    imgsz: 320
    max_rate_hz: 10.0
    person_height_m: 1.7
    camera_hfov_rad: 1.085   # Waffle Pi default Raspberry Pi cam
    image_topic: "/camera/image_raw"
    camera_info_topic: "/camera/camera_info"
    scan_topic: "/scan"
    detection_topic: "/person/detection"
```

- [ ] **Step 7.6: The detector node**

File: `d:/ros case study/ros2_ws/src/tb3_follower_perception/tb3_follower_perception/person_detector_node.py`

```python
"""YOLOv8 person detector ROS 2 node.

Subscribes to /camera/image_raw + /camera/camera_info + /scan.
Publishes /person/detection (tb3_follower_msgs/PersonDetection) at up to max_rate_hz.
"""
from __future__ import annotations

import os
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge

from tb3_follower_msgs.msg import PersonDetection

from tb3_follower_perception.geometry import (
    visual_distance,
    bbox_cx_to_lidar_index,
    fuse_distance,
)


class PersonDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("person_detector_node")

        # ----- Params -----
        self.declare_parameter("model_path", "~/models/yolov8n.pt")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("imgsz", 320)
        self.declare_parameter("max_rate_hz", 10.0)
        self.declare_parameter("person_height_m", 1.7)
        self.declare_parameter("camera_hfov_rad", 1.085)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("detection_topic", "/person/detection")

        gp = self.get_parameter
        self.conf_threshold = gp("conf_threshold").value
        self.imgsz = int(gp("imgsz").value)
        self.max_rate_hz = float(gp("max_rate_hz").value)
        self.person_height_m = float(gp("person_height_m").value)
        self.camera_hfov_rad = float(gp("camera_hfov_rad").value)
        self.min_period_s = 1.0 / max(self.max_rate_hz, 1e-6)

        # ----- YOLO model -----
        # Import here so unit tests of geometry.py don't need ultralytics.
        from ultralytics import YOLO  # noqa: WPS433

        model_path = os.path.expanduser(gp("model_path").value)
        if not Path(model_path).exists():
            self.get_logger().error(f"Model not found at {model_path}. Run scripts/03-install-yolo.sh.")
            raise FileNotFoundError(model_path)
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)

        # ----- Bridge + state -----
        self.bridge = CvBridge()
        self.focal_px: float | None = None
        self.last_scan: LaserScan | None = None
        self.last_pub_time: float = 0.0
        self.busy: bool = False  # frame-drop guard

        # ----- QoS -----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ----- Subs / pub -----
        self.create_subscription(
            CameraInfo, gp("camera_info_topic").value, self._on_camera_info, 1
        )
        self.create_subscription(
            LaserScan, gp("scan_topic").value, self._on_scan, sensor_qos
        )
        self.create_subscription(
            Image, gp("image_topic").value, self._on_image, sensor_qos
        )
        self.pub = self.create_publisher(
            PersonDetection, gp("detection_topic").value, 10
        )

        self.get_logger().info("person_detector_node ready.")

    # ---------- callbacks ----------

    def _on_camera_info(self, msg: CameraInfo) -> None:
        # Take focal length from K[0,0] (fx). Stop subscribing after first valid read.
        if self.focal_px is None and msg.k[0] > 0:
            self.focal_px = float(msg.k[0])
            self.get_logger().info(f"Focal length captured: fx={self.focal_px:.1f} px")

    def _on_scan(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self.last_pub_time < self.min_period_s:
            return  # rate limit
        if self.busy:
            return  # previous inference still running
        if self.focal_px is None:
            return  # wait for camera info

        self.busy = True
        try:
            self._process_frame(msg, now)
        finally:
            self.busy = False

    # ---------- core ----------

    def _process_frame(self, img_msg: Image, now: float) -> None:
        frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        h_px, w_px = frame.shape[:2]

        t0 = time.monotonic()
        results = self.model.predict(
            source=frame,
            imgsz=self.imgsz,
            conf=self.conf_threshold,
            classes=[0],          # COCO class 0 = person
            verbose=False,
        )
        infer_ms = (time.monotonic() - t0) * 1000.0

        out = PersonDetection()
        out.stamp = self.get_clock().now().to_msg()

        boxes = results[0].boxes if results else None
        if boxes is None or len(boxes) == 0:
            out.detected = False
            out.distance = -1.0
            self.pub.publish(out)
            self.last_pub_time = now
            return

        # Pick highest-confidence box
        confs = boxes.conf.cpu().numpy()
        best = int(np.argmax(confs))
        x1, y1, x2, y2 = boxes.xyxy[best].cpu().numpy().tolist()
        conf = float(confs[best])

        bbox_w_px = max(x2 - x1, 1.0)
        bbox_h_px = max(y2 - y1, 1.0)
        bbox_cx_px = (x1 + x2) / 2.0
        bbox_cy_px = (y1 + y2) / 2.0

        # Normalize
        out.detected = True
        out.bbox_cx = float(bbox_cx_px / w_px)
        out.bbox_cy = float(bbox_cy_px / h_px)
        out.bbox_w = float(bbox_w_px / w_px)
        out.bbox_h = float(bbox_h_px / h_px)
        out.confidence = conf

        # Distance fusion
        d_visual = visual_distance(
            person_height_m=self.person_height_m,
            bbox_h_px=bbox_h_px,
            focal_px=self.focal_px,
        )

        d_lidar = float("nan")
        if self.last_scan is not None:
            idx = bbox_cx_to_lidar_index(
                bbox_cx=out.bbox_cx,
                camera_hfov_rad=self.camera_hfov_rad,
                scan_angle_min=self.last_scan.angle_min,
                scan_angle_increment=self.last_scan.angle_increment,
                num_beams=len(self.last_scan.ranges),
            )
            if 0 <= idx < len(self.last_scan.ranges):
                d_lidar = float(self.last_scan.ranges[idx])

        out.distance = fuse_distance(
            visual_d=d_visual,
            lidar_d=d_lidar,
            lidar_range_max=(self.last_scan.range_max if self.last_scan else 10.0),
        )

        self.pub.publish(out)
        self.last_pub_time = now

        if infer_ms > 100.0:
            self.get_logger().warn(
                f"YOLO inference slow ({infer_ms:.0f} ms) — dropping next frame to keep realtime."
            )


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 7.7: Commit**

```bash
cd "d:/ros case study"
git add ros2_ws/src/tb3_follower_perception/
git commit -m "feat(perception): YOLOv8 person detector node + params + package metadata"
```

---

## Task 8: Behavior — pure controller module + tests (TDD on host)

**Environment:** Host (Windows, Python 3.10+, numpy + pytest in `.venv`)

**Files:**
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/tb3_follower_behavior/__init__.py`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/tb3_follower_behavior/control.py`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/test/test_control.py`

- [ ] **Step 8.1: Write failing tests**

File: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/test/test_control.py`

```python
"""Pure controller tests — no ROS deps."""
import pytest

from tb3_follower_behavior.control import (
    ControlParams,
    compute_approach_twist,
    clamp,
)


@pytest.fixture
def default_params() -> ControlParams:
    return ControlParams(
        target_distance=1.0,
        close_threshold=0.8,
        far_threshold=1.2,
        max_linear_speed=0.22,
        max_angular_speed=1.0,
        k_linear=0.4,
        k_angular=1.5,
    )


class TestClamp:
    def test_within_range(self):
        assert clamp(0.5, -1.0, 1.0) == 0.5

    def test_above(self):
        assert clamp(5.0, -1.0, 1.0) == 1.0

    def test_below(self):
        assert clamp(-5.0, -1.0, 1.0) == -1.0


class TestApproachTwist:
    def test_far_person_drives_forward(self, default_params):
        v, w = compute_approach_twist(distance=2.0, bbox_cx=0.5, params=default_params)
        # error = 2.0 - 1.0 = 1.0; k_lin*err = 0.4; clamped to 0.22 max
        assert v == pytest.approx(0.22, abs=1e-6)
        assert w == pytest.approx(0.0, abs=1e-6)

    def test_far_person_steers_toward_offset(self, default_params):
        # bbox_cx=0.3 -> person to LEFT of frame -> turn LEFT (positive angular.z in ROS)
        v, w = compute_approach_twist(distance=1.5, bbox_cx=0.3, params=default_params)
        assert v > 0.0
        # k_ang * -(0.3 - 0.5) = 1.5 * 0.2 = 0.3
        assert w == pytest.approx(0.3, abs=1e-6)

    def test_far_person_steers_right_for_right_offset(self, default_params):
        v, w = compute_approach_twist(distance=1.5, bbox_cx=0.7, params=default_params)
        assert w == pytest.approx(-0.3, abs=1e-6)

    def test_angular_clamped(self, default_params):
        # Extreme offset -> would saturate
        v, w = compute_approach_twist(distance=1.5, bbox_cx=0.0, params=default_params)
        # k_ang * 0.5 = 0.75, under cap 1.0
        assert w == pytest.approx(0.75, abs=1e-6)

    def test_at_target_distance_no_forward(self, default_params):
        # distance equals target -> linear error is 0
        v, w = compute_approach_twist(distance=1.0, bbox_cx=0.5, params=default_params)
        assert v == pytest.approx(0.0, abs=1e-6)
        assert w == pytest.approx(0.0, abs=1e-6)

    def test_below_target_no_backup(self, default_params):
        # If person is closer than target, we don't back up — STOP branch handles that.
        # compute_approach_twist must clamp linear to >= 0.
        v, _ = compute_approach_twist(distance=0.3, bbox_cx=0.5, params=default_params)
        assert v >= 0.0
```

- [ ] **Step 8.2: Create empty `__init__.py`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/tb3_follower_behavior/__init__.py`

(empty)

- [ ] **Step 8.3: Run test to verify it fails**

```powershell
cd "d:/ros case study/ros2_ws/src/tb3_follower_behavior"
$env:PYTHONPATH = "."; pytest test/test_control.py -v
```

Expected: `ModuleNotFoundError: No module named 'tb3_follower_behavior.control'`.

- [ ] **Step 8.4: Implement `control.py`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/tb3_follower_behavior/control.py`

```python
"""Pure controller logic for the follower behavior tree.

No ROS imports — tested directly with pytest.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ControlParams:
    target_distance: float
    close_threshold: float
    far_threshold: float
    max_linear_speed: float
    max_angular_speed: float
    k_linear: float
    k_angular: float


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def compute_approach_twist(
    *,
    distance: float,
    bbox_cx: float,
    params: ControlParams,
) -> tuple[float, float]:
    """Return (linear_x, angular_z) for the APPROACH action.

    - Linear: proportional to (distance - target_distance), clamped to [0, max_linear_speed].
      We never back up here; the STOP branch in the BT handles too-close.
    - Angular: proportional to -(bbox_cx - 0.5). Negative because in ROS, positive
      angular.z is a LEFT turn, and bbox_cx < 0.5 means the person is to the LEFT
      of frame center, so we want to turn left (positive angular).
    """
    lin_err = distance - params.target_distance
    v = clamp(params.k_linear * lin_err, 0.0, params.max_linear_speed)

    ang_err = -(bbox_cx - 0.5)
    w = clamp(params.k_angular * ang_err, -params.max_angular_speed, params.max_angular_speed)

    return v, w
```

- [ ] **Step 8.5: Run test to verify it passes**

```powershell
cd "d:/ros case study/ros2_ws/src/tb3_follower_behavior"
$env:PYTHONPATH = "."; pytest test/test_control.py -v
```

Expected: all 9 tests pass.

- [ ] **Step 8.6: Commit**

```bash
cd "d:/ros case study"
git add ros2_ws/src/tb3_follower_behavior/tb3_follower_behavior/__init__.py \
        ros2_ws/src/tb3_follower_behavior/tb3_follower_behavior/control.py \
        ros2_ws/src/tb3_follower_behavior/test/test_control.py
git commit -m "feat(behavior): pure controller (compute_approach_twist) + tests"
```

---

## Task 9: Behavior — py_trees behaviours (guards + actions + helpers)

**Environment:** Host (writing), VM (will run later — these import py_trees and geometry_msgs)

**Files:**
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/tb3_follower_behavior/behaviours/__init__.py`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/tb3_follower_behavior/behaviours/helpers.py`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/tb3_follower_behavior/behaviours/guards.py`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/tb3_follower_behavior/behaviours/actions.py`

- [ ] **Step 9.1: behaviours `__init__.py`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/tb3_follower_behavior/behaviours/__init__.py`

```python
"""py_trees behaviours for the TB3 follower."""
```

- [ ] **Step 9.2: `helpers.py` — shared Twist publisher + blackboard schema**

File: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/tb3_follower_behavior/behaviours/helpers.py`

```python
"""Shared blackboard schema + a TwistPublisher class injected into actions."""
from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import Twist


# Blackboard keys (single source of truth — import these everywhere)
KEY_DETECTION = "person/detection"          # tb3_follower_msgs/PersonDetection
KEY_DISTANCE = "person/distance"            # float
KEY_LAST_SEEN_TIME = "person/last_seen_t"   # float, monotonic seconds


@dataclass
class TwistPublisher:
    """Thin wrapper passed into actions so they can publish without holding a Node."""
    publisher: object  # rclpy.publisher.Publisher

    def send(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.publisher.publish(msg)

    def stop(self) -> None:
        self.send(0.0, 0.0)
```

- [ ] **Step 9.3: `guards.py`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/tb3_follower_behavior/behaviours/guards.py`

```python
"""Condition (guard) behaviours — read-only, never publish."""
from __future__ import annotations

import time

import py_trees

from tb3_follower_behavior.behaviours.helpers import (
    KEY_DETECTION,
    KEY_DISTANCE,
    KEY_LAST_SEEN_TIME,
)


class IsPersonDetected(py_trees.behaviour.Behaviour):
    """SUCCESS if a fresh detection (within timeout) exists on the blackboard."""

    def __init__(self, name: str = "IsPersonDetected", timeout_s: float = 1.0):
        super().__init__(name)
        self.timeout_s = timeout_s
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(KEY_LAST_SEEN_TIME, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            last_t = self.bb.get(KEY_LAST_SEEN_TIME)
        except KeyError:
            return py_trees.common.Status.FAILURE
        if last_t is None:
            return py_trees.common.Status.FAILURE
        age = time.monotonic() - float(last_t)
        return (
            py_trees.common.Status.SUCCESS
            if age <= self.timeout_s
            else py_trees.common.Status.FAILURE
        )


class ReadDistance(py_trees.behaviour.Behaviour):
    """Copies detection.distance from the blackboard into KEY_DISTANCE.

    Returns SUCCESS if a detection exists, FAILURE otherwise.
    """

    def __init__(self, name: str = "ReadDistance"):
        super().__init__(name)
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(KEY_DETECTION, access=py_trees.common.Access.READ)
        self.bb.register_key(KEY_DISTANCE, access=py_trees.common.Access.WRITE)

    def update(self) -> py_trees.common.Status:
        try:
            det = self.bb.get(KEY_DETECTION)
        except KeyError:
            return py_trees.common.Status.FAILURE
        if det is None or not getattr(det, "detected", False):
            return py_trees.common.Status.FAILURE
        self.bb.set(KEY_DISTANCE, float(det.distance))
        return py_trees.common.Status.SUCCESS


class DistanceWithin(py_trees.behaviour.Behaviour):
    """SUCCESS if blackboard distance ∈ [lo, hi]. Used as IsTooClose / IsTooFar / InRange."""

    def __init__(self, name: str, lo: float, hi: float):
        super().__init__(name)
        self.lo = lo
        self.hi = hi
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(KEY_DISTANCE, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            d = float(self.bb.get(KEY_DISTANCE))
        except (KeyError, TypeError):
            return py_trees.common.Status.FAILURE
        return (
            py_trees.common.Status.SUCCESS
            if self.lo <= d <= self.hi
            else py_trees.common.Status.FAILURE
        )


class PersonLostTimer(py_trees.behaviour.Behaviour):
    """SUCCESS if it has been MORE than timeout_s since we last saw a person."""

    def __init__(self, name: str = "PersonLostTimer", timeout_s: float = 1.0):
        super().__init__(name)
        self.timeout_s = timeout_s
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(KEY_LAST_SEEN_TIME, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            last_t = self.bb.get(KEY_LAST_SEEN_TIME)
        except KeyError:
            last_t = None
        if last_t is None:
            return py_trees.common.Status.SUCCESS  # never seen — definitely "lost"
        age = time.monotonic() - float(last_t)
        return (
            py_trees.common.Status.SUCCESS
            if age > self.timeout_s
            else py_trees.common.Status.FAILURE
        )
```

- [ ] **Step 9.4: `actions.py`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/tb3_follower_behavior/behaviours/actions.py`

```python
"""Action behaviours — publish Twist via injected TwistPublisher."""
from __future__ import annotations

import py_trees

from tb3_follower_behavior.behaviours.helpers import (
    KEY_DETECTION,
    KEY_DISTANCE,
    TwistPublisher,
)
from tb3_follower_behavior.control import (
    ControlParams,
    compute_approach_twist,
)


class Stop(py_trees.behaviour.Behaviour):
    """Publish zero Twist. Returns SUCCESS."""

    def __init__(self, twist_pub: TwistPublisher, name: str = "Stop"):
        super().__init__(name)
        self.twist_pub = twist_pub

    def update(self) -> py_trees.common.Status:
        self.twist_pub.stop()
        return py_trees.common.Status.SUCCESS


class Approach(py_trees.behaviour.Behaviour):
    """Drive toward the person using the pure controller. Returns RUNNING while driving."""

    def __init__(
        self,
        twist_pub: TwistPublisher,
        params: ControlParams,
        name: str = "Approach",
    ):
        super().__init__(name)
        self.twist_pub = twist_pub
        self.params = params
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(KEY_DETECTION, access=py_trees.common.Access.READ)
        self.bb.register_key(KEY_DISTANCE, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            det = self.bb.get(KEY_DETECTION)
            dist = float(self.bb.get(KEY_DISTANCE))
        except (KeyError, TypeError):
            self.twist_pub.stop()
            return py_trees.common.Status.FAILURE
        if det is None or not getattr(det, "detected", False):
            self.twist_pub.stop()
            return py_trees.common.Status.FAILURE
        v, w = compute_approach_twist(
            distance=dist,
            bbox_cx=float(det.bbox_cx),
            params=self.params,
        )
        self.twist_pub.send(v, w)
        return py_trees.common.Status.RUNNING


class HoldPosition(py_trees.behaviour.Behaviour):
    """In-range: stop linear motion but keep yaw-tracking the person.

    angular.z scales with bbox_cx offset only (no forward velocity).
    """

    def __init__(
        self,
        twist_pub: TwistPublisher,
        params: ControlParams,
        name: str = "HoldPosition",
    ):
        super().__init__(name)
        self.twist_pub = twist_pub
        self.params = params
        self.bb = self.attach_blackboard_client(name=name)
        self.bb.register_key(KEY_DETECTION, access=py_trees.common.Access.READ)

    def update(self) -> py_trees.common.Status:
        try:
            det = self.bb.get(KEY_DETECTION)
        except KeyError:
            self.twist_pub.stop()
            return py_trees.common.Status.FAILURE
        if det is None or not getattr(det, "detected", False):
            self.twist_pub.stop()
            return py_trees.common.Status.FAILURE
        # Reuse approach controller with distance=target so linear is 0.
        _, w = compute_approach_twist(
            distance=self.params.target_distance,
            bbox_cx=float(det.bbox_cx),
            params=self.params,
        )
        self.twist_pub.send(0.0, w)
        return py_trees.common.Status.RUNNING


class RotateInPlace(py_trees.behaviour.Behaviour):
    """Spin to search. Returns RUNNING forever (until parent re-evaluates)."""

    def __init__(self, twist_pub: TwistPublisher, yaw_rate: float, name: str = "RotateInPlace"):
        super().__init__(name)
        self.twist_pub = twist_pub
        self.yaw_rate = yaw_rate

    def update(self) -> py_trees.common.Status:
        self.twist_pub.send(0.0, self.yaw_rate)
        return py_trees.common.Status.RUNNING


class Idle(py_trees.behaviour.Behaviour):
    """Safety net: zero Twist, returns RUNNING."""

    def __init__(self, twist_pub: TwistPublisher, name: str = "Idle"):
        super().__init__(name)
        self.twist_pub = twist_pub

    def update(self) -> py_trees.common.Status:
        self.twist_pub.stop()
        return py_trees.common.Status.RUNNING
```

- [ ] **Step 9.5: Commit**

```bash
cd "d:/ros case study"
git add ros2_ws/src/tb3_follower_behavior/tb3_follower_behavior/behaviours/
git commit -m "feat(behavior): py_trees guards + actions + shared helpers"
```

---

## Task 10: Behavior — tree assembly, BT node, package metadata, params

**Environment:** Host (writing), VM (will build + run)

**Files:**
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/package.xml`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/setup.py`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/setup.cfg`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/resource/tb3_follower_behavior`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/tb3_follower_behavior/tree.py`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/tb3_follower_behavior/follower_bt_node.py`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/config/follower_params.yaml`

- [ ] **Step 10.1: `package.xml`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>tb3_follower_behavior</name>
  <version>0.1.0</version>
  <description>py_trees behavior tree node for the TB3 follower case study.</description>
  <maintainer email="midhunr2015@gmail.com">Midhun</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>py_trees</exec_depend>
  <exec_depend>py_trees_ros</exec_depend>
  <exec_depend>tb3_follower_msgs</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 10.2: `setup.py`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/setup.py`

```python
from setuptools import setup, find_packages
from glob import glob
import os

package_name = "tb3_follower_behavior"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "config"),
         glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Midhun",
    maintainer_email="midhunr2015@gmail.com",
    description="py_trees behavior tree for TB3 person following",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "follower_bt_node = tb3_follower_behavior.follower_bt_node:main",
        ],
    },
)
```

- [ ] **Step 10.3: `setup.cfg`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/setup.cfg`

```ini
[develop]
script_dir=$base/lib/tb3_follower_behavior
[install]
install_scripts=$base/lib/tb3_follower_behavior
```

- [ ] **Step 10.4: `resource/tb3_follower_behavior` marker**

File: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/resource/tb3_follower_behavior`

(empty)

- [ ] **Step 10.5: Params YAML**

File: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/config/follower_params.yaml`

```yaml
follower_bt_node:
  ros__parameters:
    target_distance: 1.0
    close_threshold: 0.8
    far_threshold: 1.2
    max_linear_speed: 0.22
    max_angular_speed: 1.0
    k_linear: 0.4
    k_angular: 1.5
    person_lost_timeout: 1.0
    search_yaw_rate: 0.3
    tick_rate_hz: 10.0
    detection_topic: "/person/detection"
    cmd_vel_topic: "/cmd_vel"
```

- [ ] **Step 10.6: `tree.py` — assembles the tree**

File: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/tb3_follower_behavior/tree.py`

```python
"""Behavior tree assembly. Pure-Python (no rclpy) — the BT node injects publishers."""
from __future__ import annotations

import py_trees

from tb3_follower_behavior.control import ControlParams
from tb3_follower_behavior.behaviours.helpers import TwistPublisher
from tb3_follower_behavior.behaviours.guards import (
    IsPersonDetected,
    ReadDistance,
    DistanceWithin,
    PersonLostTimer,
)
from tb3_follower_behavior.behaviours.actions import (
    Stop,
    Approach,
    HoldPosition,
    RotateInPlace,
    Idle,
)


def build_tree(
    *,
    twist_pub: TwistPublisher,
    params: ControlParams,
    person_lost_timeout: float,
    search_yaw_rate: float,
) -> py_trees.behaviour.Behaviour:
    """Returns the root behaviour of the follower tree.

    Root (Selector)
    ├── FOLLOW (Sequence)
    │   ├── IsPersonDetected
    │   ├── ReadDistance
    │   └── Selector
    │       ├── Sequence(IsTooClose, Stop)
    │       ├── Sequence(IsTooFar, Approach)
    │       └── Sequence(InRange, HoldPosition)
    ├── SEARCH (Sequence)
    │   ├── PersonLostTimer
    │   └── RotateInPlace
    └── Idle
    """
    # ----- FOLLOW branch -----
    distance_selector = py_trees.composites.Selector(
        name="DistanceSelector", memory=False
    )
    distance_selector.add_children([
        py_trees.composites.Sequence(name="TooClose->Stop", memory=False, children=[
            DistanceWithin(name="IsTooClose", lo=-float("inf"), hi=params.close_threshold),
            Stop(twist_pub=twist_pub),
        ]),
        py_trees.composites.Sequence(name="TooFar->Approach", memory=False, children=[
            DistanceWithin(name="IsTooFar", lo=params.far_threshold, hi=float("inf")),
            Approach(twist_pub=twist_pub, params=params),
        ]),
        py_trees.composites.Sequence(name="InRange->Hold", memory=False, children=[
            DistanceWithin(name="InRange", lo=params.close_threshold, hi=params.far_threshold),
            HoldPosition(twist_pub=twist_pub, params=params),
        ]),
    ])

    follow = py_trees.composites.Sequence(name="FOLLOW", memory=False, children=[
        IsPersonDetected(timeout_s=person_lost_timeout),
        ReadDistance(),
        distance_selector,
    ])

    # ----- SEARCH branch -----
    search = py_trees.composites.Sequence(name="SEARCH", memory=False, children=[
        PersonLostTimer(timeout_s=person_lost_timeout),
        RotateInPlace(twist_pub=twist_pub, yaw_rate=search_yaw_rate),
    ])

    # ----- Root selector -----
    root = py_trees.composites.Selector(name="Root", memory=False, children=[
        follow,
        search,
        Idle(twist_pub=twist_pub),
    ])
    return root
```

- [ ] **Step 10.7: `follower_bt_node.py`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/tb3_follower_behavior/follower_bt_node.py`

```python
"""ROS 2 node that runs the follower behavior tree."""
from __future__ import annotations

import time

import py_trees
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from tb3_follower_msgs.msg import PersonDetection

from tb3_follower_behavior.behaviours.helpers import (
    KEY_DETECTION,
    KEY_LAST_SEEN_TIME,
    TwistPublisher,
)
from tb3_follower_behavior.control import ControlParams
from tb3_follower_behavior.tree import build_tree


class FollowerBTNode(Node):
    def __init__(self) -> None:
        super().__init__("follower_bt_node")

        # ----- Params -----
        self.declare_parameter("target_distance", 1.0)
        self.declare_parameter("close_threshold", 0.8)
        self.declare_parameter("far_threshold", 1.2)
        self.declare_parameter("max_linear_speed", 0.22)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("k_linear", 0.4)
        self.declare_parameter("k_angular", 1.5)
        self.declare_parameter("person_lost_timeout", 1.0)
        self.declare_parameter("search_yaw_rate", 0.3)
        self.declare_parameter("tick_rate_hz", 10.0)
        self.declare_parameter("detection_topic", "/person/detection")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        gp = self.get_parameter
        params = ControlParams(
            target_distance=float(gp("target_distance").value),
            close_threshold=float(gp("close_threshold").value),
            far_threshold=float(gp("far_threshold").value),
            max_linear_speed=float(gp("max_linear_speed").value),
            max_angular_speed=float(gp("max_angular_speed").value),
            k_linear=float(gp("k_linear").value),
            k_angular=float(gp("k_angular").value),
        )
        person_lost_timeout = float(gp("person_lost_timeout").value)
        search_yaw_rate = float(gp("search_yaw_rate").value)
        tick_period = 1.0 / float(gp("tick_rate_hz").value)

        # ----- Publisher + Twist wrapper -----
        self.cmd_pub = self.create_publisher(Twist, gp("cmd_vel_topic").value, 10)
        twist_pub = TwistPublisher(publisher=self.cmd_pub)

        # ----- Detection subscriber -> blackboard -----
        self.bb = py_trees.blackboard.Client(name="follower_bt_node")
        self.bb.register_key(KEY_DETECTION, access=py_trees.common.Access.WRITE)
        self.bb.register_key(KEY_LAST_SEEN_TIME, access=py_trees.common.Access.WRITE)
        # Pre-seed
        self.bb.set(KEY_DETECTION, None)
        self.bb.set(KEY_LAST_SEEN_TIME, None)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            PersonDetection, gp("detection_topic").value,
            self._on_detection, sensor_qos
        )

        # ----- Build tree -----
        self.root = build_tree(
            twist_pub=twist_pub,
            params=params,
            person_lost_timeout=person_lost_timeout,
            search_yaw_rate=search_yaw_rate,
        )
        self.tree = py_trees.trees.BehaviourTree(self.root)
        self.tree.setup(timeout=5.0)

        # ----- Tick timer -----
        self.create_timer(tick_period, self._tick)

        self.get_logger().info(
            f"follower_bt_node ready. Ticking at {1.0/tick_period:.1f} Hz."
        )
        py_trees.display.ascii_tree(self.root)  # print to stdout once for sanity

    def _on_detection(self, msg: PersonDetection) -> None:
        self.bb.set(KEY_DETECTION, msg)
        if msg.detected:
            self.bb.set(KEY_LAST_SEEN_TIME, time.monotonic())

    def _tick(self) -> None:
        self.tree.tick()


def main(args=None):
    rclpy.init(args=args)
    node = FollowerBTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 10.8: Commit**

```bash
cd "d:/ros case study"
git add ros2_ws/src/tb3_follower_behavior/
git commit -m "feat(behavior): tree assembly + BT node + params + package metadata"
```

---

## Task 11: VM-side first build + smoke check (CHECKPOINT — first time on VM)

**Environment:** VM (Ubuntu inside VirtualBox)

Prerequisites: Tasks 0–10 committed. User has executed Task 1 (created the VM) and has Ubuntu running with shared folder mounted at `/mnt/host_project`.

- [ ] **Step 11.1: Run the ROS install script in the VM**

In the VM terminal:

```bash
bash /mnt/host_project/scripts/02-install-ros.sh
```

Expected: completes with `DONE` message. Takes 10–15 minutes.

- [ ] **Step 11.2: Open a NEW terminal and run YOLO install**

```bash
bash /mnt/host_project/scripts/03-install-yolo.sh
```

Expected: `torch`, `ultralytics`, `cv2`, `numpy` versions print. `~/models/yolov8n.pt` exists.

- [ ] **Step 11.3: Open a NEW terminal (so `.bashrc` env applies) and build the workspace**

```bash
bash /mnt/host_project/scripts/04-build-workspace.sh
```

Expected: `colcon build --symlink-install` succeeds for all four packages:
- `tb3_follower_msgs`
- `tb3_follower_perception`
- `tb3_follower_behavior`
- `tb3_follower_bringup` (will be added in later tasks — at this checkpoint, only the first three exist)

- [ ] **Step 11.4: Verify the custom msg is registered**

```bash
source ~/ros2_ws/install/setup.bash
ros2 interface show tb3_follower_msgs/msg/PersonDetection
```

Expected: prints the .msg content.

- [ ] **Step 11.5: Verify both node entry points are installed**

```bash
ros2 pkg executables tb3_follower_perception
ros2 pkg executables tb3_follower_behavior
```

Expected:
```
tb3_follower_perception person_detector_node
tb3_follower_behavior follower_bt_node
```

- [ ] **Step 11.6: If anything fails — fix and re-commit on host**

If a build error occurs, edit the file on the **Windows host** (the shared folder syncs instantly), then in the VM rerun:

```bash
bash /mnt/host_project/scripts/04-build-workspace.sh
```

Common issues and fixes:

| Error                                                  | Fix                                                                |
|--------------------------------------------------------|--------------------------------------------------------------------|
| `package.xml` schema warning                           | Ignore — informational only                                        |
| `No module named 'tb3_follower_msgs'`                  | Source `install/setup.bash`, or check `colcon build` output for msg pkg errors |
| `entry_point ... not found`                            | Verify `entry_points` in `setup.py` matches file name              |
| `permission denied` on shared folder                   | `sudo usermod -aG vboxsf $USER && sudo reboot`                     |
| CRLF line ending error in a `.sh` script               | On host: `git config core.autocrlf false`, re-commit; `.gitattributes` should have caught it |

No commit needed here — this is a verification checkpoint.

---

## Task 12: BT integration test (VM-side)

**Environment:** VM (needs py_trees + tb3_follower_msgs)

**Files:**
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/test/test_tree_ticks.py`

- [ ] **Step 12.1: Write the integration test**

File: `d:/ros case study/ros2_ws/src/tb3_follower_behavior/test/test_tree_ticks.py`

```python
"""Integration test for the behaviour tree.

Runs in the VM only (imports py_trees + tb3_follower_msgs).
"""
from __future__ import annotations

import time
from dataclasses import dataclass

import pytest

from tb3_follower_msgs.msg import PersonDetection
from tb3_follower_behavior.control import ControlParams
from tb3_follower_behavior.behaviours.helpers import (
    KEY_DETECTION,
    KEY_LAST_SEEN_TIME,
    TwistPublisher,
)
from tb3_follower_behavior.tree import build_tree
import py_trees


@dataclass
class FakePublisher:
    """Stand-in for rclpy.publisher.Publisher — records last published Twist."""
    last_linear_x: float = 0.0
    last_angular_z: float = 0.0
    publish_count: int = 0

    def publish(self, msg) -> None:  # ROS-style API
        self.last_linear_x = float(msg.linear.x)
        self.last_angular_z = float(msg.angular.z)
        self.publish_count += 1


@pytest.fixture
def params() -> ControlParams:
    return ControlParams(
        target_distance=1.0,
        close_threshold=0.8,
        far_threshold=1.2,
        max_linear_speed=0.22,
        max_angular_speed=1.0,
        k_linear=0.4,
        k_angular=1.5,
    )


@pytest.fixture
def tree_and_pub(params):
    pub = FakePublisher()
    twist_pub = TwistPublisher(publisher=pub)
    root = build_tree(
        twist_pub=twist_pub,
        params=params,
        person_lost_timeout=1.0,
        search_yaw_rate=0.3,
    )
    # Seed blackboard with the keys the tree expects.
    bb = py_trees.blackboard.Client(name="test_seed")
    bb.register_key(KEY_DETECTION, access=py_trees.common.Access.WRITE)
    bb.register_key(KEY_LAST_SEEN_TIME, access=py_trees.common.Access.WRITE)
    bb.set(KEY_DETECTION, None)
    bb.set(KEY_LAST_SEEN_TIME, None)
    return root, pub, bb


def _make_detection(*, detected: bool, distance: float = -1.0, cx: float = 0.5) -> PersonDetection:
    msg = PersonDetection()
    msg.detected = detected
    msg.distance = float(distance)
    msg.bbox_cx = float(cx)
    msg.bbox_cy = 0.5
    msg.bbox_w = 0.2
    msg.bbox_h = 0.4
    msg.confidence = 0.9
    return msg


class TestTreeBranches:
    def test_no_detection_ever_rotates(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        root.tick_once()
        assert pub.last_linear_x == pytest.approx(0.0)
        assert pub.last_angular_z == pytest.approx(0.3)  # search yaw rate

    def test_person_too_close_stops(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        bb.set(KEY_DETECTION, _make_detection(detected=True, distance=0.5, cx=0.5))
        bb.set(KEY_LAST_SEEN_TIME, time.monotonic())
        root.tick_once()
        assert pub.last_linear_x == pytest.approx(0.0)
        assert pub.last_angular_z == pytest.approx(0.0)

    def test_person_too_far_approaches(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        bb.set(KEY_DETECTION, _make_detection(detected=True, distance=2.0, cx=0.5))
        bb.set(KEY_LAST_SEEN_TIME, time.monotonic())
        root.tick_once()
        assert pub.last_linear_x > 0.0
        assert pub.last_angular_z == pytest.approx(0.0)

    def test_person_far_and_offset_steers(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        bb.set(KEY_DETECTION, _make_detection(detected=True, distance=2.0, cx=0.3))
        bb.set(KEY_LAST_SEEN_TIME, time.monotonic())
        root.tick_once()
        assert pub.last_linear_x > 0.0
        # cx=0.3 -> person to the LEFT -> +angular (turn left in ROS convention)
        assert pub.last_angular_z > 0.0

    def test_person_in_range_holds_position(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        bb.set(KEY_DETECTION, _make_detection(detected=True, distance=1.0, cx=0.5))
        bb.set(KEY_LAST_SEEN_TIME, time.monotonic())
        root.tick_once()
        assert pub.last_linear_x == pytest.approx(0.0)
        assert pub.last_angular_z == pytest.approx(0.0)

    def test_stale_detection_triggers_search(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        # Detection from 5 seconds ago -> stale (timeout is 1s)
        bb.set(KEY_DETECTION, _make_detection(detected=True, distance=1.0, cx=0.5))
        bb.set(KEY_LAST_SEEN_TIME, time.monotonic() - 5.0)
        root.tick_once()
        assert pub.last_angular_z == pytest.approx(0.3)

    def test_lost_then_found_resumes(self, tree_and_pub):
        root, pub, bb = tree_and_pub
        # Phase 1: no person -> SEARCH
        root.tick_once()
        assert pub.last_angular_z == pytest.approx(0.3)
        # Phase 2: fresh detection appears
        bb.set(KEY_DETECTION, _make_detection(detected=True, distance=2.0, cx=0.5))
        bb.set(KEY_LAST_SEEN_TIME, time.monotonic())
        root.tick_once()
        assert pub.last_linear_x > 0.0  # FOLLOW branch took over
```

- [ ] **Step 12.2: Run the integration test in the VM**

In the VM:

```bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/tb3_follower_behavior
python3 -m pytest test/test_tree_ticks.py -v
```

Expected: all 7 tests pass.

- [ ] **Step 12.3: Commit**

```bash
cd "d:/ros case study"
git add ros2_ws/src/tb3_follower_behavior/test/test_tree_ticks.py
git commit -m "test(behavior): integration tests for tree branch transitions"
```

---

## Task 13: Bringup — Gazebo world with walking actor

**Environment:** Host (writing), VM (will run later)

**Files:**
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_bringup/package.xml`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_bringup/CMakeLists.txt`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_bringup/worlds/follow_world.world`

- [ ] **Step 13.1: `package.xml`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_bringup/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>tb3_follower_bringup</name>
  <version>0.1.0</version>
  <description>Launch files, Gazebo world, and RViz config for the TB3 follower demo.</description>
  <maintainer email="midhunr2015@gmail.com">Midhun</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>turtlebot3_gazebo</exec_depend>
  <exec_depend>gazebo_ros</exec_depend>
  <exec_depend>tb3_follower_perception</exec_depend>
  <exec_depend>tb3_follower_behavior</exec_depend>
  <exec_depend>tb3_follower_msgs</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 13.2: `CMakeLists.txt`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_bringup/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.8)
project(tb3_follower_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

- [ ] **Step 13.3: `follow_world.world` — empty room + walking actor**

File: `d:/ros case study/ros2_ws/src/tb3_follower_bringup/worlds/follow_world.world`

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="follow_world">

    <!-- Light + ground -->
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

    <!-- Simple enclosure: 4 walls forming a 10x10 m room -->
    <model name="wall_north">
      <static>true</static>
      <pose>0 5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>10 0.2 1</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>10 0.2 1</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient></material></visual>
      </link>
    </model>
    <model name="wall_south">
      <static>true</static>
      <pose>0 -5 0.5 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>10 0.2 1</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>10 0.2 1</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient></material></visual>
      </link>
    </model>
    <model name="wall_east">
      <static>true</static>
      <pose>5 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>0.2 10 1</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.2 10 1</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient></material></visual>
      </link>
    </model>
    <model name="wall_west">
      <static>true</static>
      <pose>-5 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="col"><geometry><box><size>0.2 10 1</size></box></geometry></collision>
        <visual name="vis"><geometry><box><size>0.2 10 1</size></box></geometry>
          <material><ambient>0.6 0.6 0.6 1</ambient></material></visual>
      </link>
    </model>

    <!-- Walking human actor (Gazebo built-in animation: walk.dae) -->
    <actor name="walking_person">
      <pose>2 0 0 0 0 0</pose>
      <skin>
        <filename>walk.dae</filename>
        <scale>1.0</scale>
      </skin>
      <animation name="walking">
        <filename>walk.dae</filename>
        <interpolate_x>true</interpolate_x>
      </animation>
      <script>
        <loop>true</loop>
        <delay_start>0.0</delay_start>
        <auto_start>true</auto_start>
        <trajectory id="0" type="walking">
          <waypoint><time>0</time>  <pose>2  0  0 0 0 1.5708</pose></waypoint>
          <waypoint><time>8</time>  <pose>2  2  0 0 0 3.1416</pose></waypoint>
          <waypoint><time>16</time> <pose>-1 2  0 0 0 -1.5708</pose></waypoint>
          <waypoint><time>24</time> <pose>-1 -1 0 0 0 0</pose></waypoint>
          <waypoint><time>32</time> <pose>2  -1 0 0 0 1.5708</pose></waypoint>
          <waypoint><time>40</time> <pose>2  0  0 0 0 1.5708</pose></waypoint>
        </trajectory>
      </script>
    </actor>

    <!-- Sim physics defaults: keep real-time factor near 1.0 -->
    <physics name="default_physics" default="0" type="ode">
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

  </world>
</sdf>
```

- [ ] **Step 13.4: Commit**

```bash
cd "d:/ros case study"
git add ros2_ws/src/tb3_follower_bringup/package.xml \
        ros2_ws/src/tb3_follower_bringup/CMakeLists.txt \
        ros2_ws/src/tb3_follower_bringup/worlds/follow_world.world
git commit -m "feat(bringup): Gazebo world with walking human actor"
```

---

## Task 14: Bringup — launch files

**Environment:** Host (writing), VM (will run)

**Files:**
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_bringup/launch/sim.launch.py`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_bringup/launch/perception.launch.py`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_bringup/launch/behavior.launch.py`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_bringup/launch/follower.launch.py`
- Create: `d:/ros case study/ros2_ws/src/tb3_follower_bringup/rviz/follower.rviz`

- [ ] **Step 14.1: `sim.launch.py`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_bringup/launch/sim.launch.py`

```python
"""Launch Gazebo with the follow_world and spawn a TurtleBot3 Waffle Pi."""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory("tb3_follower_bringup")
    tb3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    gazebo_ros_share = get_package_share_directory("gazebo_ros")

    world_path = os.path.join(bringup_share, "worlds", "follow_world.world")

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world_path, "verbose": "true"}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, "launch", "gzclient.launch.py")
        ),
    )

    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_share, "launch", "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={
            "x_pose": LaunchConfiguration("x_pose"),
            "y_pose": LaunchConfiguration("y_pose"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("x_pose", default_value="0.0"),
        DeclareLaunchArgument("y_pose", default_value="0.0"),
        gzserver,
        gzclient,
        spawn_tb3,
    ])
```

- [ ] **Step 14.2: `perception.launch.py`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_bringup/launch/perception.launch.py`

```python
"""Launch the YOLOv8 person detector node with its params YAML."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    perception_share = get_package_share_directory("tb3_follower_perception")
    params = os.path.join(perception_share, "config", "detector_params.yaml")

    return LaunchDescription([
        Node(
            package="tb3_follower_perception",
            executable="person_detector_node",
            name="person_detector_node",
            output="screen",
            parameters=[params],
            emulate_tty=True,
        ),
    ])
```

- [ ] **Step 14.3: `behavior.launch.py`**

File: `d:/ros case study/ros2_ws/src/tb3_follower_bringup/launch/behavior.launch.py`

```python
"""Launch the follower behavior tree node."""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    behavior_share = get_package_share_directory("tb3_follower_behavior")
    params = os.path.join(behavior_share, "config", "follower_params.yaml")

    return LaunchDescription([
        Node(
            package="tb3_follower_behavior",
            executable="follower_bt_node",
            name="follower_bt_node",
            output="screen",
            parameters=[params],
            emulate_tty=True,
        ),
    ])
```

- [ ] **Step 14.4: `follower.launch.py` — composed entry point**

File: `d:/ros case study/ros2_ws/src/tb3_follower_bringup/launch/follower.launch.py`

```python
"""Top-level launch: sim + perception + behavior, all together."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bringup_share = get_package_share_directory("tb3_follower_bringup")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "sim.launch.py"))
    )
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "perception.launch.py"))
    )
    behavior = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, "launch", "behavior.launch.py"))
    )

    # Delay perception + behavior until Gazebo is up and camera/scan topics are publishing.
    return LaunchDescription([
        sim,
        TimerAction(period=8.0, actions=[perception]),
        TimerAction(period=9.0, actions=[behavior]),
    ])
```

- [ ] **Step 14.5: `follower.rviz` — preconfigured visualization**

File: `d:/ros case study/ros2_ws/src/tb3_follower_bringup/rviz/follower.rviz`

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
    Property Tree Widget:
      Expanded: ~
      Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Class: rviz_default_plugins/RobotModel
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Name: RobotModel
      Value: true
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Class: rviz_default_plugins/LaserScan
      Color: 239; 41; 41
      Enabled: true
      Name: LaserScan
      Position Transformer: XYZ
      Size (m): 0.05
      Style: Points
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
      Value: true
    - Class: rviz_default_plugins/Image
      Enabled: true
      Name: Camera
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /camera/image_raw
      Value: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_footprint
    Frame Rate: 30
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 5
      Focal Point: {X: 0, Y: 0, Z: 0}
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.785
      Target Frame: <Fixed Frame>
      Yaw: 0.785
    Saved: ~
Window Geometry:
  Height: 800
  Width: 1200
```

- [ ] **Step 14.6: Commit**

```bash
cd "d:/ros case study"
git add ros2_ws/src/tb3_follower_bringup/launch/ \
        ros2_ws/src/tb3_follower_bringup/rviz/
git commit -m "feat(bringup): launch files + RViz config"
```

---

## Task 15: Demo runner script (tmux 3-pane)

**Environment:** Host (writing), VM (executing)

**Files:**
- Create: `d:/ros case study/scripts/05-run-demo.sh`

- [ ] **Step 15.1: Write the tmux launcher**

File: `d:/ros case study/scripts/05-run-demo.sh`

```bash
#!/usr/bin/env bash
# 05-run-demo.sh — Launch the full demo in a 3-pane tmux session.
# Pane 0: gazebo + tb3 spawn (sim.launch.py)
# Pane 1: YOLO person detector
# Pane 2: behavior tree node

set -euo pipefail

SESSION="tb3-follower"

if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux not installed. Run scripts/02-install-ros.sh first." >&2
    exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "Session '$SESSION' already running. Attach with: tmux attach -t $SESSION"
    echo "To kill: tmux kill-session -t $SESSION"
    exit 0
fi

# All panes need the env vars sourced
SETUP='source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models'

tmux new-session  -d -s "$SESSION" -n demo
tmux send-keys    -t "$SESSION:demo.0" \
    "$SETUP && ros2 launch tb3_follower_bringup sim.launch.py" C-m

tmux split-window -h -t "$SESSION:demo"
tmux send-keys    -t "$SESSION:demo.1" \
    "$SETUP && sleep 8 && ros2 launch tb3_follower_bringup perception.launch.py" C-m

tmux split-window -v -t "$SESSION:demo.1"
tmux send-keys    -t "$SESSION:demo.2" \
    "$SETUP && sleep 10 && ros2 launch tb3_follower_bringup behavior.launch.py" C-m

tmux select-pane  -t "$SESSION:demo.0"
tmux attach       -t "$SESSION"
```

- [ ] **Step 15.2: Commit**

```bash
cd "d:/ros case study"
git add scripts/05-run-demo.sh
git update-index --chmod=+x scripts/05-run-demo.sh
git commit -m "feat(scripts): tmux 3-pane demo runner"
```

---

## Task 16: Setup docs + architecture doc

**Environment:** Host (writing)

**Files:**
- Create: `d:/ros case study/docs/setup/02-install-ros.md`
- Create: `d:/ros case study/docs/setup/03-run-demo.md`
- Create: `d:/ros case study/docs/architecture.md`

- [ ] **Step 16.1: `02-install-ros.md`**

File: `d:/ros case study/docs/setup/02-install-ros.md`

````markdown
# 02 — Install ROS 2 + YOLO + Build the Workspace

Run these three scripts in the VM, in order. Each is idempotent — safe to re-run after a partial failure.

## 1. ROS 2 Humble + TurtleBot3 + Gazebo

```bash
bash /mnt/host_project/scripts/02-install-ros.sh
```

What it does:
- Sets UTF-8 locale.
- Adds the official ROS 2 apt repository.
- Installs `ros-humble-desktop`, `ros-humble-turtlebot3`, `ros-humble-turtlebot3-gazebo`, `ros-humble-cv-bridge`.
- Initializes rosdep.
- Installs tmux.
- Adds env vars to `~/.bashrc` (TURTLEBOT3_MODEL, sources `/opt/ros/humble/setup.bash`).

**Wait 10–15 minutes.** When it finishes, **open a fresh terminal** so the new `.bashrc` takes effect.

## 2. YOLOv8 + py_trees

```bash
bash /mnt/host_project/scripts/03-install-yolo.sh
```

What it does:
- Installs Python 3.10 + pip.
- Installs CPU-only PyTorch 2.2.
- Installs Ultralytics, OpenCV (headless), NumPy.
- Installs `ros-humble-py-trees` + `ros-humble-py-trees-ros`.
- Pre-downloads `yolov8n.pt` to `~/models/yolov8n.pt`.
- Runs an import sanity check.

Sanity output should list torch, ultralytics, cv2, numpy versions.

## 3. Build the workspace

```bash
bash /mnt/host_project/scripts/04-build-workspace.sh
```

What it does:
- Symlinks `/mnt/host_project/ros2_ws` → `~/ros2_ws` so colcon writes its `build/`, `install/`, `log/` dirs locally (avoids VirtualBox shared-folder permission quirks).
- Runs `rosdep install` for any missing deps.
- Runs `colcon build --symlink-install`.

Expected: all four packages build with no errors.

## 4. Quick sanity check

```bash
source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep tb3_follower
ros2 interface show tb3_follower_msgs/msg/PersonDetection
ros2 pkg executables tb3_follower_perception
ros2 pkg executables tb3_follower_behavior
```

If all four `tb3_follower_*` packages appear and the executables list `person_detector_node` and `follower_bt_node`, you are ready to run the demo. Proceed to `03-run-demo.md`.

## Troubleshooting

| Symptom                                             | Fix                                                                |
|-----------------------------------------------------|--------------------------------------------------------------------|
| `colcon: command not found`                         | New terminal needed after `02-install-ros.sh`.                     |
| `No module named 'ultralytics'`                     | Re-run `03-install-yolo.sh`. Check `pip install --user` succeeded. |
| `CrLf` errors building scripts                      | Host issue. Re-clone using `git config core.autocrlf input`.       |
| `Failed to find package 'tb3_follower_msgs'`        | `colcon build` did not complete — re-run `04-build-workspace.sh`.  |
| `libGL error: failed to load driver: swrast`        | Add `export LIBGL_ALWAYS_SOFTWARE=1` (the install script does this — open new terminal). |
````

- [ ] **Step 16.2: `03-run-demo.md`**

File: `d:/ros case study/docs/setup/03-run-demo.md`

````markdown
# 03 — Run the Demo

In the VM, run:

```bash
bash /mnt/host_project/scripts/05-run-demo.sh
```

This opens a tmux session named `tb3-follower` with three panes:

```
┌─────────────────────────┬─────────────────────────┐
│                         │  YOLO person detector   │
│   Gazebo + TurtleBot3   │                         │
│                         ├─────────────────────────┤
│                         │  Behavior tree node     │
└─────────────────────────┴─────────────────────────┘
```

Gazebo will open in a separate window. **First boot of Gazebo is slow** (downloading the `walk.dae` and tb3 meshes the first time, often 30–60s).

## What to watch

**Pane 0 (Gazebo):** A 10×10 m room with the TurtleBot3 at the origin and a walking person actor. The actor walks a loop around the room.

**Pane 1 (perception):** Logs `Focal length captured: fx=...` once at startup. Otherwise quiet unless YOLO inference is too slow (`YOLO inference slow (XXX ms)`).

**Pane 2 (BT):** Prints the ASCII tree once at startup, then ticks at 10 Hz.

## Manual smoke test checklist

Tick each:

- [ ] Robot rotates in place at startup (SEARCH branch — person not yet seen by camera).
- [ ] When the walking actor comes into the camera frame, robot starts driving toward it.
- [ ] Robot stops at ~1m from the actor.
- [ ] When the actor walks out of frame, robot rotates within ~1s to search.
- [ ] Robot re-engages when the actor comes back into view.

## Tmux quick reference

- **Detach (keep running):** `Ctrl+B`, then `D`
- **Re-attach:** `tmux attach -t tb3-follower`
- **Switch pane:** `Ctrl+B`, then arrow key
- **Kill all:** `tmux kill-session -t tb3-follower`

## Inspecting topics live

In a separate terminal:

```bash
source ~/ros2_ws/install/setup.bash

# Watch detections
ros2 topic echo /person/detection

# Watch cmd_vel
ros2 topic echo /cmd_vel

# Topic list
ros2 topic list

# Topic rate
ros2 topic hz /person/detection   # should be near 10 Hz
ros2 topic hz /cmd_vel            # should be 10 Hz
```

## Visualize the camera + scan in RViz

```bash
source ~/ros2_ws/install/setup.bash
rviz2 -d ~/ros2_ws/src/tb3_follower_bringup/rviz/follower.rviz
```

## Tuning

If the robot oscillates or doesn't follow well, edit:

```
ros2_ws/src/tb3_follower_behavior/config/follower_params.yaml
ros2_ws/src/tb3_follower_perception/config/detector_params.yaml
```

Useful knobs:
- `target_distance` — distance the robot tries to maintain (default 1.0 m)
- `k_linear`, `k_angular` — controller gains; lower if oscillating
- `conf_threshold` — YOLO confidence; lower to detect at longer range
- `max_rate_hz` — perception rate; lower if VM is struggling

Rebuild after editing (in the VM):

```bash
cd ~/ros2_ws && colcon build --symlink-install
```

(With `--symlink-install`, YAML changes don't actually need a rebuild — but it's a safe habit.)
````

- [ ] **Step 16.3: `architecture.md`**

File: `d:/ros case study/docs/architecture.md`

````markdown
# Architecture

## System diagram

```
┌────────────────────────────────────────────────────────────┐
│  Ubuntu 22.04 VM (VirtualBox)                              │
│                                                            │
│  ┌─────────────────┐    /camera/image_raw                  │
│  │ Gazebo Classic  │ ─────────────────────┐                │
│  │  + TurtleBot3   │    /scan             │                │
│  │   Waffle Pi     │ ──────────────┐      │                │
│  │  + walking      │               │      │                │
│  │   actor (human) │ ←─/cmd_vel─┐  │      │                │
│  └─────────────────┘            │  │      │                │
│                                 │  │      ▼                │
│                                 │  │  ┌──────────────────┐ │
│                                 │  │  │ person_detector  │ │
│                                 │  │  │  (YOLOv8n)       │ │
│                                 │  │  └────────┬─────────┘ │
│                                 │  │           │           │
│                                 │  │    /person/detection  │
│                                 │  │           ▼           │
│                                 │  │  ┌──────────────────┐ │
│                                 │  └──│ follower_bt      │ │
│                                 │     │  (py_trees_ros)  │ │
│                                 └─────│                  │ │
│                                       └──────────────────┘ │
└────────────────────────────────────────────────────────────┘
```

## Topics

| Topic                  | Type                              | Producer        | Consumer        |
|------------------------|-----------------------------------|-----------------|-----------------|
| `/camera/image_raw`    | sensor_msgs/Image                 | Gazebo (TB3 cam)| person_detector |
| `/camera/camera_info`  | sensor_msgs/CameraInfo            | Gazebo          | person_detector |
| `/scan`                | sensor_msgs/LaserScan             | Gazebo (TB3 LiDAR)| person_detector + follower_bt |
| `/person/detection`    | tb3_follower_msgs/PersonDetection | person_detector | follower_bt     |
| `/cmd_vel`             | geometry_msgs/Twist               | follower_bt     | Gazebo (TB3)    |

## Behavior tree

```
Root (Selector)
├── FOLLOW (Sequence)              ← runs if person currently visible
│   ├── IsPersonDetected           ← guard: detection within last 1s
│   ├── ReadDistance               ← copies detection.distance → blackboard
│   └── DistanceSelector
│       ├── Sequence(IsTooClose, Stop)
│       ├── Sequence(IsTooFar, Approach)
│       └── Sequence(InRange, HoldPosition)
├── SEARCH (Sequence)              ← runs if no person for > 1s
│   ├── PersonLostTimer
│   └── RotateInPlace
└── Idle                           ← safety net, publishes zero Twist
```

## Code organization principle

Pure-math modules (`geometry.py`, `control.py`) import only `numpy`/`dataclasses` and have no ROS dependencies. They are unit-tested with `pytest` on the host. ROS-dependent modules (the node files and `behaviours/*.py`) are tested with integration tests inside the VM after `colcon build`. This gives a fast TDD loop without needing a full ROS install just to verify maths.

## Distance estimation

Two-source fusion:
1. **Visual:** pinhole model — `d = (1.7 * focal_px) / bbox_h_px` where focal length comes from `/camera/camera_info` and 1.7m is the assumed person height.
2. **LiDAR:** map the bbox center x to a scan beam index and read the range there.

If the LiDAR reading is valid and within range, use it. Otherwise fall back to the visual estimate. Logic in `tb3_follower_perception/geometry.py:fuse_distance`.

## Control law (APPROACH action)

```
v = clamp(k_linear  * (distance - target_distance), 0, max_linear_speed)
w = clamp(k_angular * -(bbox_cx - 0.5),             -max_yaw, max_yaw)
```

Encoded in `tb3_follower_behavior/control.py:compute_approach_twist`. Tested in `test_control.py`.

## Dead-band

`close_threshold < d < far_threshold` triggers the `HoldPosition` action: zero linear velocity, angular only. Prevents oscillation around the target distance.
````

- [ ] **Step 16.4: Commit**

```bash
cd "d:/ros case study"
git add docs/setup/02-install-ros.md \
        docs/setup/03-run-demo.md \
        docs/architecture.md
git commit -m "docs: install guide, demo runbook, architecture overview"
```

---

## Task 17: VM-side full build + demo (CHECKPOINT 2 — final integration)

**Environment:** VM

Prerequisites: Tasks 0–16 committed and pushed to the shared folder.

- [ ] **Step 17.1: Re-source env**

```bash
source ~/.bashrc
```

- [ ] **Step 17.2: Build the full workspace (now including bringup)**

```bash
bash /mnt/host_project/scripts/04-build-workspace.sh
```

Expected: all four `tb3_follower_*` packages build cleanly.

- [ ] **Step 17.3: Verify all launch files load (dry parse)**

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch tb3_follower_bringup sim.launch.py --print
ros2 launch tb3_follower_bringup perception.launch.py --print
ros2 launch tb3_follower_bringup behavior.launch.py --print
ros2 launch tb3_follower_bringup follower.launch.py --print
```

Expected: each prints the launch graph. No errors.

- [ ] **Step 17.4: Run the demo**

```bash
bash /mnt/host_project/scripts/05-run-demo.sh
```

Expected: tmux opens. Gazebo window appears within ~30s. After ~10s the BT pane prints the ASCII tree.

- [ ] **Step 17.5: Verify topics**

In a second terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic hz /person/detection
ros2 topic hz /cmd_vel
ros2 topic echo /person/detection --once
```

Expected:
- `/person/detection` rate ~10 Hz
- `/cmd_vel` rate ~10 Hz
- Detection echo shows either `detected: true` (with bbox + distance) or `detected: false`

- [ ] **Step 17.6: Walk through smoke checklist from `03-run-demo.md`**

Tick each item:
- Robot rotates in place at startup.
- Robot drives toward walking actor when in view.
- Robot stops at ~1m.
- Robot rotates again when actor walks out of view.
- Robot re-engages when actor returns.

- [ ] **Step 17.7: Commit a SUCCESS marker file**

This is the canonical end-of-implementation commit.

File: `d:/ros case study/docs/DEMO_VERIFIED.md`

```markdown
# Demo verification

Demo executed end-to-end on $(date).

Checklist all green:
- [x] Robot searches when no person visible
- [x] Robot follows walking actor
- [x] Robot maintains ~1m safe distance
- [x] Robot recovers when person re-enters frame

Environment:
- Host: Windows 11
- VM: Ubuntu 22.04.4 LTS in VirtualBox (D: drive)
- ROS: 2 Humble
- Model: yolov8n.pt
```

```bash
cd "d:/ros case study"
git add docs/DEMO_VERIFIED.md
git commit -m "docs: demo verified end-to-end"
```

---

## Self-Review (post-write check)

**Spec coverage map (spec section → implementing task):**

| Spec section                                       | Task(s)                          |
|----------------------------------------------------|----------------------------------|
| §1 Goal + demo success criteria                    | 17 (manual checklist)            |
| §2 Stack                                           | 2, 3 (install scripts)           |
| §3 System architecture + topics                    | 5, 7, 10 (msg, perception, BT)   |
| §3 PersonDetection.msg                             | 5                                |
| §4 Perception node pipeline                        | 6 (geometry), 7 (node)           |
| §4 Throttle / frame drop                           | 7 (busy guard)                   |
| §4 Distance fusion (visual + LiDAR)                | 6 (fuse_distance), 7 (callsite)  |
| §5 Behavior tree structure                         | 9 (behaviours), 10 (tree.py)     |
| §5 Blackboard keys                                 | 9 (helpers.py)                   |
| §5 Control law                                     | 8 (compute_approach_twist)       |
| §5 Params YAML                                     | 10 (follower_params.yaml)        |
| §6 Simulation world (walking actor)                | 13                               |
| §7 Repository layout                               | 0, 5, 7, 10, 13, 14              |
| §7 Shared folder workflow                          | 1, 4                             |
| §8 VirtualBox VM specs                             | 1                                |
| §8 Env vars in .bashrc                             | 2                                |
| §9 Setup flow                                      | 1, 16                            |
| §10 Unit tests (distance, tree ticks)              | 6, 8, 12                         |
| §10 Manual sim test                                | 17                               |
| §11 Risks/mitigations                              | covered by code in 2,4,7,10      |
| §12 Out of scope                                   | not implemented (by design)      |

All spec sections have implementing tasks.

**Placeholder check:** No `TBD`, `TODO`, `implement later`, or "similar to" references in the plan. Each step contains the exact code or command.

**Type consistency check:**
- `PersonDetection.msg` field names (`detected`, `bbox_cx`, `bbox_cy`, `bbox_w`, `bbox_h`, `distance`, `confidence`, `stamp`) match across Task 5 (msg), Task 7 (publisher), Task 9 (consumer), Task 12 (test).
- `ControlParams` fields (`target_distance`, `close_threshold`, `far_threshold`, `max_linear_speed`, `max_angular_speed`, `k_linear`, `k_angular`) match across Task 8 (dataclass), Task 9 (Approach action), Task 10 (BT node param read), Task 12 (test fixture).
- `KEY_DETECTION`, `KEY_DISTANCE`, `KEY_LAST_SEEN_TIME` defined once in `helpers.py` (Task 9), imported everywhere else.
- `compute_approach_twist` signature `(distance, bbox_cx, params) -> (v, w)` consistent in Task 8 (def) and Task 9 (call).
- `build_tree` signature `(twist_pub, params, person_lost_timeout, search_yaw_rate)` consistent in Task 10 (def) and Task 12 (test).

No inconsistencies found.

**Scope check:** Single implementation plan. All tasks produce code that integrates into a single deliverable. No sub-decomposition needed.
