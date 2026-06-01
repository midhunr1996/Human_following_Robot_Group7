# TurtleBot3 Human Follower — YOLO Perception + Behavior Tree

**Date:** 2026-06-01
**Status:** Approved (design phase)
**Author:** brainstorming session

---

## 1. Goal

Build a ROS 2 case study in which a simulated TurtleBot3 in Gazebo detects a walking human using a YOLOv8 model, then follows that human while maintaining a safe distance, all controlled by a Behavior Tree. The full stack runs inside an Ubuntu 22.04 VirtualBox VM hosted on a Windows machine.

**Demo success criteria:**

- Robot rotates in place when no person is in view.
- When the walking actor enters the camera frame, YOLO detects it and the robot drives toward it.
- Robot stops at ~1 m and holds position while keeping the person centered.
- When the person walks out of view, the robot resumes searching within ~1 s.
- All transitions are driven by an inspectable Behavior Tree, not hard-coded if/else logic.

**Non-goals:**

- Multi-person tracking / ID persistence (single highest-confidence detection only).
- GPU acceleration (CPU inference only — VirtualBox cannot pass GPU through reliably).
- SLAM, mapping, or navigation stack integration.
- Real hardware deployment — sim only.

---

## 2. Stack

| Layer            | Choice                              | Why                                                       |
|------------------|-------------------------------------|-----------------------------------------------------------|
| Host OS          | Windows 11 (existing)               | User's machine                                            |
| Hypervisor       | VirtualBox (already installed)      | User constraint                                           |
| Guest OS         | Ubuntu 22.04.4 LTS Desktop          | Required for ROS 2 Humble                                 |
| ROS distro       | ROS 2 Humble                        | Current LTS, best TurtleBot3 + py_trees_ros support       |
| Simulator        | Gazebo Classic 11                   | Ships with Humble, well-tested with TurtleBot3            |
| Robot model      | TurtleBot3 Waffle Pi                | Has a camera (Burger does not) and a 360° LiDAR           |
| Perception model | YOLOv8n via Ultralytics             | Smallest model, CPU-feasible inside a VM                  |
| Behavior tree    | py_trees + py_trees_ros2            | Pure Python — same language as YOLO node, simplest stack  |
| Languages        | Python 3.10 (only)                  | One language across all nodes                             |

---

## 3. System Architecture

```
┌────────────────────────────────────────────────────────────┐
│  Ubuntu 22.04 VM (VirtualBox, stored on D: drive)          │
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

### Topics

| Topic                  | Type                              | Pub                | Sub              |
|------------------------|-----------------------------------|--------------------|------------------|
| `/camera/image_raw`    | sensor_msgs/Image                 | Gazebo (TB3 cam)   | person_detector  |
| `/camera/camera_info`  | sensor_msgs/CameraInfo            | Gazebo             | person_detector  |
| `/scan`                | sensor_msgs/LaserScan             | Gazebo (TB3 LiDAR) | follower_bt      |
| `/person/detection`    | tb3_follower_msgs/PersonDetection | person_detector    | follower_bt      |
| `/cmd_vel`             | geometry_msgs/Twist               | follower_bt        | Gazebo (TB3 ctrl)|

### Custom message `tb3_follower_msgs/PersonDetection`

```
bool detected
float32 bbox_cx          # normalized [0,1] — center x
float32 bbox_cy          # normalized [0,1] — center y
float32 bbox_w           # normalized [0,1] — width
float32 bbox_h           # normalized [0,1] — height
float32 distance         # meters, -1 if unknown
float32 confidence       # YOLO conf [0,1]
builtin_interfaces/Time stamp
```

---

## 4. Perception Node — `person_detector_node.py`

**Pipeline per frame (capped 10 Hz):**

1. `cv_bridge` converts `sensor_msgs/Image` → numpy BGR.
2. `YOLO('yolov8n.pt')` inference with `classes=[0]` (COCO person), `conf=0.5`, `imgsz=320`.
3. Pick the highest-confidence detection (single-target follow).
4. Compute normalized bbox `(cx, cy, w, h)`.
5. **Distance estimate** (fused):
   - **Visual:** `dist_visual = (1.7 * focal_px) / bbox_h_px` — focal length read from `camera_info` once at startup.
   - **LiDAR:** map `bbox_cx` → scan angle, sample range there.
   - Use LiDAR reading if `< scan.range_max`, else fall back to visual.
6. Publish `PersonDetection`. If no detection: publish with `detected=False`.

**Throttling:** if inference latency > 100 ms, drop the next frame rather than queue.

**Params (YAML):**
- `model_path` (default: `~/models/yolov8n.pt`)
- `conf_threshold` (default: `0.5`)
- `imgsz` (default: `320`)
- `max_rate_hz` (default: `10`)
- `person_height_m` (default: `1.7`)

---

## 5. Behavior Tree — `follower_bt_node.py`

**Tick rate:** 10 Hz.

**Tree structure:**

```
Root (Selector)
├── FOLLOW (Sequence)              ← runs if person currently visible
│   ├── IsPersonDetected           ← guard: detection within last 1s
│   ├── ReadDistance               ← copies detection.distance → blackboard.distance
│   └── Selector
│       ├── Sequence: IsTooClose (d < 0.8m) → Stop
│       ├── Sequence: IsTooFar   (d > 1.2m) → Approach
│       └── Sequence: InRange              → HoldPosition (face-track only)
├── SEARCH (Sequence)              ← runs if no person for > 1s
│   ├── PersonLostTimer
│   └── RotateInPlace
└── IDLE (always-running fallback) ← safety net, publishes zero Twist
```

**Blackboard keys:** `detection`, `distance`, `last_seen_time`.

**Control law (`Approach`):**
- `linear.x = clamp(k_lin * (distance - target_distance), 0, max_speed)`
- `angular.z = clamp(-k_ang * (bbox_cx - 0.5), -max_yaw, max_yaw)`

**Params (YAML):**
- `target_distance` (1.0 m)
- `close_threshold` (0.8 m)
- `far_threshold` (1.2 m)
- `max_linear_speed` (0.22 m/s — Waffle Pi cap)
- `max_angular_speed` (1.0 rad/s)
- `k_linear` (0.4), `k_angular` (1.5)
- `person_lost_timeout` (1.0 s)
- `search_yaw_rate` (0.3 rad/s)

---

## 6. Simulation World

`tb3_follower_bringup/worlds/follow_world.world`:

- Empty room (10 × 10 m, walls).
- Ground plane + sun light.
- One Gazebo `<actor>` walking a fixed waypoint loop (4 corners of a 4×4 m square) at ~0.5 m/s, using the default walking animation.
- TurtleBot3 Waffle Pi spawned at origin facing +x.

The walking actor publishes no ROS topics — it's a pure visual entity that YOLO detects through the robot's camera.

---

## 7. Repository / Workspace Layout

```
d:/ros case study/                            ← git repo root (Windows host)
├── README.md
├── docs/
│   ├── superpowers/specs/                    ← this doc
│   ├── setup/
│   │   ├── 01-create-vm.md
│   │   ├── 02-install-ros.md
│   │   └── 03-run-demo.md
│   └── architecture.md
├── scripts/
│   ├── 01-host-prep.ps1                      ← prints VM checklist on Windows
│   ├── 02-install-ros.sh                     ← VM: ROS2 Humble + TB3 + Gazebo
│   ├── 03-install-yolo.sh                    ← VM: Ultralytics + yolov8n.pt
│   ├── 04-build-workspace.sh                 ← VM: symlink + colcon build
│   └── 05-run-demo.sh                        ← VM: tmux launcher
└── ros2_ws/
    └── src/
        ├── tb3_follower_msgs/                ← custom PersonDetection.msg
        ├── tb3_follower_perception/          ← YOLO detector node
        ├── tb3_follower_behavior/            ← py_trees BT node
        └── tb3_follower_bringup/             ← launch + worlds + params
```

(Full file-level breakdown is in the implementation plan, not here.)

**Shared folder:** the host directory `d:/ros case study` mounts inside the VM at `/mnt/host_project`. The build script symlinks `/mnt/host_project/ros2_ws` → `~/ros2_ws` in the VM so `colcon build` runs natively while sources live on the Windows side.

---

## 8. VirtualBox VM Specifications

| Setting          | Value                                          |
|------------------|------------------------------------------------|
| Name             | `ros2-tb3-follower`                            |
| **Storage path** | **`D:\VirtualBox VMs\ros2-tb3-follower\`** (D: drive — user constraint) |
| Type             | Linux / Ubuntu (64-bit)                        |
| RAM              | 8 GB (4 GB minimum)                            |
| CPUs             | 4 cores (2 minimum)                            |
| Video memory     | 128 MB, enable 3D acceleration                 |
| Disk             | 40 GB dynamic VDI                              |
| Network          | NAT (default)                                  |
| Shared folder    | `d:/ros case study` → `/mnt/host_project`, auto-mount, R/W |
| Guest OS image   | Ubuntu 22.04.4 Desktop LTS                     |

**Env vars** (set in `~/.bashrc` by install script):

```bash
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
export LIBGL_ALWAYS_SOFTWARE=1    # fallback if 3D accel is unstable
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

## 9. Setup & Run Flow

1. **Host (Windows):** read `docs/setup/01-create-vm.md`, create VM in VirtualBox GUI per Section 8, install Ubuntu 22.04, install Guest Additions, mount shared folder.
2. **VM:** `bash /mnt/host_project/scripts/02-install-ros.sh` (~10–15 min: apt installs ROS 2 Humble + TurtleBot3 + Gazebo).
3. **VM:** `bash /mnt/host_project/scripts/03-install-yolo.sh` (pip installs ultralytics, downloads `yolov8n.pt` to `~/models/`).
4. **VM:** `bash /mnt/host_project/scripts/04-build-workspace.sh` (symlinks workspace, runs `colcon build --symlink-install`).
5. **VM:** `bash /mnt/host_project/scripts/05-run-demo.sh` — tmux with three panes (Gazebo, detector, BT).

All install scripts are idempotent — safe to re-run after partial failures.

---

## 10. Testing Strategy

**Unit tests** (Python `unittest`, no ROS runtime needed):

- `test_distance_estimate.py` — visual focal-length formula, bbox-to-LiDAR angle mapping.
- `test_tree_ticks.py` — instantiate the BT with a mock publisher, feed synthetic `PersonDetection` messages, assert correct `Twist` output for each branch:
  - person close → zero Twist
  - person far → forward + steering toward bbox_cx
  - no detection for 2 s → rotate in place
  - detection returns → resume FOLLOW within one tick

**Manual sim test** (the demo, checklisted in `docs/setup/03-run-demo.md`):

- Person walks → robot follows.
- Person stops near robot → robot holds at ~1 m.
- Person walks out of frame → robot rotates within 1 s.
- Person re-enters frame → robot re-engages.

No CI. Local testing only — this is a case study, not a product.

---

## 11. Risks & Mitigations

| Risk                                        | Mitigation                                                      |
|---------------------------------------------|-----------------------------------------------------------------|
| YOLO too slow on VM CPU                     | `yolov8n` + `imgsz=320` + 10 Hz cap + frame-drop on overrun     |
| Gazebo 3D accel unstable in VirtualBox      | `LIBGL_ALWAYS_SOFTWARE=1` fallback baked into install script    |
| Robot oscillates near target distance       | Dead-band: `[close_threshold, far_threshold]` = HoldPosition    |
| YOLO false negatives mid-follow             | `person_lost_timeout=1s` prevents flapping into SEARCH branch   |
| Shared-folder file-permission issues        | Build script symlinks workspace into `~` to avoid vbox FS quirks|
| User has insufficient host RAM (<12 GB)     | Setup doc states 16 GB host minimum; suggests reducing VM to 6 GB if tight |

---

## 12. Out of Scope (Explicitly)

- ROS 1 / Noetic.
- BehaviorTree.CPP / Groot2.
- Nav2 stack integration.
- Multi-person tracking, ID persistence, occlusion handling.
- GPU passthrough or CUDA inference.
- Real hardware (physical TurtleBot3).
- Production logging, metrics, or telemetry.
