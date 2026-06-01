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
