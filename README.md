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
