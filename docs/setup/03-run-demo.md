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
