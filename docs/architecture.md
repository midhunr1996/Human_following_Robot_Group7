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
