# How It Works — In Simple Words

A plain-English walkthrough of the TurtleBot3 human-follower case study. No prior robotics, AI, or ROS knowledge assumed. Where a technical term is unavoidable, it's defined the first time it appears.

---

## TL;DR

We taught a small robot to **find a person with a camera, then drive over and stop at a friendly distance**.

It runs in a virtual world (no real robot needed). The "seeing" part uses an AI model called **YOLOv8**. The "deciding" part uses a **behavior tree** — a small flowchart of common-sense rules. The two parts talk to each other through **ROS 2**, the standard "wiring" used by modern robots.

You watch the whole thing through a **dashboard** with a live camera view, status badges, and START/STOP buttons.

---

## 1. The big picture

Think of the robot as having three jobs that work together, like a tiny team:

| Role | Who does it | What they do |
|------|-------------|--------------|
| **Eyes** | YOLOv8 (AI vision model) | Looks at each camera frame and reports *"yes there's a person, here, this big, ~85% sure"* — or *"nope, nothing"*. |
| **Brain** | Behavior tree (py_trees) | Reads the eyes' report and picks one of four actions: SEARCH, APPROACH, HOLD, or STOP. |
| **Body** | TurtleBot3 + Gazebo | Drives the wheels based on what the brain decided. |

A "loop" runs **10 times per second**: see → decide → act. Repeat. That's it.

```
[CAMERA] ──image──▶ [YOLO eyes] ──"person at (x,y), dist=2m"──▶ [behavior brain] ──"drive forward"──▶ [wheels]
   ▲                                                                                                       │
   └───────────────────────────────────── world changes ───────────────────────────────────────────────────┘
```

---

## 2. The robot's body

We use a **TurtleBot3 Waffle Pi** — a small, well-known educational robot. We don't need a physical one because we run the whole thing in a simulator called **Gazebo** (think of Gazebo as a video game where physics works correctly). The simulated robot has the same parts a real one would:

- **Camera** on the front — looks straight ahead, about 62° field of view (a bit narrower than your phone's main camera).
- **LiDAR** on top — a spinning laser that measures distance in every direction, like a circular tape measure that updates many times per second.
- **Two wheels** — one motor per wheel. To turn left, spin the right wheel faster than the left, and vice-versa. To go straight, spin both equally.
- **Max speed:** 0.22 m/s — about how fast a slow walk is.

The robot is small, slow, and predictable. That makes it a good first vehicle for a case study.

---

## 3. The robot's eyes — YOLOv8 (what is ML?)

### What is machine learning, really?

**Machine learning (ML)** is a way of letting a computer *learn from examples* instead of being given explicit rules. If you show a program a million pictures of cats and a million pictures of not-cats, it eventually learns the patterns and can guess "cat or not" on a new picture it has never seen.

**YOLOv8** ("You Only Look Once, version 8") is one of these learned programs. It was trained by Ultralytics (a company) on millions of photos containing 80 everyday objects — people, cars, dogs, chairs, etc. When you give it a fresh image, it answers three questions for each thing it sees:

1. **What is it?** (e.g. "person")
2. **Where in the image?** (a rectangle around it called a *bounding box*)
3. **How sure am I?** (a confidence number from 0 to 100%)

### How we use it

Our perception node grabs a frame from the robot's camera (a 320×240 colour image), sends it to YOLOv8, and asks: "show me only the people, and only ones you're at least 50% sure about." YOLO replies with a list of rectangles. We pick the most confident one and treat it as "the person we want to follow."

### Estimating distance from a 2D image

YOLO only gives us a flat rectangle — but the robot needs to know **how far away** the person is in metres. We use two tricks together:

1. **The pinhole formula.** A person who's 1.7 m tall and *far away* makes a small rectangle in the image. The same person *close up* makes a big rectangle. The maths is:

   ```
   distance ≈ (real_height_in_metres × camera_focal_length_in_pixels) / box_height_in_pixels
   ```

   Example: a person who's 1.7 m tall, camera focal length 525 px, bbox 400 px tall → distance ≈ 2.2 m.

2. **The LiDAR check.** The LiDAR already measures distance directly with a laser. We figure out which laser beam is pointing at the person (using where in the image the box is centred) and read that beam's range. This is usually more accurate than the camera estimate, *unless* the laser is out of range — in which case we fall back to the camera estimate.

Both formulas live in `tb3_follower_perception/geometry.py`. They're tiny, ~30 lines, and we cover them with unit tests so the maths is proven correct before we wire it into the robot.

### What this looks like in practice

Every ~0.5 seconds (YOLO inference is slow inside a VM with no GPU), the perception node publishes a message like:

```
detected:    true
bbox_cx:     0.40       (person is 40% across the image — slightly left)
bbox_cy:     0.50       (centred vertically)
bbox_w:      0.22
bbox_h:      0.55
distance:    3.40       (metres)
confidence:  0.86       (86% sure)
```

That message is the **only** input the behavior tree uses. Everything else is computed from it.

---

## 4. The robot's brain — a behavior tree

### Why not just write a big `if … else if … else` block?

You could. Many simple robots do. The problem is that `if/else` chains get tangled fast — once you have more than 4–5 conditions, you can't tell at a glance what the robot will do, and small changes break things in surprising places.

A **behavior tree** is a structured version of the same idea, drawn as a tree. Each box is a node. Each node either:

- **Asks a question** (return SUCCESS or FAILURE) — these are called *conditions* or *guards*.
- **Does something** (move the wheels, change a flag) — these are called *actions*.
- **Combines its children** in a specific way — these are called *composites*. The two main composites are:
  - **Sequence:** run my children left to right. If any fails, I fail. If all succeed, I succeed.
  - **Selector** (also called *Fallback*): run my children left to right. If any succeeds, I succeed. If all fail, I fail.

The tree is "ticked" repeatedly (10 times per second in our case). Each tick, the tree walks itself from root to leaves and decides what to do *right now*.

### Our actual tree

```
Root (Selector — pick the first branch that "works")
│
├── FOLLOW  (Sequence — only runs if I can see a person)
│     ├── IsPersonDetected?      ← guard: was the last detection within 1 second?
│     ├── ReadDistance           ← guard: copy distance to the blackboard
│     └── Selector
│           ├── Sequence: IsTooClose (d < 0.8 m)  → Stop          (sit still)
│           ├── Sequence: IsTooFar  (d > 1.2 m)  → Approach       (drive toward person)
│           └── Sequence: InRange  (between)     → HoldPosition   (stop driving, keep face-tracking)
│
├── SEARCH  (Sequence — runs if no person was seen for >1 second)
│     ├── PersonLostTimer        ← guard: succeeds if last_seen_time is stale
│     └── RotateInPlace          ← action: spin slowly to look around
│
└── Idle  (always-running safety net — publishes zero velocity)
```

In English: **"Try to follow. If you can't see anyone, search. If something is broken, sit still."**

The thresholds (0.8 m, 1.2 m, 1.0 m target) give us a small *dead-band* in the middle so the robot doesn't oscillate forward and backward when the person stands still around 1 m away. This is the same trick a thermostat uses: don't run the heater for tiny temperature changes — only when you go past a wider band.

### Why is this better than `if/else`?

Two reasons:

1. **It's a picture.** Anyone can look at the tree and know what will happen. Want to change the robot's behaviour? Move a box, add a node. The structure is the documentation.
2. **It's reactive.** Every tick, the Selector re-evaluates from the left. The moment a person is seen again, the robot snaps out of SEARCH and back into FOLLOW — no "state machine got stuck" bugs.

---

## 5. The control law — how "drive toward person" actually drives

When the brain picks the `Approach` action, it has to set two numbers on the wheels:

- **linear.x** — forward speed in m/s (positive = forward, negative = backward; we never reverse here)
- **angular.z** — turning speed in radians per second (positive = left turn, negative = right turn)

We use a **proportional controller**. Simple idea: *the further the error, the harder we react.* Two errors matter:

| Error | Formula | What it does |
|------|---------|--------------|
| Distance error | `distance − target_distance` (= dist − 1.0 m) | If positive (person is far), drive forward. If 0 (right at target), don't move. |
| Steering error | `−(bbox_cx − 0.5)` | If the box is left of centre, turn left. If right, turn right. |

We multiply each by a *gain* (`k_linear = 0.4`, `k_angular = 1.5`) and clamp the result so we never exceed the robot's max speed:

```
v = clamp(0.4 × (distance − 1.0), 0, 0.22 m/s)
w = clamp(1.5 × (0.5 − bbox_cx), −1.0, 1.0 rad/s)
```

That's literally the entire driving logic for "approach." Tiny, predictable, and tested in pure Python without needing the simulator (see `tb3_follower_behavior/control.py` and `test_control.py`).

---

## 6. How the parts talk to each other — ROS 2 in plain words

A robot is not one program — it's a bunch of small programs (called **nodes**) that need to share information without getting tangled. **ROS 2** is the postal service that connects them.

- A node can *publish* messages to a **topic** (a named channel, like a Slack channel for robots).
- Other nodes can *subscribe* to that topic and react when new messages arrive.
- Nobody needs to know who's listening or where they are. Senders just shout into the topic; receivers pick up whatever's relevant.

### Our topics

| Topic | What's on it | Who publishes | Who listens |
|-------|--------------|---------------|-------------|
| `/camera/image_raw` | RGB image (from the simulated camera) | Gazebo | Perception node |
| `/camera/camera_info` | Camera calibration (focal length, etc.) | Gazebo | Perception node |
| `/scan` | LiDAR scan (360 distance readings) | Gazebo | Perception node |
| `/person/detection` | Our custom message: bbox + distance + confidence | Perception node | BT node, Dashboard |
| `/cmd_vel` | Twist message: linear.x, angular.z | BT node | Gazebo (drives wheels), Dashboard |

The pattern is *publish/subscribe with no central coordinator*. Add a third node that wants to log detections? It just subscribes to `/person/detection` — no changes needed anywhere else. That's why ROS 2 is the standard plumbing in modern robotics.

---

## 7. The simulated world (Gazebo)

**Gazebo** is a physics-accurate 3D simulator. We give it a description of a world and the robot, and it does the rest (gravity, collisions, friction, sensor data, the works).

Our world (`tb3_follower_bringup/worlds/follow_world.world`) is a 20×20 m empty room with four walls and **three walking human actors**. Gazebo ships a built-in `walk.dae` animation — a person figure that walks a loop you define. One actor walks a square pattern in front of where the robot spawns (so it's detected right away), and two more pace rectangles on the west and east sides of the room.

The actors don't publish anything to ROS — they're pure visual + physics entities. YOLO sees them through the robot's camera the same way it would see real people. When several people are in frame at once, the detector estimates a distance for each and publishes only the **nearest** one — that's who the robot follows.

---

## 8. The dashboard

Reading log files isn't fun. So we built a **PyQt5 dashboard** — a normal desktop window with:

- **Live camera feed** with the YOLO bbox drawn on top (so you see what the robot sees).
- **State badge** in the corner: SEARCHING / APPROACHING / HOLDING / STOP / STOPPED — colour-coded.
- **Stat cards** for: detected/not, confidence, distance, bbox centre, current linear.x and angular.z, detection rate, command rate.
- **Buttons:** START DEMO, STOP DEMO, RESTART, CLOSE GAZEBO GUI.
- **Activity log** at the bottom.

Under the hood, the dashboard is itself a ROS 2 node. It subscribes to the same topics anything else would (`/camera/image_raw`, `/person/detection`, `/cmd_vel`). The Qt UI runs in the main thread; rclpy spins in a background thread; the two communicate via Qt signals so we never touch a widget from a non-Qt thread.

Code in `tb3_follower_gui/dashboard.py` — ~350 lines, one file.

---

## 9. What happens in one tick (the 10-Hz cycle)

To make this concrete, here's what happens in **one 100-millisecond slice**:

1. **t = 0 ms** — Gazebo simulates 100 ms of physics. The actor walks a few centimetres. The robot's wheels move based on the last `cmd_vel`. The camera renders a fresh frame, the LiDAR a fresh scan.
2. **t ≈ 5 ms** — Gazebo publishes the new image to `/camera/image_raw` and the new scan to `/scan`.
3. **t ≈ 10 ms** — Perception node receives the image. Sometimes it skips this frame (if YOLO is still processing the previous one) — otherwise it runs inference (~150–200 ms in our VM). Once done, it computes the bbox, fuses the visual+LiDAR distance estimate, and publishes a `PersonDetection` message to `/person/detection`.
4. **t ≈ 0–100 ms** (any time during the tick) — BT node receives the latest detection and stores it on its **blackboard** (an in-memory key→value store the tree reads from).
5. **t = 100 ms** — BT timer fires. The tree is ticked once: it walks from root to leaves, decides on an action, and publishes the resulting Twist to `/cmd_vel`.
6. **t = 100 ms** — Gazebo's diff-drive plugin receives the Twist and adjusts wheel velocities. Back to step 1.

Because YOLO is slow (CPU only in the VM), the detection rate is ~2 Hz, not 10. But the BT still ticks at 10 Hz — between detections it just acts on the last-known person position, and after 1 second of silence it gives up and switches to SEARCH.

---

## 10. Why we made the choices we made

A few decisions deserve to be explained:

- **Why YOLOv8 and not a custom-trained model?** Because we don't need to detect anything special — just "person," which is one of YOLO's 80 built-in classes. Training is expensive; using a pre-trained model is free and works on day one.
- **Why YOLOv8n (the "nano" variant)?** It's the smallest and fastest. Bigger variants (s, m, l, x) are more accurate but slower. With CPU-only inference in a VM, anything bigger than `n` ran at less than 1 frame per second.
- **Why a behavior tree and not a state machine or finite-state machine?** Both work for problems this size. Behavior trees are easier to extend (just add a node) and re-evaluate continuously (so the robot reacts instantly to changes), at the cost of slightly more bookkeeping.
- **Why ROS 2 Humble (not 1 or the newest)?** Humble is the current long-term support release. ROS 1 is end-of-life. The newest ROS 2 releases are still settling.
- **Why a VM (VirtualBox) instead of native or Docker?** Because the host is Windows, ROS 2 + Gazebo work best on Linux, and a VM is the simplest one-time setup that lets a Windows user run everything end-to-end without dual-booting.
- **Why split perception and BT into two nodes instead of one?** Two reasons: (a) clearer responsibilities and easier testing, and (b) if we ever swap YOLO for a different vision algorithm, the BT doesn't change.

---

## 11. Known limitations (honest list)

- **Slow YOLO in the VM.** ~150–200 ms per inference instead of ~20 ms with a GPU. The robot still works, but the detection rate is ~2 Hz instead of 10 Hz. Between detections the BT falls back to its last-seen-person memory; if the person moves fast the robot can lose them briefly.
- **No multi-person tracking.** If two people are in view, we follow whichever YOLO scored highest. We don't try to keep "the same person" across frames.
- **No path planning.** The robot drives in a straight line toward the person. If there were an obstacle in the way, it would bump into it. The room is empty for a reason.
- **No localisation or mapping.** The robot doesn't know where it is in the world. It only reacts to what it sees right now.
- **Single-screen UI.** The dashboard subscribes to topics but doesn't yet show LiDAR overlay, tree state, or a timeline.

These are all things that could be added — they're scoped out, not impossible.

---

## 12. How the whole thing starts up (the order of operations)

When you run `bash /mnt/host_project/scripts/05-run-demo.sh` inside the VM:

1. A `tmux` session is created with three panes.
2. **Pane 0:** runs `ros2 launch tb3_follower_bringup sim.launch.py` which boots Gazebo, loads our world, starts the `robot_state_publisher` (so the robot's body description is available to anything that needs it), and spawns the TurtleBot3.
3. **Pane 1:** after an 8-second delay (so Gazebo has time to come up), runs `ros2 launch tb3_follower_bringup perception.launch.py` which starts the perception node with its YAML parameters.
4. **Pane 2:** after a 10-second delay, runs `ros2 launch tb3_follower_bringup behavior.launch.py` which starts the BT node with its YAML parameters.

If you also run the dashboard (`ros2 run tb3_follower_gui dashboard`), it subscribes to the same topics and starts showing live data within seconds. None of these processes know about each other — they just publish and subscribe to the same topics, and ROS 2 wires them together.

---

## 13. Where everything lives in the repo

```
d:/ros case study/
├── README.md                                    ← project overview
├── demo.mp4                                     ← 60-sec recording of the running demo
├── Object_Following_BT_ML.pptx                  ← 10-slide presentation
│
├── docs/
│   ├── HOW_IT_WORKS.md                          ← THIS DOCUMENT
│   ├── architecture.md                          ← technical version (diagrams + topic table)
│   ├── setup/
│   │   ├── 01-create-vm.md                      ← how to make the Ubuntu VM
│   │   ├── 02-install-ros.md                    ← how to install ROS + YOLO + build
│   │   └── 03-run-demo.md                       ← how to actually run the demo
│   └── superpowers/                             ← design spec + implementation plan
│
├── scripts/                                     ← shell scripts run inside the VM
│   ├── 00-bootstrap-vm.sh                       ← passwordless sudo + symlink setup
│   ├── 02-install-ros.sh                        ← ROS 2 Humble + TurtleBot3 + Gazebo
│   ├── 03-install-yolo.sh                       ← Ultralytics + py_trees + model download
│   ├── 04-build-workspace.sh                    ← colcon build
│   └── 05-run-demo.sh                           ← tmux launcher for the demo
│
└── ros2_ws/
    └── src/
        ├── tb3_follower_msgs/                   ← custom PersonDetection.msg
        ├── tb3_follower_perception/             ← YOLO detector node
        │   ├── tb3_follower_perception/
        │   │   ├── geometry.py                  ← pure-math helpers (visual_distance, fuse_distance)
        │   │   └── person_detector_node.py      ← the ROS node
        │   └── test/test_geometry.py            ← unit tests (run anywhere)
        │
        ├── tb3_follower_behavior/               ← py_trees behavior tree node
        │   ├── tb3_follower_behavior/
        │   │   ├── control.py                   ← pure controller (compute_approach_twist)
        │   │   ├── tree.py                      ← assembles the tree
        │   │   ├── follower_bt_node.py          ← the ROS node
        │   │   └── behaviours/
        │   │       ├── guards.py                ← IsPersonDetected, IsTooClose, IsTooFar, …
        │   │       ├── actions.py               ← Approach, Stop, HoldPosition, …
        │   │       └── helpers.py               ← shared blackboard keys + TwistPublisher
        │   └── test/
        │       ├── test_control.py              ← unit tests for the controller
        │       └── test_tree_ticks.py           ← integration tests for the tree
        │
        ├── tb3_follower_bringup/                ← launch + Gazebo world + RViz config
        │   ├── launch/                          ← 4 launch files
        │   ├── worlds/follow_world.world        ← the simulated room + walking actor
        │   └── rviz/follower.rviz               ← pre-configured RViz visualisation
        │
        └── tb3_follower_gui/                    ← PyQt5 dashboard
            └── tb3_follower_gui/dashboard.py
```

Each package has its own `package.xml` and `setup.py` (or `CMakeLists.txt` for the C-style ones). `colcon build` walks them in dependency order.

---

## 14. The pure-math discipline

A subtle but important design choice: **anything that's just maths lives in its own file with no ROS imports.** Two examples:

- `tb3_follower_perception/geometry.py` — the visual-distance formula and the bbox→LiDAR-angle mapping. Imports only `math`.
- `tb3_follower_behavior/control.py` — the proportional controller. Imports only `dataclasses`.

This means we can run their unit tests on any computer with Python and pytest — no ROS install needed. Catching maths errors at this level (with `pytest` on a laptop, in seconds) is *much* cheaper than catching them after the robot drives into a wall.

The ROS nodes are then thin wrappers around the pure functions. The node reads parameters, subscribes to topics, calls into the pure code, publishes results. That's it.

---

## 15. Quick glossary

A one-line definition for every jargon term used above.

| Term | Plain-English meaning |
|------|----------------------|
| **AI** / **ML** | A program that learned from examples instead of being programmed with rules. |
| **ament_python** / **ament_cmake** | The two ways ROS 2 packages can be built — Python-style or C++-style. |
| **Behavior tree** | A flowchart of decisions, drawn as a tree, ticked many times per second. |
| **Blackboard** | A shared key→value store that BT nodes read from and write to. |
| **Bounding box / bbox** | The rectangle YOLO draws around an object it found. |
| **`cmd_vel`** | A ROS topic carrying Twist messages (linear + angular velocity). |
| **`colcon`** | The ROS 2 build tool. Compiles all packages in dependency order. |
| **Composite** | A behavior-tree node that combines its children (e.g. Sequence, Selector). |
| **Confidence** | YOLO's "how sure am I?" score from 0 to 100%. |
| **Diff-drive** | A robot with two independently-driven wheels (most small robots are this). |
| **Gazebo** | A physics-accurate 3D simulator for robots. |
| **LiDAR** | A spinning laser distance sensor. Gives a circle of distance readings. |
| **Node** | One small program in a ROS system. |
| **`py_trees`** | A Python library that runs behavior trees. |
| **`rclpy`** | "ROS Client Library for Python" — the Python binding for ROS 2. |
| **ROS 2** | An open-source framework for connecting parts of a robot via topics. |
| **Selector** | A behavior-tree composite that runs children until one succeeds. |
| **Sequence** | A behavior-tree composite that runs children until one fails. |
| **`tmux`** | A terminal multiplexer — split one terminal into multiple panes. |
| **Topic** | A named channel for ROS messages (publish/subscribe). |
| **Twist** | A ROS message type holding a linear and an angular velocity. |
| **VirtualBox** | A program that runs an operating system inside another. |
| **YOLO** / **YOLOv8** | A fast, pre-trained AI model that finds objects in images. |

---

## 16. Where to go next

If you want to:

- **Run the demo yourself** → `docs/setup/01-create-vm.md` → `02-install-ros.md` → `03-run-demo.md`.
- **Read the technical architecture** → `docs/architecture.md`.
- **See the design decisions in detail** → `docs/superpowers/specs/2026-06-01-tb3-yolo-bt-follower-design.md`.
- **Watch it in action** → open `demo.mp4` directly, or open `Object_Following_BT_ML.pptx` and click slide 9.

If you want to tinker:

- **Make the robot follow at a different distance** → change `target_distance` in `ros2_ws/src/tb3_follower_behavior/config/follower_params.yaml`.
- **Make YOLO more/less sensitive** → change `conf_threshold` in `ros2_ws/src/tb3_follower_perception/config/detector_params.yaml`.
- **Add a new BT behaviour (e.g. wave when person is close)** → add a class in `behaviours/actions.py`, wire it into `tree.py`.

That's the whole thing. The robot has eyes, a brain, and wheels. We gave it a sensible flowchart of what to do, and we wired the parts together through ROS 2. Everything else is detail.
