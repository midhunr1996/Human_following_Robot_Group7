#!/usr/bin/env bash
# 06-follow-real.sh — One command: make the REAL TurtleBot3 Burger follow the
# nearest person.
#
# Run this ON THE VM. It will:
#   1. Start turtlebot3_bringup on the Pi (if not already running)
#   2. Start the RealSense color stream on the Pi (if not already running)
#   3. Wait for /scan + the camera color topic to reach the VM
#   4. Launch the YOLO person detector (nearest-person) + behavior-tree follower
#      on the VM, in a tmux session
#
# Usage:
#   bash /mnt/host_project/scripts/06-follow-real.sh
#   PI_PASS=yourpass bash /mnt/host_project/scripts/06-follow-real.sh   # override
#
# Stop everything:
#   tmux kill-session -t tb3-follow
#   sshpass -p turtlebot ssh ubuntu@192.168.137.105 'pkill -f robot.launch; pkill -f realsense2_camera_node'

# NOTE: no `set -u` — sourcing /opt/ros/humble/setup.bash references unset vars.
set -o pipefail

# ---------- config ----------
PI_HOST="${PI_HOST:-192.168.137.105}"
PI_USER="${PI_USER:-ubuntu}"
PI_PASS="${PI_PASS:-turtlebot}"
# Force the ROBOT's domain — do NOT inherit the VM's ROS_DOMAIN_ID (bashrc sets 7).
DOMAIN="${PI_DOMAIN:-30}"
TB3_MODEL="${TURTLEBOT3_MODEL:-waffle_pi}"
COLOR_TOPIC="/camera/camera/color/image_raw"
CINFO_TOPIC="/camera/camera/color/camera_info"
SESSION="tb3-follow"

log(){ echo -e "\033[1;34m[follow]\033[0m $*"; }
die(){ echo -e "\033[1;31m[follow]\033[0m $*" >&2; exit 1; }

command -v sshpass >/dev/null || die "sshpass missing:  sudo apt install -y sshpass"
command -v tmux    >/dev/null || die "tmux missing:     sudo apt install -y tmux"

SSH="sshpass -p $PI_PASS ssh -o StrictHostKeyChecking=no -o ConnectTimeout=8 $PI_USER@$PI_HOST"

# ---------- VM ROS env ----------
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOCALHOST_ONLY=0
export TURTLEBOT3_MODEL="$TB3_MODEL"

log "Reaching Pi at $PI_USER@$PI_HOST ..."
$SSH 'echo ok' >/dev/null 2>&1 || die "Cannot SSH to the Pi. Check the bridge/network and credentials."

# ---------- 1. robot bringup on the Pi ----------
log "Ensuring robot bringup on the Pi ..."
$SSH 'bash -s' <<REMOTE
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=0 TURTLEBOT3_MODEL=$TB3_MODEL LDS_MODEL=LDS-02
if pgrep -f robot.launch >/dev/null; then echo "  bringup already running"; else
  setsid nohup ros2 launch turtlebot3_bringup robot.launch.py </dev/null >/tmp/bringup.log 2>&1 &
  echo "  bringup started"
fi
REMOTE

# ---------- 2. RealSense color stream on the Pi ----------
log "Ensuring RealSense color stream on the Pi ..."
$SSH 'bash -s' <<REMOTE
source /opt/ros/humble/setup.bash
source ~/realsense-ros/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=0
if pgrep -f realsense2_camera_node >/dev/null; then echo "  camera already running"; else
  setsid nohup ros2 launch realsense2_camera rs_launch.py \
    enable_depth:=false pointcloud.enable:=false rgb_camera.color_profile:=424x240x15 \
    </dev/null >/tmp/realsense.log 2>&1 &
  echo "  camera started"
fi
REMOTE

# ---------- 3. wait for topics on the VM ----------
log "Waiting for /scan + camera stream on domain $DOMAIN ..."
for i in $(seq 1 40); do
  T=$(ros2 topic list 2>/dev/null)
  if echo "$T" | grep -q "^/scan$" && echo "$T" | grep -q "$COLOR_TOPIC"; then
    log "topics visible."; break
  fi
  sleep 2
done

HZ=$(timeout 8 ros2 topic hz "$COLOR_TOPIC" 2>/dev/null | grep -oE 'average rate: [0-9.]+' | head -1)
if [ -z "$HZ" ]; then
  log "WARNING: no camera frames on $COLOR_TOPIC yet."
  log "         The RealSense likely isn't streaming (Pi USB under-voltage — use a powered USB-3 hub)."
  log "         Starting the follower anyway; it will search until frames arrive."
else
  log "camera streaming: $HZ Hz"
fi

# ---------- 3.5 video transport ----------
# Default: DIRECT raw over WiFi (reliable; ~8 Hz at 424x240). Opt into compressed
# with FOLLOWER_COMPRESSED=yes for higher frame rate — it decodes the Pi's
# /compressed stream locally on the VM into /camera/image_raw (needs
# ros-humble-compressed-image-transport on both the Pi and the VM).
USE_COMPRESSED=no
IMG_TOPIC="$COLOR_TOPIC"
if [ "${FOLLOWER_COMPRESSED:-no}" = "yes" ]; then
  if ! ros2 pkg prefix compressed_image_transport >/dev/null 2>&1; then
    log "Installing compressed-image-transport on the VM ..."
    sudo apt-get install -y ros-humble-compressed-image-transport >/dev/null 2>&1
    source /opt/ros/humble/setup.bash
  fi
  if ros2 pkg prefix compressed_image_transport >/dev/null 2>&1; then
    USE_COMPRESSED=yes; IMG_TOPIC="/camera/image_raw"
    log "Compressed video transport ON (local decode -> $IMG_TOPIC)."
  else
    log "WARN: compressed-image-transport unavailable; using direct raw."
  fi
fi

# ---------- 4. launch follower in tmux ----------
if tmux has-session -t "$SESSION" 2>/dev/null; then
  log "session '$SESSION' already exists — kill: tmux kill-session -t $SESSION"
  exit 0
fi
ENV="source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=$DOMAIN ROS_LOCALHOST_ONLY=0 TURTLEBOT3_MODEL=$TB3_MODEL"

tmux new-session -d -s "$SESSION" -n follow
# pane 0: decode the Pi's COMPRESSED color stream into a full-rate LOCAL raw topic
if [ "$USE_COMPRESSED" = yes ]; then
  tmux send-keys -t "$SESSION:follow.0" \
    "$ENV; ros2 run image_transport republish compressed raw --ros-args -r in/compressed:=$COLOR_TOPIC/compressed -r out:=$IMG_TOPIC" C-m
  PERC_DELAY=5
else
  tmux send-keys -t "$SESSION:follow.0" "$ENV; echo '[follow] compressed transport unavailable — using direct raw ($COLOR_TOPIC)'; sleep infinity" C-m
  PERC_DELAY=1
fi
# pane 1: YOLO person detector (nearest-person) on the fast local topic + Pi camera_info
tmux split-window -v -t "$SESSION:follow"
tmux send-keys -t "$SESSION:follow.1" \
  "$ENV; sleep $PERC_DELAY; ros2 run tb3_follower_perception person_detector_node --ros-args \
     --params-file ~/ros2_ws/src/tb3_follower_perception/config/detector_params.yaml \
     -p image_topic:=$IMG_TOPIC -p camera_info_topic:=$CINFO_TOPIC" C-m
# pane 2: behavior-tree follower (publishes /cmd_vel)
tmux split-window -h -t "$SESSION:follow.1"
tmux send-keys -t "$SESSION:follow.2" "$ENV; sleep $((PERC_DELAY + 6)); ros2 launch tb3_follower_bringup behavior.launch.py" C-m

log "Follower running in tmux '$SESSION' (video transport: $USE_COMPRESSED). Robot drives toward the nearest person."
log "  watch:  tmux attach -t $SESSION       (detach: Ctrl+B then D)"
log "  stop:   tmux kill-session -t $SESSION"
log "  KEEP FLOOR CLEARANCE — the real robot moves."
