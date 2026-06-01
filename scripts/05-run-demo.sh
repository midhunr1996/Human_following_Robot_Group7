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
SETUP='export DISPLAY=:0 && source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && export TURTLEBOT3_MODEL=waffle_pi && export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models'

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

# Attach only if we have a real TTY (interactive shell). When invoked through
# VBoxManage guestcontrol or any non-tty caller, attaching would fail — just
# leave the session running detached and print the attach hint.
if [[ -t 0 && -t 1 ]]; then
    tmux attach -t "$SESSION"
else
    echo "tmux session '$SESSION' running detached."
    echo "Attach from inside the VM with: tmux attach -t $SESSION"
fi
