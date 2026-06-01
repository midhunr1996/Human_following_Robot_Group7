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
# /opt/ros/humble/setup.bash references unset vars; temporarily relax nounset.
set +u
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
set -u

cd "$LOCAL_WS"

# ----- rosdep deps -----
log "Resolving package dependencies with rosdep..."
rosdep install --from-paths src --ignore-src -r -y || true

# ----- colcon build -----
log "Running colcon build --symlink-install (this can take a few minutes)..."
colcon build --symlink-install

log "DONE. Open a NEW terminal and run: source ~/ros2_ws/install/setup.bash"
log "Then: bash /mnt/host_project/scripts/05-run-demo.sh"
