#!/usr/bin/env bash
# 04-build-workspace.sh — Symlink the workspace from the shared folder into $HOME
# and run colcon build. Idempotent.

set -euo pipefail

log()  { echo -e "\033[1;34m[build-ws]\033[0m $*"; }
die()  { echo -e "\033[1;31m[build-ws]\033[0m $*"; exit 1; }

HOST_SRC="/mnt/host_project/ros2_ws/src"
LOCAL_WS="$HOME/ros2_ws"

[[ -d "$HOST_SRC" ]] || die "Shared folder workspace src not found at $HOST_SRC. Is the VirtualBox shared folder mounted?"

# ----- Workspace layout -----
# colcon writes build/install/log into the workspace root, and VirtualBox
# shared folders cannot create symlinks (Windows host limitation). So we keep
# the workspace root in $HOME (real FS) and only symlink src/ from the shared
# folder. Sources live on the host; build outputs live in the VM.
if [[ -L "$LOCAL_WS" ]]; then
    log "Removing legacy whole-workspace symlink at $LOCAL_WS"
    rm "$LOCAL_WS"
fi
mkdir -p "$LOCAL_WS"
cd "$LOCAL_WS"

if [[ -L src ]]; then
    log "src symlink already exists, leaving in place."
elif [[ -e src ]]; then
    die "$LOCAL_WS/src exists and is not a symlink. Remove it manually and re-run."
else
    log "Creating src symlink: $LOCAL_WS/src -> $HOST_SRC"
    ln -s "$HOST_SRC" src
fi

# Clean up any stale build/install/log artifacts that earlier failed runs may
# have created inside the shared folder (where they don't belong).
rm -rf /mnt/host_project/ros2_ws/build \
       /mnt/host_project/ros2_ws/install \
       /mnt/host_project/ros2_ws/log 2>/dev/null || true

# ----- Source ROS env -----
# /opt/ros/humble/setup.bash references unset vars; temporarily relax nounset.
set +u
# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash
set -u

# ----- rosdep deps -----
log "Resolving package dependencies with rosdep..."
rosdep install --from-paths src --ignore-src -r -y || true

# ----- colcon build -----
log "Running colcon build --symlink-install (this can take a few minutes)..."
colcon build --symlink-install

log "DONE. Open a NEW terminal and run: source ~/ros2_ws/install/setup.bash"
log "Then: bash /mnt/host_project/scripts/05-run-demo.sh"
