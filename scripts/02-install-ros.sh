#!/usr/bin/env bash
# 02-install-ros.sh — Install ROS 2 Humble + TurtleBot3 + Gazebo on Ubuntu 22.04.
# Idempotent: safe to re-run after partial failure.

set -euo pipefail

log()  { echo -e "\033[1;34m[install-ros]\033[0m $*"; }
warn() { echo -e "\033[1;33m[install-ros]\033[0m $*"; }
die()  { echo -e "\033[1;31m[install-ros]\033[0m $*"; exit 1; }

# ----- Sanity checks -----
[[ "$(lsb_release -cs)" == "jammy" ]] || die "This script requires Ubuntu 22.04 (jammy). Got: $(lsb_release -cs)"
[[ $EUID -ne 0 ]] || die "Do NOT run as root. Run as your normal user; sudo is invoked per-step."

log "Ubuntu 22.04 confirmed. Starting install."

# ----- Locale -----
log "Setting UTF-8 locale..."
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ----- ROS 2 apt repo -----
log "Adding ROS 2 apt repository..."
sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository -y universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

ROS_REPO_LINE="deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main"
echo "$ROS_REPO_LINE" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

# ----- Install ROS 2 Humble desktop + dev tools -----
log "Installing ros-humble-desktop and dev tools (~5-10 min)..."
sudo apt install -y \
    ros-humble-desktop \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete

# ----- rosdep init -----
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    log "Initializing rosdep..."
    sudo rosdep init
fi
rosdep update

# ----- TurtleBot3 + Gazebo packages -----
log "Installing TurtleBot3 + Gazebo Classic..."
sudo apt install -y \
    ros-humble-turtlebot3 \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-simulations \
    ros-humble-turtlebot3-gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-vision-msgs

# ----- tmux for the demo runner -----
sudo apt install -y tmux

# ----- Env vars in ~/.bashrc -----
log "Configuring ~/.bashrc..."
BASHRC="$HOME/.bashrc"
MARKER="# >>> tb3-follower env >>>"
if ! grep -q "$MARKER" "$BASHRC"; then
    cat >> "$BASHRC" <<EOF

$MARKER
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/humble/setup.bash
[[ -f \$HOME/ros2_ws/install/setup.bash ]] && source \$HOME/ros2_ws/install/setup.bash
# <<< tb3-follower env <<<
EOF
    log "Added env block to ~/.bashrc."
else
    log "~/.bashrc env block already present, skipping."
fi

log "DONE. Open a NEW terminal (or 'source ~/.bashrc') and run scripts/03-install-yolo.sh next."
