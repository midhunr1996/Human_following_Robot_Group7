#!/usr/bin/env bash
# 03-install-yolo.sh — Install Ultralytics YOLOv8 + dependencies and pre-download yolov8n.pt.
# Idempotent.

set -euo pipefail

log()  { echo -e "\033[1;34m[install-yolo]\033[0m $*"; }
die()  { echo -e "\033[1;31m[install-yolo]\033[0m $*"; exit 1; }

[[ $EUID -ne 0 ]] || die "Do NOT run as root."

log "Installing Python dev tools..."
sudo apt update
sudo apt install -y python3-pip python3-venv libgl1 libglib2.0-0

log "Upgrading pip..."
python3 -m pip install --upgrade pip

log "Installing Ultralytics (CPU-only torch) and runtime deps..."
# CPU-only torch keeps the install small and avoids pulling CUDA toolchain
python3 -m pip install --user \
    --index-url https://download.pytorch.org/whl/cpu \
    torch==2.2.2 torchvision==0.17.2
python3 -m pip install --user \
    ultralytics==8.2.31 \
    opencv-python-headless==4.10.0.84 \
    numpy==1.26.4

log "Installing py_trees + py_trees_ros for ROS 2 Humble..."
sudo apt install -y \
    ros-humble-py-trees \
    ros-humble-py-trees-ros \
    ros-humble-py-trees-ros-interfaces

log "Pre-downloading yolov8n.pt to ~/models/..."
mkdir -p "$HOME/models"
if [[ ! -f "$HOME/models/yolov8n.pt" ]]; then
    python3 - <<'PY'
from ultralytics import YOLO
import shutil, os
m = YOLO("yolov8n.pt")  # downloads to current dir
shutil.move("yolov8n.pt", os.path.expanduser("~/models/yolov8n.pt"))
print("Downloaded yolov8n.pt ->", os.path.expanduser("~/models/yolov8n.pt"))
PY
else
    log "yolov8n.pt already present at ~/models/, skipping."
fi

log "Sanity check: torch + ultralytics import..."
python3 - <<'PY'
import torch, ultralytics, cv2, numpy
print("torch     :", torch.__version__)
print("ultralytics:", ultralytics.__version__)
print("cv2       :", cv2.__version__)
print("numpy     :", numpy.__version__)
PY

log "DONE. Next: scripts/04-build-workspace.sh"
