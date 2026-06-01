#!/usr/bin/env bash
# 00-bootstrap-vm.sh — one-time pre-install setup inside the Ubuntu VM.
#
# Sets up passwordless sudo for the current user and creates the
# /mnt/host_project symlink so the rest of the install scripts can run
# non-interactively via VBoxManage guestcontrol (no password prompts).
#
# Usage:
#   bash 00-bootstrap-vm.sh <sudo-password>

set -euo pipefail

PW="${1:?usage: $0 <sudo-password>}"
USER_NAME="$(id -un)"

echo "[bootstrap] configuring passwordless sudo for $USER_NAME..."
echo "$PW" | sudo -S sh -c "echo '$USER_NAME ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/99-$USER_NAME-nopasswd && chmod 440 /etc/sudoers.d/99-$USER_NAME-nopasswd"

if sudo -n true 2>/dev/null; then
    echo "[bootstrap] OK: passwordless sudo configured for $USER_NAME"
else
    echo "[bootstrap] FAILED to configure passwordless sudo" >&2
    exit 1
fi

echo "[bootstrap] creating /mnt/host_project -> /media/sf_host_project symlink..."
sudo ln -sfn /media/sf_host_project /mnt/host_project
echo "[bootstrap] OK: /mnt/host_project -> $(readlink /mnt/host_project)"

echo "[bootstrap] shared folder contents:"
ls /mnt/host_project | head -5

echo "[bootstrap] done."
