# 01 — Create the Ubuntu 22.04 VM in VirtualBox

## What you need before starting

- VirtualBox 7.x already installed on Windows.
- At least 16 GB host RAM (VM uses 6 GB; OS uses the rest).
  > **Important — do not give the VM 8 GB on a 16 GB host.** With only ~2 GB free
  > on the host, VirtualBox 7.1.x cannot commit an 8 GB guest and the GUI frontend
  > (`VirtualBoxVM.exe`) crashes on start with *"The instruction at 0x… referenced
  > memory at 0x… The memory could not be read."* 6 GB (`6144 MB`) is stable and is
  > enough for Gazebo + YOLOv8n + ROS 2. See Troubleshooting.
- ~50 GB free on the **D: drive**.
- Ubuntu 22.04.4 LTS Desktop ISO. Download from:
  https://releases.ubuntu.com/22.04/ubuntu-22.04.4-desktop-amd64.iso

Save the ISO anywhere — e.g. `D:\ISOs\ubuntu-22.04.4-desktop-amd64.iso`.

## Step 1 — Set VirtualBox default machine folder to D:

This makes new VMs land on D: by default.

1. Open VirtualBox.
2. **File → Preferences → General**.
3. Set **Default Machine Folder** to `D:\VirtualBox VMs`.
4. Click **OK**.

## Step 2 — Create the VM

1. **Machine → New**.
2. Settings:
   - **Name:** `ros2-tb3-follower`
   - **Folder:** `D:\VirtualBox VMs` (should already be the default after Step 1)
   - **ISO Image:** the Ubuntu 22.04.4 ISO you downloaded
   - **Type:** Linux
   - **Version:** Ubuntu (64-bit)
   - **Skip Unattended Installation:** **CHECK THIS BOX** (we want manual install for shared-folder setup)
3. **Hardware:**
   - **Base Memory:** 6144 MB (6 GB) — see the host-RAM note above; do **not** use 8192 on a 16 GB host
   - **Processors:** 4 CPUs
4. **Hard Disk:**
   - **Create a Virtual Hard Disk Now**
   - **Size:** 40 GB
   - **Format:** VDI, dynamically allocated
5. Click **Finish**.

## Step 3 — Tune VM settings before first boot

Select `ros2-tb3-follower` → **Settings**:

- **System → Processor:** confirm 4 CPUs, **enable** `Enable PAE/NX` and `Enable Nested VT-x/AMD-V` if greyed-on.
- **Display → Screen:**
  - **Video Memory:** 128 MB
  - **Graphics Controller:** VMSVGA
  - **Enable 3D Acceleration:** ON (best effort; Gazebo will fall back to software rendering if this is unreliable)
- **Network → Adapter 1:** NAT (default, no change needed).
- **Shared Folders → Add:**
  - **Folder Path:** `D:\ros case study`
  - **Folder Name:** `host_project`
  - **Auto-mount:** ON
  - **Make Permanent:** ON
  - **Read-only:** OFF
- **General → Advanced:** Shared Clipboard → Bidirectional, Drag'n'Drop → Bidirectional.

Click **OK**.

## Step 4 — Install Ubuntu

1. **Start** the VM.
2. Boot into Ubuntu installer → choose:
   - Language: English
   - Keyboard: your layout
   - **Normal installation** (NOT minimal)
   - **Download updates while installing:** ON
   - **Install third-party software:** ON
   - **Erase disk and install Ubuntu** (this only erases the VM's virtual disk, not your real one)
   - User: pick whatever username/password you want; remember them
   - **Log in automatically:** ON (convenience for a dev VM)
3. Wait for install (~10 min). Reboot when prompted. Press Enter to eject the ISO.

## Step 5 — Install VirtualBox Guest Additions

Inside the running Ubuntu VM:

1. Open a terminal (Ctrl+Alt+T).
2. Run:
   ```bash
   sudo apt update
   sudo apt install -y build-essential dkms linux-headers-$(uname -r)
   ```
3. From the VirtualBox menu bar (top of the VM window): **Devices → Insert Guest Additions CD image**.
4. When the file manager opens, right-click in the CD window → **Open in Terminal**, then:
   ```bash
   sudo ./VBoxLinuxAdditions.run
   ```
5. After it finishes, add your user to the `vboxsf` group so you can read the shared folder:
   ```bash
   sudo usermod -aG vboxsf $USER
   ```
6. Reboot the VM:
   ```bash
   sudo reboot
   ```

## Step 6 — Confirm the shared folder is mounted

After reboot, open a terminal and run:

```bash
ls /media/sf_host_project
```

You should see `README.md`, `docs/`, `scripts/`, etc. — the contents of `D:\ros case study`.

The default mount path is `/media/sf_host_project`. We'll create a symlink to make it `/mnt/host_project` for shorter paths:

```bash
sudo ln -s /media/sf_host_project /mnt/host_project
ls /mnt/host_project
```

You should see the same contents. **You are done with VM setup.** Proceed to `02-install-ros.md`.

## Troubleshooting

- **Shared folder shows "Permission denied":** you forgot Step 5.5 (`usermod -aG vboxsf`). Re-run it and reboot.
- **VM boots to a black screen after install:** disable 3D acceleration in Settings → Display, then restart.
- **`ls /media/` is empty:** Guest Additions install failed. Re-do Step 5, watching for errors.
- **VM window crashes on start with `VirtualBoxVM.exe - Application Error` / "The memory could not be read":**
  the guest RAM is too large for the free host RAM to commit. Power the VM off and lower Base Memory:
  ```
  VBoxManage modifyvm "ros2-tb3-follower" --memory 6144
  ```
  (Optional, also helps stability: `--accelerate3d off`.) Then start it again. If 6 GB still crashes,
  close other host apps or drop to `--memory 4096`. A **headless** start (`VBoxManage startvm
  "ros2-tb3-follower" --type headless`) never hits this crash and is a good fallback.
