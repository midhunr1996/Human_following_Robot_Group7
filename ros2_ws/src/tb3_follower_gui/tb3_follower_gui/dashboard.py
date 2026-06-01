"""PyQt5 dashboard for the TB3 follower demo.

Subscribes to /person/detection, /cmd_vel, /camera/image_raw and displays a
live status panel + bbox-overlaid camera preview. Buttons spawn/kill the demo
processes inside the VM.

Runs as a ROS 2 node + Qt main thread. rclpy.spin lives in a QThread; messages
are marshaled into the Qt main thread via signals.
"""
from __future__ import annotations

import os
# opencv-python bundles its own Qt5 plugins that conflict with system PyQt5.
# Strip the cv2-installed plugin path BEFORE importing anything Qt-related, and
# point Qt at the system plugins directory.
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
os.environ.setdefault(
    "QT_QPA_PLATFORM_PLUGIN_PATH",
    "/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms",
)

import signal
import subprocess
import sys
import time
from collections import deque
from datetime import datetime

# Import PyQt5 BEFORE cv2 so the system Qt libs load first.
from PyQt5.QtCore import Qt, QObject, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QImage, QPixmap, QFont, QColor, QPalette
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QPushButton, QTextEdit,
    QGridLayout, QHBoxLayout, QVBoxLayout, QFrame, QSizePolicy, QGroupBox,
)

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from tb3_follower_msgs.msg import PersonDetection


DEMO_SCRIPT = "/mnt/host_project/scripts/05-run-demo.sh"
TMUX_SESSION = "tb3-follower"


# ---------------------------------------------------------------------------
# Signal hub — rclpy callbacks emit here; Qt slots receive in the main thread.
# ---------------------------------------------------------------------------

class Signals(QObject):
    detection = pyqtSignal(object)
    cmd_vel = pyqtSignal(object)
    image = pyqtSignal(object)


# ---------------------------------------------------------------------------
# ROS node — subscribes to topics, forwards via signals.
# ---------------------------------------------------------------------------

class GuiRosNode(Node):
    def __init__(self, signals: Signals):
        super().__init__("tb3_follower_gui")
        self.signals = signals
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            PersonDetection, "/person/detection", self._on_detection, 10
        )
        self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, 10
        )
        self.create_subscription(
            Image, "/camera/image_raw", self._on_image, sensor_qos
        )

    def _on_detection(self, msg: PersonDetection):
        self.signals.detection.emit(msg)

    def _on_cmd_vel(self, msg: Twist):
        self.signals.cmd_vel.emit(msg)

    def _on_image(self, msg: Image):
        self.signals.image.emit(msg)


class RosThread(QThread):
    def __init__(self, signals: Signals):
        super().__init__()
        self.signals = signals
        self.node = None

    def run(self):
        rclpy.init(args=None)
        self.node = GuiRosNode(self.signals)
        try:
            rclpy.spin(self.node)
        except Exception:
            pass
        finally:
            if self.node is not None:
                self.node.destroy_node()
            try:
                rclpy.shutdown()
            except Exception:
                pass


# ---------------------------------------------------------------------------
# UI widgets
# ---------------------------------------------------------------------------

CARD_QSS = """
QFrame#card {
    background-color: #1f2430;
    border-radius: 10px;
    border: 1px solid #2c3142;
}
QLabel#cardTitle {
    color: #99a4c4;
    font-size: 11px;
    font-weight: 600;
    letter-spacing: 1px;
}
QLabel#cardValue {
    color: #f3f6ff;
    font-size: 22px;
    font-weight: 700;
}
"""


def _card(title: str, initial: str = "—") -> tuple[QFrame, QLabel]:
    frame = QFrame()
    frame.setObjectName("card")
    frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
    v = QVBoxLayout(frame)
    v.setContentsMargins(14, 10, 14, 12)
    v.setSpacing(2)
    t = QLabel(title)
    t.setObjectName("cardTitle")
    val = QLabel(initial)
    val.setObjectName("cardValue")
    v.addWidget(t)
    v.addWidget(val)
    return frame, val


class StateBadge(QLabel):
    """A pill-shaped badge that shows current robot state."""

    def __init__(self):
        super().__init__("STOPPED")
        self.setAlignment(Qt.AlignCenter)
        self.setFont(QFont("DejaVu Sans", 16, QFont.Bold))
        self.set_state("stopped")
        self.setFixedHeight(44)
        self.setMinimumWidth(220)

    def set_state(self, state: str):
        palette = {
            "stopped":   ("#3b4256", "#cfd6e8"),
            "searching": ("#caa030", "#1f1a0a"),
            "approach":  ("#2f7d4f", "#eafff1"),
            "hold":      ("#2360b0", "#eaf2ff"),
            "stop":      ("#b8413d", "#fff0ee"),
        }
        bg, fg = palette.get(state, palette["stopped"])
        label = {
            "stopped":   "● STOPPED",
            "searching": "● SEARCHING",
            "approach":  "● APPROACHING",
            "hold":      "● HOLDING",
            "stop":      "● TOO CLOSE — STOP",
        }.get(state, "● STOPPED")
        self.setText(label)
        self.setStyleSheet(
            f"background-color: {bg}; color: {fg}; "
            f"border-radius: 22px; padding: 0 22px;"
        )


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class Dashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TurtleBot3 Human Follower — Dashboard")
        self.resize(1100, 720)

        self.bridge = CvBridge()
        self.last_detection: PersonDetection | None = None
        self.last_detection_t: float = 0.0
        self.last_cmd: Twist | None = None
        self.last_image_arr: np.ndarray | None = None

        self.det_timestamps = deque(maxlen=40)
        self.cmd_timestamps = deque(maxlen=40)

        self.demo_proc: subprocess.Popen | None = None

        self._build_ui()
        self._apply_theme()

        # Signals
        self.signals = Signals()
        self.signals.detection.connect(self._on_detection)
        self.signals.cmd_vel.connect(self._on_cmd_vel)
        self.signals.image.connect(self._on_image)

        # ROS thread
        self.ros_thread = RosThread(self.signals)
        self.ros_thread.start()

        # Periodic UI refresh (rates, freshness, state badge)
        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self._refresh_derived)
        self.refresh_timer.start(200)  # 5 Hz refresh

        self._log("Dashboard ready. Click START DEMO to launch Gazebo + perception + BT.")

    # ---------------- layout ----------------

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        # Title bar
        title = QLabel("TurtleBot3 Human Follower")
        title.setFont(QFont("DejaVu Sans", 18, QFont.Bold))
        subtitle = QLabel("YOLOv8 perception · py_trees behavior tree · ROS 2 Humble")
        subtitle.setStyleSheet("color: #8a93ad;")
        topbar = QHBoxLayout()
        title_box = QVBoxLayout()
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        topbar.addLayout(title_box)
        topbar.addStretch(1)
        self.badge = StateBadge()
        topbar.addWidget(self.badge)
        root.addLayout(topbar)

        # Main split: camera (left) | metrics (right)
        body = QHBoxLayout()
        body.setSpacing(12)
        root.addLayout(body, stretch=1)

        # Camera frame
        cam_box = QGroupBox("Live Camera + Detection")
        cam_box.setStyleSheet(
            "QGroupBox { color: #99a4c4; font-weight: 600; "
            "border: 1px solid #2c3142; border-radius: 10px; margin-top: 8px; padding: 12px; } "
            "QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 6px; }"
        )
        cam_v = QVBoxLayout(cam_box)
        self.camera_label = QLabel("waiting for /camera/image_raw …")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setMinimumSize(480, 360)
        self.camera_label.setStyleSheet(
            "background-color: #0f131c; color: #5a6480; "
            "border: 1px dashed #2c3142; border-radius: 6px;"
        )
        cam_v.addWidget(self.camera_label, stretch=1)
        body.addWidget(cam_box, stretch=3)

        # Metrics column
        metrics_col = QVBoxLayout()
        metrics_col.setSpacing(10)

        # Grid of stat cards
        grid = QGridLayout()
        grid.setSpacing(10)

        self.card_detected_frame, self.card_detected = _card("PERSON", "—")
        self.card_conf_frame, self.card_conf = _card("CONFIDENCE", "—")
        self.card_dist_frame, self.card_dist = _card("DISTANCE", "—")
        self.card_bbox_frame, self.card_bbox = _card("BBOX CENTER", "—")
        self.card_lin_frame, self.card_lin = _card("LINEAR.X (m/s)", "—")
        self.card_ang_frame, self.card_ang = _card("ANGULAR.Z (rad/s)", "—")
        self.card_drate_frame, self.card_drate = _card("DET RATE (Hz)", "—")
        self.card_crate_frame, self.card_crate = _card("CMD RATE (Hz)", "—")

        grid.addWidget(self.card_detected_frame, 0, 0)
        grid.addWidget(self.card_conf_frame,     0, 1)
        grid.addWidget(self.card_dist_frame,     1, 0)
        grid.addWidget(self.card_bbox_frame,     1, 1)
        grid.addWidget(self.card_lin_frame,      2, 0)
        grid.addWidget(self.card_ang_frame,      2, 1)
        grid.addWidget(self.card_drate_frame,    3, 0)
        grid.addWidget(self.card_crate_frame,    3, 1)

        metrics_col.addLayout(grid)
        metrics_col.addStretch(1)

        body.addLayout(metrics_col, stretch=2)

        # Controls
        ctrl = QHBoxLayout()
        ctrl.setSpacing(10)
        self.btn_start = QPushButton("▶  START DEMO")
        self.btn_stop = QPushButton("■  STOP DEMO")
        self.btn_restart = QPushButton("⟳  RESTART")
        self.btn_close_gz = QPushButton("✕  CLOSE GAZEBO GUI")
        for b, color in [
            (self.btn_start, "#2f7d4f"),
            (self.btn_stop, "#b8413d"),
            (self.btn_restart, "#3a6cb0"),
            (self.btn_close_gz, "#5a6480"),
        ]:
            b.setFixedHeight(46)
            b.setCursor(Qt.PointingHandCursor)
            b.setStyleSheet(
                f"QPushButton {{ background: {color}; color: white; "
                f"border-radius: 10px; font-weight: 700; font-size: 14px; padding: 0 20px; }} "
                f"QPushButton:hover {{ filter: brightness(1.1); }} "
                f"QPushButton:disabled {{ background: #3b4256; color: #6c768d; }}"
            )
            ctrl.addWidget(b)
        ctrl.addStretch(1)
        root.addLayout(ctrl)

        self.btn_start.clicked.connect(self._start_demo)
        self.btn_stop.clicked.connect(self._stop_demo)
        self.btn_restart.clicked.connect(self._restart_demo)
        self.btn_close_gz.clicked.connect(self._close_gz_gui)

        # Log box
        log_box = QGroupBox("Activity Log")
        log_box.setStyleSheet(
            "QGroupBox { color: #99a4c4; font-weight: 600; "
            "border: 1px solid #2c3142; border-radius: 10px; margin-top: 8px; padding: 12px; } "
            "QGroupBox::title { subcontrol-origin: margin; left: 14px; padding: 0 6px; }"
        )
        lv = QVBoxLayout(log_box)
        self.log_view = QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setFixedHeight(120)
        self.log_view.setStyleSheet(
            "QTextEdit { background: #0f131c; color: #c9d3ea; "
            "border: 1px solid #2c3142; border-radius: 6px; font-family: 'DejaVu Sans Mono'; font-size: 11px; }"
        )
        lv.addWidget(self.log_view)
        root.addWidget(log_box)

    def _apply_theme(self):
        self.setStyleSheet(CARD_QSS + """
            QMainWindow, QWidget { background-color: #11141d; color: #e3e8f5; }
        """)
        # Force dark palette so platform-style widgets don't break
        pal = QPalette()
        pal.setColor(QPalette.Window, QColor("#11141d"))
        pal.setColor(QPalette.WindowText, QColor("#e3e8f5"))
        pal.setColor(QPalette.Base, QColor("#1f2430"))
        pal.setColor(QPalette.Text, QColor("#e3e8f5"))
        self.setPalette(pal)

    # ---------------- ROS callbacks (main thread via signals) ----------------

    def _on_detection(self, msg: PersonDetection):
        self.last_detection = msg
        now = time.monotonic()
        self.last_detection_t = now
        self.det_timestamps.append(now)
        if msg.detected:
            self.card_detected.setText("✓  DETECTED")
            self.card_detected.setStyleSheet("color: #5fd687;")
            self.card_conf.setText(f"{msg.confidence * 100:.0f}%")
            self.card_dist.setText(f"{msg.distance:.2f} m")
            self.card_bbox.setText(f"({msg.bbox_cx:.2f}, {msg.bbox_cy:.2f})")
        else:
            self.card_detected.setText("✗  NO PERSON")
            self.card_detected.setStyleSheet("color: #d6915f;")
            self.card_conf.setText("—")
            self.card_dist.setText("—")
            self.card_bbox.setText("—")

    def _on_cmd_vel(self, msg: Twist):
        self.last_cmd = msg
        self.cmd_timestamps.append(time.monotonic())
        self.card_lin.setText(f"{msg.linear.x:+.3f}")
        self.card_ang.setText(f"{msg.angular.z:+.3f}")

    def _on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self._log(f"image conv error: {e}")
            return
        # Overlay bbox if we have a fresh detection
        if self.last_detection is not None and self.last_detection.detected and \
                (time.monotonic() - self.last_detection_t) < 1.0:
            h, w = frame.shape[:2]
            d = self.last_detection
            cx, cy, bw, bh = d.bbox_cx * w, d.bbox_cy * h, d.bbox_w * w, d.bbox_h * h
            x1, y1 = int(cx - bw / 2), int(cy - bh / 2)
            x2, y2 = int(cx + bw / 2), int(cy + bh / 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (60, 220, 100), 2)
            label = f"person {d.confidence:.0%}  {d.distance:.1f}m"
            cv2.rectangle(frame, (x1, y1 - 22), (x1 + 230, y1), (60, 220, 100), -1)
            cv2.putText(frame, label, (x1 + 5, y1 - 7),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (10, 30, 20), 2, cv2.LINE_AA)
        self.last_image_arr = frame
        self._render_image(frame)

    def _render_image(self, frame_bgr: np.ndarray):
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        qimg = QImage(rgb.data, w, h, ch * w, QImage.Format_RGB888)
        target = self.camera_label.size()
        pix = QPixmap.fromImage(qimg).scaled(
            target, Qt.KeepAspectRatio, Qt.SmoothTransformation
        )
        self.camera_label.setPixmap(pix)

    # ---------------- Derived metrics + state ----------------

    def _refresh_derived(self):
        now = time.monotonic()
        # Detection freshness for state badge
        age = now - self.last_detection_t if self.last_detection_t else 999.0
        det_alive = (
            self.last_detection is not None
            and self.last_detection.detected
            and age < 1.0
        )

        if not self._demo_running():
            self.badge.set_state("stopped")
        elif det_alive:
            d = self.last_detection.distance
            if d < 0.8:
                self.badge.set_state("stop")
            elif d > 1.2:
                self.badge.set_state("approach")
            else:
                self.badge.set_state("hold")
        else:
            self.badge.set_state("searching")

        # Rates over last 4-second window
        self.card_drate.setText(f"{self._rate(self.det_timestamps, now):.1f}")
        self.card_crate.setText(f"{self._rate(self.cmd_timestamps, now):.1f}")

        # If detection went stale, mark the bbox card dim
        if not det_alive and self._demo_running():
            self.card_detected.setText("✗  NO PERSON")
            self.card_detected.setStyleSheet("color: #d6915f;")
            self.card_conf.setText("—")
            self.card_dist.setText("—")
            self.card_bbox.setText("—")

    @staticmethod
    def _rate(stamps: deque, now: float, window: float = 4.0) -> float:
        recent = [t for t in stamps if now - t <= window]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / (recent[-1] - recent[0]) if recent[-1] > recent[0] else 0.0

    # ---------------- Demo control ----------------

    def _demo_running(self) -> bool:
        return subprocess.run(
            ["tmux", "has-session", "-t", TMUX_SESSION],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        ).returncode == 0

    def _start_demo(self):
        if self._demo_running():
            self._log("Demo already running.")
            return
        self._log("Starting demo (tmux: gazebo + perception + BT) …")
        env = os.environ.copy()
        env.setdefault("DISPLAY", ":0")
        try:
            subprocess.Popen(
                ["bash", DEMO_SCRIPT],
                env=env,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            self._log(f"Spawned {DEMO_SCRIPT}. Allow ~15 s for Gazebo + spawn.")
        except FileNotFoundError:
            self._log(f"ERROR: {DEMO_SCRIPT} not found. Is /mnt/host_project mounted?")

    def _stop_demo(self):
        self._log("Stopping demo …")
        for cmd in (
            ["tmux", "kill-session", "-t", TMUX_SESSION],
            ["pkill", "-9", "-f", "gzserver"],
            ["pkill", "-9", "-f", "gzclient"],
            ["pkill", "-9", "-f", "ros2 launch tb3_follower"],
            ["pkill", "-9", "-f", "person_detector_node"],
            ["pkill", "-9", "-f", "follower_bt_node"],
            ["pkill", "-9", "-f", "robot_state_publisher"],
        ):
            subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._log("Stopped.")
        self.camera_label.setText("waiting for /camera/image_raw …")
        self.camera_label.setPixmap(QPixmap())
        self.last_detection = None

    def _restart_demo(self):
        self._stop_demo()
        QTimer.singleShot(1500, self._start_demo)

    def _close_gz_gui(self):
        subprocess.run(["pkill", "-9", "-f", "gzclient"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._log("Closed gzclient (Gazebo GUI). Simulation continues headless.")

    # ---------------- log ----------------

    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log_view.append(f"[{ts}] {msg}")

    # ---------------- shutdown ----------------

    def closeEvent(self, event):
        try:
            if self.ros_thread.node is not None:
                self.ros_thread.node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        self.ros_thread.quit()
        self.ros_thread.wait(1500)
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    win = Dashboard()
    win.show()
    # Graceful Ctrl-C
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
