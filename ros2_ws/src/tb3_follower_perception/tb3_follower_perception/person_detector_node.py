"""YOLOv8 person detector ROS 2 node.

Subscribes to /camera/image_raw + /camera/camera_info + /scan.
Publishes /person/detection (tb3_follower_msgs/PersonDetection) at up to max_rate_hz.
"""
from __future__ import annotations

import os
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo, LaserScan
from cv_bridge import CvBridge

from tb3_follower_msgs.msg import PersonDetection

from tb3_follower_perception.geometry import (
    visual_distance,
    bbox_cx_to_lidar_index,
    fuse_distance,
)


class PersonDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("person_detector_node")

        # ----- Params -----
        self.declare_parameter("model_path", "~/models/yolov8n.pt")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("imgsz", 320)
        self.declare_parameter("max_rate_hz", 10.0)
        self.declare_parameter("person_height_m", 1.7)
        self.declare_parameter("camera_hfov_rad", 1.085)
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("detection_topic", "/person/detection")

        gp = self.get_parameter
        self.conf_threshold = gp("conf_threshold").value
        self.imgsz = int(gp("imgsz").value)
        self.max_rate_hz = float(gp("max_rate_hz").value)
        self.person_height_m = float(gp("person_height_m").value)
        self.camera_hfov_rad = float(gp("camera_hfov_rad").value)
        self.min_period_s = 1.0 / max(self.max_rate_hz, 1e-6)

        # ----- YOLO model -----
        # Import here so unit tests of geometry.py don't need ultralytics.
        from ultralytics import YOLO  # noqa: WPS433

        model_path = os.path.expanduser(gp("model_path").value)
        if not Path(model_path).exists():
            self.get_logger().error(f"Model not found at {model_path}. Run scripts/03-install-yolo.sh.")
            raise FileNotFoundError(model_path)
        self.get_logger().info(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)

        # ----- Bridge + state -----
        self.bridge = CvBridge()
        self.focal_px: float | None = None
        self.last_scan: LaserScan | None = None
        self.last_pub_time: float = 0.0
        self.busy: bool = False  # frame-drop guard

        # ----- QoS -----
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ----- Subs / pub -----
        self.create_subscription(
            CameraInfo, gp("camera_info_topic").value, self._on_camera_info, 1
        )
        self.create_subscription(
            LaserScan, gp("scan_topic").value, self._on_scan, sensor_qos
        )
        self.create_subscription(
            Image, gp("image_topic").value, self._on_image, sensor_qos
        )
        self.pub = self.create_publisher(
            PersonDetection, gp("detection_topic").value, 10
        )

        self.get_logger().info("person_detector_node ready.")

    # ---------- callbacks ----------

    def _on_camera_info(self, msg: CameraInfo) -> None:
        # Take focal length from K[0,0] (fx). Stop subscribing after first valid read.
        if self.focal_px is None and msg.k[0] > 0:
            self.focal_px = float(msg.k[0])
            self.get_logger().info(f"Focal length captured: fx={self.focal_px:.1f} px")

    def _on_scan(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self.last_pub_time < self.min_period_s:
            return  # rate limit
        if self.busy:
            return  # previous inference still running
        if self.focal_px is None:
            return  # wait for camera info

        self.busy = True
        try:
            self._process_frame(msg, now)
        finally:
            self.busy = False

    # ---------- core ----------

    def _process_frame(self, img_msg: Image, now: float) -> None:
        frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        h_px, w_px = frame.shape[:2]

        t0 = time.monotonic()
        results = self.model.predict(
            source=frame,
            imgsz=self.imgsz,
            conf=self.conf_threshold,
            classes=[0],          # COCO class 0 = person
            verbose=False,
        )
        infer_ms = (time.monotonic() - t0) * 1000.0

        out = PersonDetection()
        out.stamp = self.get_clock().now().to_msg()

        boxes = results[0].boxes if results else None
        if boxes is None or len(boxes) == 0:
            out.detected = False
            out.distance = -1.0
            self.pub.publish(out)
            self.last_pub_time = now
            return

        # Pick highest-confidence box
        confs = boxes.conf.cpu().numpy()
        best = int(np.argmax(confs))
        x1, y1, x2, y2 = boxes.xyxy[best].cpu().numpy().tolist()
        conf = float(confs[best])

        bbox_w_px = max(x2 - x1, 1.0)
        bbox_h_px = max(y2 - y1, 1.0)
        bbox_cx_px = (x1 + x2) / 2.0
        bbox_cy_px = (y1 + y2) / 2.0

        # Normalize
        out.detected = True
        out.bbox_cx = float(bbox_cx_px / w_px)
        out.bbox_cy = float(bbox_cy_px / h_px)
        out.bbox_w = float(bbox_w_px / w_px)
        out.bbox_h = float(bbox_h_px / h_px)
        out.confidence = conf

        # Distance fusion
        d_visual = visual_distance(
            person_height_m=self.person_height_m,
            bbox_h_px=bbox_h_px,
            focal_px=self.focal_px,
        )

        d_lidar = float("nan")
        if self.last_scan is not None:
            idx = bbox_cx_to_lidar_index(
                bbox_cx=out.bbox_cx,
                camera_hfov_rad=self.camera_hfov_rad,
                scan_angle_min=self.last_scan.angle_min,
                scan_angle_increment=self.last_scan.angle_increment,
                num_beams=len(self.last_scan.ranges),
            )
            if 0 <= idx < len(self.last_scan.ranges):
                d_lidar = float(self.last_scan.ranges[idx])

        out.distance = fuse_distance(
            visual_d=d_visual,
            lidar_d=d_lidar,
            lidar_range_max=(self.last_scan.range_max if self.last_scan else 10.0),
        )

        self.pub.publish(out)
        self.last_pub_time = now

        if infer_ms > 100.0:
            self.get_logger().warn(
                f"YOLO inference slow ({infer_ms:.0f} ms) — dropping next frame to keep realtime."
            )


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
