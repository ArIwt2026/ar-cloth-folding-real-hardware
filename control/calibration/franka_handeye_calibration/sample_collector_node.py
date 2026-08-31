from __future__ import annotations

import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import tf2_ros
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from .charuco_pose_estimator import detect_aruco_pose, get_dictionary
from .job_config import CameraIntrinsics


class SampleCollector(Node):
    """Raw image + robot pose collector; deliberately contains no CV detection."""

    def __init__(self):
        super().__init__("handeye_sample_collector")
        self.image_topic = self.declare_parameter("image_topic", "/lucid/triton/image_color").value
        self.info_topic = self.declare_parameter("camera_info_topic", "/lucid/triton/camera_info").value
        self.base_frame = self.declare_parameter("base_frame", "iiwa7_link_0").value
        self.ee_frame = self.declare_parameter("ee_frame", "iiwa7_link_ee").value
        self.output_dir = Path(self.declare_parameter("output_dir", "calibration_data/raw_samples/kuka").value).expanduser()
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.image = None
        self.image_msg = None
        self.camera_info = None
        self.lock = __import__("threading").Lock()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.button = (20, 20, 260, 60)
        self.window = "Raw Hand-Eye Sample Collector"
        self.saved = 0
        self.create_subscription(Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.info_topic, self.info_cb, qos_profile_sensor_data)

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self.lock:
                self.image, self.image_msg = frame, msg
        except Exception as exc:
            self.get_logger().error(f"Image conversion failed: {exc}")

    def info_cb(self, msg):
        with self.lock:
            self.camera_info = msg

    def save_sample(self):
        with self.lock:
            frame, msg, info = self.image, self.image_msg, self.camera_info
        if frame is None or msg is None or info is None:
            self.get_logger().warn("Cannot save: waiting for image and CameraInfo")
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, rclpy.time.Time())
        except Exception as exc:
            self.get_logger().warn(f"Cannot save: TF {self.base_frame} -> {self.ee_frame} unavailable: {exc}")
            return
        # Validate the raw frame before saving, but never modify the raw image.
        k = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        d = np.asarray(info.d, dtype=np.float64)
        if int(info.width) == frame.shape[1] and int(info.height) != frame.shape[0]:
            k[1, 2] -= 0.5 * (int(info.height) - frame.shape[0])
        intrinsics = CameraIntrinsics(k, d, frame.shape[1], frame.shape[0], Path("collector"))
        detection = detect_aruco_pose(
            frame, get_dictionary("DICT_APRILTAG_25h9"), intrinsics, 0.07, robust=True
        )
        if not detection.success:
            self.get_logger().warn("Cannot save: AprilTag 25h9 was not detected in this raw frame")
            return
        self.saved += 1
        stem = f"sample_{self.saved:04d}"
        image_path = self.output_dir / f"{stem}.png"
        metadata_path = self.output_dir / f"{stem}.yaml"
        cv2.imwrite(str(image_path), frame)  # untouched full-resolution frame
        t, q = tf.transform.translation, tf.transform.rotation
        metadata = {
            "image": str(image_path.resolve()),
            "image_topic": self.image_topic,
            "image_encoding": msg.encoding,
            "image_width": int(msg.width),
            "image_height": int(msg.height),
            "image_stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
            "camera_info_topic": self.info_topic,
            "camera_frame": info.header.frame_id,
            "camera_info_width": int(info.width),
            "camera_info_height": int(info.height),
            "camera_matrix": [float(value) for value in info.k],
            "distortion_model": info.distortion_model,
            "distortion_coefficients": [float(value) for value in info.d],
            "base_frame": self.base_frame,
            "ee_frame": self.ee_frame,
            "tf_stamp": {"sec": int(tf.header.stamp.sec), "nanosec": int(tf.header.stamp.nanosec)},
            "base_to_ee_translation": [float(t.x), float(t.y), float(t.z)],
            "base_to_ee_quaternion_xyzw": [float(q.x), float(q.y), float(q.z), float(q.w)],
            "marker_ids": [int(v) for v in detection.marker_ids.flatten()],
        }
        metadata_path.write_text(yaml.safe_dump(metadata, sort_keys=False))
        self.get_logger().info(f"Saved raw sample {self.saved}: {image_path}")

    def mouse_cb(self, event, x, y, _flags, _param):
        bx, by, bw, bh = self.button
        if event == cv2.EVENT_LBUTTONUP and bx <= x <= bx + bw and by <= y <= by + bh:
            self.save_sample()

    def spin_gui(self):
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window, self.mouse_cb)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            with self.lock:
                frame = None if self.image is None else self.image.copy()
            if frame is None:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "Waiting for camera...", (40, 240), 0, 0.9, (0, 255, 255), 2)
            display = cv2.resize(frame, (960, int(frame.shape[0] * 960 / frame.shape[1]))) if frame.shape[1] > 960 else frame
            cv2.rectangle(display, (20, 20), (280, 80), (30, 130, 30), -1)
            cv2.putText(display, "SAVE SAMPLE (S)", (38, 58), 0, 0.8, (255, 255, 255), 2)
            cv2.putText(display, f"Saved: {self.saved}", (20, display.shape[0] - 20), 0, 0.7, (0, 255, 255), 2)
            cv2.imshow(self.window, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("s"), ord("S")):
                self.save_sample()
            elif key in (ord("q"), 27):
                break
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = SampleCollector()
    try:
        node.spin_gui()
    finally:
        node.destroy_node()
        rclpy.shutdown()
