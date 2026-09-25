#!/usr/bin/env python3
"""
Multi-Camera 2x2 Grid Viewer with Integrated Workcell Demonstration Recorder.

Displays live streams from:
  1. Lucid Triton Color (/lucid/triton/image_color)
  2. Lucid Helios Depth (/lucid/helios/image_raw)
  3. KUKA D455 Wrist (/iiwa7/d455/color/image_raw)
  4. Panda D455 Wrist (/panda/d455/color/image_raw)

Features:
  - Double-click any view to maximize / restore
  - Integrated One-Click Demonstration Recording directly to:
      /home/iwtros/Documents/ar/record_dataset/episode_XXXXXX_<timestamp>/
        ├── videos/ (4 synchronized MP4 video streams)
        ├── trajectory.h5 (LeRobot / ALOHA / RoboMimic compatible)
        ├── trajectory.csv (Human-readable spreadsheet)
        └── metadata.json (Calibration, frame rates, joint names)
  - Interactive Start/Stop button, delay countdown (3s/5s/10s), and hotkeys (Ctrl+R / Space)
"""

import csv
import datetime
import glob
import json
import math
import os
import queue
import signal
import subprocess
import sys
import threading
import time

import cv2
from cv_bridge import CvBridge
import h5py
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
import tf2_ros

try:
    from wsg50_msgs.msg import State as WSG50State
except ImportError:
    WSG50State = None

from PyQt5 import QtCore, QtGui, QtWidgets


WORKSPACE_ROOT = "/home/iwtros/Documents/ar"
DATASET_DIR = os.path.join(WORKSPACE_ROOT, "record_dataset", "episodes")

KUKA_ARM_JOINTS = [
    "iiwa7_A1", "iiwa7_A2", "iiwa7_A3", "iiwa7_A4",
    "iiwa7_A5", "iiwa7_A6", "iiwa7_A7"
]

PANDA_ARM_JOINTS = [
    "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
    "panda_joint5", "panda_joint6", "panda_joint7"
]

DEFAULT_CAMERAS = [
    {
        "key": "lucid_triton",
        "title": "Lucid Triton Color",
        "default_topic": "/lucid/triton/image_color",
        "alt_topics": [
            "/lucid/triton/image_color",
            "/lucid/triton/image_raw",
        ],
        "video_name": "lucid_triton_rgb.mp4",
    },
    {
        "key": "lucid_helios",
        "title": "Lucid Helios Depth",
        "default_topic": "/lucid/helios/image_raw",
        "alt_topics": [
            "/lucid/helios/image_raw",
        ],
        "video_name": "lucid_helios_depth.mp4",
    },
    {
        "key": "kuka_wrist",
        "title": "KUKA D455 Wrist",
        "default_topic": "/iiwa7/d455/color/image_raw",
        "alt_topics": [
            "/iiwa7/d455/color/image_raw",
            "/iiwa7/d455/depth/image_rect_raw",
            "/iiwa7/d455/aligned_depth_to_color/image_raw",
        ],
        "video_name": "kuka_wrist_rgb.mp4",
    },
    {
        "key": "panda_wrist",
        "title": "Panda D455 Wrist",
        "default_topic": "/panda/d455/color/image_raw",
        "alt_topics": [
            "/panda/d455/color/image_raw",
            "/panda/d455/depth/image_rect_raw",
            "/panda/d455/aligned_depth_to_color/image_raw",
        ],
        "video_name": "panda_wrist_rgb.mp4",
    },
]


class AsyncVideoWriter(threading.Thread):
    """Background worker thread writing video frames asynchronously."""

    def __init__(self, filepath, fps=30.0, codec="mp4v"):
        super().__init__(daemon=True)
        self.filepath = filepath
        self.fps = fps
        self.codec = codec
        self.queue = queue.Queue(maxsize=300)
        self.writer = None
        self.running = True
        self.frames_written = 0
        self.width = None
        self.height = None

    def put_frame(self, frame_bgr):
        if not self.running:
            return
        try:
            self.queue.put_nowait(frame_bgr)
        except queue.Full:
            pass  # Drop if disk is lagging to prevent memory bloat

    def run(self):
        while self.running or not self.queue.empty():
            try:
                frame = self.queue.get(timeout=0.1)
            except queue.Empty:
                continue

            if self.writer is None:
                self.height, self.width = frame.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*self.codec)
                self.writer = cv2.VideoWriter(
                    self.filepath, fourcc, float(self.fps), (self.width, self.height)
                )

            self.writer.write(frame)
            self.frames_written += 1
            self.queue.task_done()

        if self.writer is not None:
            self.writer.release()
            self.writer = None

    def stop(self):
        self.running = False


class DemonstrationRecorderEngine:
    """Handles multi-stream recording: 4 videos + joint states + TCP poses."""

    def __init__(self, node: Node, base_dir=DATASET_DIR, fps=30.0):
        self.node = node
        self.base_dir = base_dir
        self.fps = fps
        self.is_recording = False

        os.makedirs(self.base_dir, exist_ok=True)

        # Video writers map: key -> AsyncVideoWriter
        self.video_writers = {}
        self.video_counts = {}
        self.episode_dir = ""
        self.episode_name = ""

        # TF Buffer for End-Effector / TCP lookups
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        # Robot states cache
        self.lock = threading.Lock()
        self.kuka_joints = {j: 0.0 for j in KUKA_ARM_JOINTS}
        self.kuka_velocities = {j: 0.0 for j in KUKA_ARM_JOINTS}
        self.kuka_efforts = {j: 0.0 for j in KUKA_ARM_JOINTS}
        self.kuka_gripper_width = 0.0
        self.kuka_gripper_force = 0.0

        self.panda_joints = {j: 0.0 for j in PANDA_ARM_JOINTS}
        self.panda_velocities = {j: 0.0 for j in PANDA_ARM_JOINTS}
        self.panda_efforts = {j: 0.0 for j in PANDA_ARM_JOINTS}
        self.panda_gripper_width = 0.0

        # Trajectory buffer
        self.records = []
        self.start_time = 0.0
        self.last_frame_time = None
        self.recorded_duration = 0.0

        # Subscriptions for robot states
        qos_best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.node.create_subscription(
            JointState, "/iiwa7/joint_states", self._on_kuka_joint_state, qos_best_effort
        )
        self.node.create_subscription(
            JointState, "/panda/joint_states", self._on_panda_joint_state, qos_best_effort
        )
        self.node.create_subscription(
            JointState, "/panda/panda_gripper/joint_states", self._on_panda_gripper_state, qos_best_effort
        )
        if WSG50State is not None:
            self.node.create_subscription(
                WSG50State, "/wsg50/driver/state", self._on_wsg50_state, qos_best_effort
            )

    def _on_kuka_joint_state(self, msg: JointState):
        with self.lock:
            for name, pos, vel, eff in zip(
                msg.name,
                msg.position,
                msg.velocity if msg.velocity else [0.0] * len(msg.position),
                msg.effort if msg.effort else [0.0] * len(msg.position),
            ):
                if name in self.kuka_joints:
                    self.kuka_joints[name] = float(pos)
                    self.kuka_velocities[name] = float(vel)
                    self.kuka_efforts[name] = float(eff)
                elif "wsg50" in name:
                    self.kuka_gripper_width = abs(float(pos)) * 2.0

    def _on_wsg50_state(self, msg):
        with self.lock:
            self.kuka_gripper_width = float(msg.width)
            self.kuka_gripper_force = float(msg.force)

    def _on_panda_joint_state(self, msg: JointState):
        with self.lock:
            for name, pos, vel, eff in zip(
                msg.name,
                msg.position,
                msg.velocity if msg.velocity else [0.0] * len(msg.position),
                msg.effort if msg.effort else [0.0] * len(msg.position),
            ):
                if name in self.panda_joints:
                    self.panda_joints[name] = float(pos)
                    self.panda_velocities[name] = float(vel)
                    self.panda_efforts[name] = float(eff)

    def _on_panda_gripper_state(self, msg: JointState):
        with self.lock:
            if msg.position and len(msg.position) >= 2:
                self.panda_gripper_width = float(msg.position[0] + msg.position[1])

    def lookup_tcp_pose(self, parent_frame, child_frame):
        """Lookup 7-DoF pose [x, y, z, qx, qy, qz, qw] from TF."""
        try:
            t = self.tf_buffer.lookup_transform(parent_frame, child_frame, rclpy.time.Time())
            tr = t.transform.translation
            rot = t.transform.rotation
            return [tr.x, tr.y, tr.z, rot.x, rot.y, rot.z, rot.w]
        except Exception:
            return [float("nan")] * 7

    def start_recording(self):
        """Initialize episode folder and start async video writers."""
        if self.is_recording:
            return

        existing = glob.glob(os.path.join(self.base_dir, "episode_*"))
        next_ep_num = len(existing) + 1
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.episode_name = f"episode_{next_ep_num:06d}_{timestamp}"
        self.episode_dir = os.path.join(self.base_dir, self.episode_name)
        videos_dir = os.path.join(self.episode_dir, "videos")
        os.makedirs(videos_dir, exist_ok=True)

        self.video_writers = {}
        self.video_counts = {}
        for cfg in DEFAULT_CAMERAS:
            key = cfg["key"]
            filepath = os.path.join(videos_dir, cfg["video_name"])
            writer = AsyncVideoWriter(filepath, fps=self.fps, codec="mp4v")
            writer.start()
            self.video_writers[key] = writer
            self.video_counts[key] = 0

        self.records = []
        self.start_time = time.time()
        self.last_frame_time = self.start_time
        self.recorded_duration = 0.0
        self.is_recording = True

    def put_video_frame(self, camera_key, frame_bgr):
        """Enqueue frame for writing."""
        if self.is_recording and camera_key in self.video_writers:
            self.video_writers[camera_key].put_frame(frame_bgr)
            self.video_counts[camera_key] += 1

    def sample_step(self):
        """Synchronized sample tick at 30 Hz for trajectory logging."""
        if not self.is_recording:
            return

        now = time.time()
        dt = now - (self.last_frame_time or now)
        if dt < 1.0:
            self.recorded_duration += dt
        self.last_frame_time = now

        with self.lock:
            kuka_q = [self.kuka_joints[j] for j in KUKA_ARM_JOINTS]
            kuka_dq = [self.kuka_velocities[j] for j in KUKA_ARM_JOINTS]
            kuka_tau = [self.kuka_efforts[j] for j in KUKA_ARM_JOINTS]
            kuka_gw = self.kuka_gripper_width
            kuka_gf = self.kuka_gripper_force

            panda_q = [self.panda_joints[j] for j in PANDA_ARM_JOINTS]
            panda_dq = [self.panda_velocities[j] for j in PANDA_ARM_JOINTS]
            panda_tau = [self.panda_efforts[j] for j in PANDA_ARM_JOINTS]
            panda_gw = self.panda_gripper_width

        kuka_tcp_base = self.lookup_tcp_pose("iiwa7_link_0", "iiwa7_link_ee")
        kuka_tcp_world = self.lookup_tcp_pose("world", "iiwa7_link_ee")
        panda_tcp_base = self.lookup_tcp_pose("panda_link0", "panda_hand_tcp")
        panda_tcp_world = self.lookup_tcp_pose("world", "panda_hand_tcp")

        self.records.append({
            "timestamp": now,
            "step_index": len(self.records),
            "kuka_q": kuka_q,
            "kuka_dq": kuka_dq,
            "kuka_tau": kuka_tau,
            "kuka_gripper_width": kuka_gw,
            "kuka_gripper_force": kuka_gf,
            "kuka_tcp_base": kuka_tcp_base,
            "kuka_tcp_world": kuka_tcp_world,
            "panda_q": panda_q,
            "panda_dq": panda_dq,
            "panda_tau": panda_tau,
            "panda_gripper_width": panda_gw,
            "panda_tcp_base": panda_tcp_base,
            "panda_tcp_world": panda_tcp_world,
            "video_frame_counts": [
                self.video_counts.get("lucid_triton", 0),
                self.video_counts.get("lucid_helios", 0),
                self.video_counts.get("kuka_wrist", 0),
                self.video_counts.get("panda_wrist", 0),
            ],
        })

    def stop_recording(self):
        """Finalize video files and save HDF5/CSV/JSON."""
        if not self.is_recording:
            return None

        self.is_recording = False

        # Stop and flush all video writer threads
        for writer in self.video_writers.values():
            writer.stop()
        for writer in self.video_writers.values():
            writer.join(timeout=3.0)

        summary = {
            "episode_name": self.episode_name,
            "episode_dir": self.episode_dir,
            "duration": max(self.recorded_duration, 0.001),
            "sample_count": len(self.records),
            "video_counts": {k: w.frames_written for k, w in self.video_writers.items()},
        }

        if self.records:
            h5_path = os.path.join(self.episode_dir, "trajectory.h5")
            csv_path = os.path.join(self.episode_dir, "trajectory.csv")
            meta_path = os.path.join(self.episode_dir, "metadata.json")

            self._save_hdf5(h5_path)
            self._save_csv(csv_path)
            self._save_metadata(meta_path)

        return summary

    def _save_hdf5(self, h5_path):
        N = len(self.records)
        timestamps = np.array([r["timestamp"] for r in self.records], dtype=np.float64)
        step_indices = np.array([r["step_index"] for r in self.records], dtype=np.int64)

        kuka_q = np.array([r["kuka_q"] for r in self.records], dtype=np.float64)
        kuka_dq = np.array([r["kuka_dq"] for r in self.records], dtype=np.float64)
        kuka_tau = np.array([r["kuka_tau"] for r in self.records], dtype=np.float64)
        kuka_gw = np.array([r["kuka_gripper_width"] for r in self.records], dtype=np.float64)
        kuka_gf = np.array([r["kuka_gripper_force"] for r in self.records], dtype=np.float64)
        kuka_tcp_base = np.array([r["kuka_tcp_base"] for r in self.records], dtype=np.float64)
        kuka_tcp_world = np.array([r["kuka_tcp_world"] for r in self.records], dtype=np.float64)

        panda_q = np.array([r["panda_q"] for r in self.records], dtype=np.float64)
        panda_dq = np.array([r["panda_dq"] for r in self.records], dtype=np.float64)
        panda_tau = np.array([r["panda_tau"] for r in self.records], dtype=np.float64)
        panda_gw = np.array([r["panda_gripper_width"] for r in self.records], dtype=np.float64)
        panda_tcp_base = np.array([r["panda_tcp_base"] for r in self.records], dtype=np.float64)
        panda_tcp_world = np.array([r["panda_tcp_world"] for r in self.records], dtype=np.float64)
        video_indices = np.array([r["video_frame_counts"] for r in self.records], dtype=np.int64)

        with h5py.File(h5_path, "w") as f:
            obs = f.create_group("observations")
            obs.create_dataset("timestamps", data=timestamps, compression="gzip")
            obs.create_dataset("step_index", data=step_indices, compression="gzip")
            obs.create_dataset("video_frame_indices", data=video_indices, compression="gzip")

            kuka_grp = obs.create_group("kuka")
            kuka_grp.create_dataset("joint_positions", data=kuka_q, compression="gzip")
            kuka_grp.create_dataset("joint_velocities", data=kuka_dq, compression="gzip")
            kuka_grp.create_dataset("joint_efforts", data=kuka_tau, compression="gzip")
            kuka_grp.create_dataset("gripper_width", data=kuka_gw, compression="gzip")
            kuka_grp.create_dataset("gripper_force", data=kuka_gf, compression="gzip")
            kuka_grp.create_dataset("tcp_pose_base", data=kuka_tcp_base, compression="gzip")
            kuka_grp.create_dataset("tcp_pose_world", data=kuka_tcp_world, compression="gzip")

            panda_grp = obs.create_group("panda")
            panda_grp.create_dataset("joint_positions", data=panda_q, compression="gzip")
            panda_grp.create_dataset("joint_velocities", data=panda_dq, compression="gzip")
            panda_grp.create_dataset("joint_efforts", data=panda_tau, compression="gzip")
            panda_grp.create_dataset("gripper_width", data=panda_gw, compression="gzip")
            panda_grp.create_dataset("tcp_pose_base", data=panda_tcp_base, compression="gzip")
            panda_grp.create_dataset("tcp_pose_world", data=panda_tcp_world, compression="gzip")

            f.attrs["episode_name"] = self.episode_name
            f.attrs["total_samples"] = N
            f.attrs["recorded_duration_s"] = self.recorded_duration
            f.attrs["target_fps"] = self.fps

    def _save_csv(self, csv_path):
        header = ["timestamp", "step_index"]
        header += [f"kuka_{j}_pos" for j in KUKA_ARM_JOINTS]
        header += ["kuka_gripper_width", "kuka_gripper_force"]
        header += [f"kuka_tcp_base_{axis}" for axis in ["x", "y", "z", "qx", "qy", "qz", "qw"]]
        header += [f"kuka_tcp_world_{axis}" for axis in ["x", "y", "z", "qx", "qy", "qz", "qw"]]
        header += [f"panda_{j}_pos" for j in PANDA_ARM_JOINTS]
        header += ["panda_gripper_width"]
        header += [f"panda_tcp_base_{axis}" for axis in ["x", "y", "z", "qx", "qy", "qz", "qw"]]
        header += [f"panda_tcp_world_{axis}" for axis in ["x", "y", "z", "qx", "qy", "qz", "qw"]]

        with open(csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            for r in self.records:
                row = [f"{r['timestamp']:.4f}", r["step_index"]]
                row += [f"{v:.5f}" for v in r["kuka_q"]]
                row += [f"{r['kuka_gripper_width']:.4f}", f"{r['kuka_gripper_force']:.2f}"]
                row += [f"{v:.5f}" for v in r["kuka_tcp_base"]]
                row += [f"{v:.5f}" for v in r["kuka_tcp_world"]]
                row += [f"{v:.5f}" for v in r["panda_q"]]
                row += [f"{r['panda_gripper_width']:.4f}"]
                row += [f"{v:.5f}" for v in r["panda_tcp_base"]]
                row += [f"{v:.5f}" for v in r["panda_tcp_world"]]
                writer.writerow(row)

    def _save_metadata(self, meta_path):
        meta = {
            "episode_name": self.episode_name,
            "created_at": datetime.datetime.now().isoformat(),
            "target_fps": self.fps,
            "total_samples": len(self.records),
            "recorded_duration_s": round(self.recorded_duration, 3),
            "cameras": {
                key: {
                    "frames_written": w.frames_written,
                    "resolution": [w.width, w.height],
                    "file": f"videos/{key}.mp4",
                }
                for key, w in self.video_writers.items()
            },
            "robots": {
                "kuka": {
                    "joint_names": KUKA_ARM_JOINTS,
                    "topic": "/iiwa7/joint_states",
                    "base_frame": "iiwa7_link_0",
                    "tcp_frame": "iiwa7_link_ee",
                },
                "panda": {
                    "joint_names": PANDA_ARM_JOINTS,
                    "topic": "/panda/joint_states",
                    "base_frame": "panda_link0",
                    "tcp_frame": "panda_hand_tcp",
                },
            },
        }

        calib_file = os.path.join(WORKSPACE_ROOT, "calibration_export/calibration.yaml")
        if os.path.exists(calib_file):
            try:
                import yaml
                with open(calib_file, "r") as f:
                    meta["calibration_export"] = yaml.safe_load(f)
            except Exception:
                pass

        with open(meta_path, "w") as f:
            json.dump(meta, f, indent=2)


class AspectRatioLabel(QtWidgets.QLabel):
    """Custom QLabel that scales QPixmap preserving aspect ratio."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(160, 120)
        self.setAlignment(QtCore.Qt.AlignCenter)
        self.setStyleSheet("background-color: #121216; color: #888899; font-size: 13px;")
        self._pixmap = None

    def set_image(self, pixmap: QtGui.QPixmap):
        self._pixmap = pixmap
        self._update_display()

    def set_status_text(self, text: str):
        self._pixmap = None
        self.setText(text)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._update_display()

    def _update_display(self):
        if self._pixmap and not self._pixmap.isNull():
            scaled = self._pixmap.scaled(
                self.size(), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation
            )
            self.setPixmap(scaled)


class CameraTile(QtWidgets.QFrame):
    """Widget representing one camera cell in the 2x2 grid."""

    double_clicked = QtCore.pyqtSignal(object)

    def __init__(self, camera_key, title, default_topic, alt_topics, bridge, node, recorder=None, parent=None):
        super().__init__(parent)
        self.camera_key = camera_key
        self.title = title
        self.current_topic = default_topic
        self.alt_topics = alt_topics
        self.bridge = bridge
        self.node = node
        self.recorder = recorder
        self.subscription = None
        self.frame_count = 0

        self.setObjectName("cameraTile")
        self.setStyleSheet("""
            QFrame#cameraTile {
                background-color: #1e1e24;
                border: 1px solid #32323e;
                border-radius: 6px;
            }
        """)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(4)

        # Header bar
        header = QtWidgets.QHBoxLayout()
        header.setContentsMargins(2, 2, 2, 2)

        self.title_label = QtWidgets.QLabel(title)
        self.title_label.setStyleSheet("font-weight: bold; font-size: 13px; color: #eeeeff;")
        header.addWidget(self.title_label)

        header.addStretch(1)

        self.topic_combo = QtWidgets.QComboBox()
        self.topic_combo.setStyleSheet("""
            QComboBox {
                background-color: #2b2b36;
                color: #ccccdd;
                border: 1px solid #3d3d4d;
                border-radius: 4px;
                padding: 2px 8px;
                font-size: 11px;
            }
            QComboBox::drop-down { border: none; }
            QComboBox QAbstractItemView {
                background-color: #2b2b36;
                color: #eeeeff;
                selection-background-color: #3b5998;
            }
        """)
        for t in alt_topics:
            self.topic_combo.addItem(t)
        self.topic_combo.setCurrentText(default_topic)
        self.topic_combo.currentTextChanged.connect(self.change_topic)
        header.addWidget(self.topic_combo)

        layout.addLayout(header)

        # Image view label
        self.image_label = AspectRatioLabel(self)
        self.image_label.set_status_text(f"Waiting for {default_topic} ...")
        layout.addWidget(self.image_label, stretch=1)

        self.subscribe(default_topic)

    def subscribe(self, topic: str):
        if self.subscription is not None:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None

        self.current_topic = topic
        self.image_label.set_status_text(f"Subscribing to {topic} ...")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subscription = self.node.create_subscription(
            Image, topic, self.on_image_msg, qos
        )

    def change_topic(self, new_topic: str):
        if new_topic and new_topic != self.current_topic:
            self.subscribe(new_topic)

    def on_image_msg(self, msg: Image):
        try:
            encoding = msg.encoding.lower()

            if "bayer" in encoding:
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
                if "rggb" in encoding:
                    cv_bgr = cv2.cvtColor(raw, cv2.COLOR_BayerRG2BGR)
                elif "bggr" in encoding:
                    cv_bgr = cv2.cvtColor(raw, cv2.COLOR_BayerBG2BGR)
                elif "gbrg" in encoding:
                    cv_bgr = cv2.cvtColor(raw, cv2.COLOR_BayerGB2BGR)
                else:
                    cv_bgr = cv2.cvtColor(raw, cv2.COLOR_BayerGR2BGR)
                cv_rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)
                fmt = QtGui.QImage.Format_RGB888
                bytes_per_line = 3 * msg.width
            elif encoding in ("bgr8", "8uc3"):
                cv_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                cv_rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)
                fmt = QtGui.QImage.Format_RGB888
                bytes_per_line = 3 * msg.width
            elif encoding in ("rgb8",):
                cv_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                cv_bgr = cv2.cvtColor(cv_rgb, cv2.COLOR_RGB2BGR)
                fmt = QtGui.QImage.Format_RGB888
                bytes_per_line = 3 * msg.width
            elif "mono16" in encoding:
                raw16 = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width))
                norm8 = cv2.normalize(raw16, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                cv_bgr = cv2.cvtColor(norm8, cv2.COLOR_GRAY2BGR)
                cv_rgb = cv_bgr
                fmt = QtGui.QImage.Format_RGB888
                bytes_per_line = 3 * msg.width
            elif "mono8" in encoding or "8uc1" in encoding:
                gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
                cv_bgr = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                cv_rgb = cv_bgr
                fmt = QtGui.QImage.Format_RGB888
                bytes_per_line = 3 * msg.width
            else:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                if len(cv_img.shape) == 2:
                    norm8 = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                    cv_bgr = cv2.cvtColor(norm8, cv2.COLOR_GRAY2BGR)
                    cv_rgb = cv_bgr
                else:
                    cv_bgr = cv_img
                    cv_rgb = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2RGB)
                fmt = QtGui.QImage.Format_RGB888
                bytes_per_line = 3 * msg.width

            # Pass to background recorder if recording
            if self.recorder and self.recorder.is_recording:
                self.recorder.put_video_frame(self.camera_key, cv_bgr)

            h, w = cv_rgb.shape[:2]
            qimg = QtGui.QImage(cv_rgb.data, w, h, bytes_per_line, fmt).copy()
            pixmap = QtGui.QPixmap.fromImage(qimg)
            self.image_label.set_image(pixmap)
            self.frame_count += 1
        except Exception as e:
            self.image_label.set_status_text(f"Error decoding: {e}")

    def mouseDoubleClickEvent(self, event):
        self.double_clicked.emit(self)


class CameraGridViewer(QtWidgets.QMainWindow):
    """Main Window: 4 camera tiles in 2x2 grid with Demonstration Recording."""

    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.bridge = CvBridge()
        self.tiles = []
        self.is_maximized_tile = False
        self.maximized_tile = None

        # Demonstration Recorder Engine
        self.recorder = DemonstrationRecorderEngine(self.node, base_dir=DATASET_DIR, fps=30.0)

        # Countdown state
        self.countdown_remaining = 0
        self.countdown_timer = QtCore.QTimer(self)
        self.countdown_timer.timeout.connect(self._on_countdown_tick)

        # GUI update timer for recording stats
        self.stats_timer = QtCore.QTimer(self)
        self.stats_timer.timeout.connect(self._update_recording_ui)
        self.stats_timer.start(100)

        # Master trajectory sampler timer at 30 Hz
        self.sample_timer = QtCore.QTimer(self)
        self.sample_timer.timeout.connect(self.recorder.sample_step)
        self.sample_timer.start(33)  # ~30 Hz

        self.setWindowTitle("Multi-Camera 2x2 Grid & Demonstration Recorder")
        self.resize(1340, 850)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #141418;
            }
        """)

        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)

        self.main_layout = QtWidgets.QVBoxLayout(central)
        self.main_layout.setContentsMargins(8, 8, 8, 8)
        self.main_layout.setSpacing(6)

        # -------------------------------------------------------------
        # Top Bar with Title, Recording Controls & Shortcuts
        # -------------------------------------------------------------
        top_bar = QtWidgets.QHBoxLayout()

        title_label = QtWidgets.QLabel("Workcell Multi-Camera Grid")
        title_label.setStyleSheet("font-size: 15px; font-weight: bold; color: #ffffff;")
        top_bar.addWidget(title_label)

        top_bar.addSpacing(20)

        # Recording Section
        rec_frame = QtWidgets.QFrame()
        rec_frame.setStyleSheet("""
            QFrame {
                background-color: #1c1c24;
                border: 1px solid #323242;
                border-radius: 6px;
                padding: 2px;
            }
        """)
        rec_layout = QtWidgets.QHBoxLayout(rec_frame)
        rec_layout.setContentsMargins(8, 3, 8, 3)
        rec_layout.setSpacing(10)

        # Delay Combobox
        delay_label = QtWidgets.QLabel("Delay:")
        delay_label.setStyleSheet("color: #aaaabb; font-size: 12px;")
        rec_layout.addWidget(delay_label)

        self.delay_combo = QtWidgets.QComboBox()
        self.delay_combo.setStyleSheet("""
            QComboBox {
                background-color: #2b2b36;
                color: #eeeeff;
                border: 1px solid #3d3d4d;
                border-radius: 4px;
                padding: 2px 8px;
                font-size: 11px;
            }
        """)
        self.delay_combo.addItem("No Delay", 0)
        self.delay_combo.addItem("3s Delay", 3)
        self.delay_combo.addItem("5s Delay", 5)
        self.delay_combo.addItem("10s Delay", 10)
        rec_layout.addWidget(self.delay_combo)

        # Big Record Button
        self.rec_button = QtWidgets.QPushButton("● Record Demonstration")
        self.rec_button.setCursor(QtCore.Qt.PointingHandCursor)
        self.rec_button.setFixedHeight(32)
        self.rec_button.setStyleSheet("""
            QPushButton {
                background-color: #c62828;
                color: #ffffff;
                font-size: 12px;
                font-weight: bold;
                border-radius: 4px;
                padding: 0 16px;
            }
            QPushButton:hover {
                background-color: #d32f2f;
            }
        """)
        self.rec_button.clicked.connect(self.toggle_recording)
        rec_layout.addWidget(self.rec_button)

        # Status / Timer Label
        self.status_label = QtWidgets.QLabel("Ready (Space / Ctrl+R)")
        self.status_label.setStyleSheet("color: #88cc88; font-size: 12px; font-weight: bold;")
        rec_layout.addWidget(self.status_label)

        # Open Dataset Folder Button
        self.open_folder_button = QtWidgets.QPushButton("📁 Dataset Folder")
        self.open_folder_button.setCursor(QtCore.Qt.PointingHandCursor)
        self.open_folder_button.setFixedHeight(28)
        self.open_folder_button.setStyleSheet("""
            QPushButton {
                background-color: #2b2b38;
                color: #ccccdd;
                border: 1px solid #3d3d4e;
                border-radius: 4px;
                font-size: 11px;
                padding: 0 10px;
            }
            QPushButton:hover {
                background-color: #3b3b4d;
                color: #ffffff;
            }
        """)
        self.open_folder_button.clicked.connect(self.open_dataset_folder)
        rec_layout.addWidget(self.open_folder_button)

        top_bar.addWidget(rec_frame)

        top_bar.addStretch(1)

        hint_label = QtWidgets.QLabel("Double-click tile to maximize | Space: Record")
        hint_label.setStyleSheet("font-size: 11px; color: #777788;")
        top_bar.addWidget(hint_label)

        self.main_layout.addLayout(top_bar)

        # -------------------------------------------------------------
        # Grid Layout (2 rows x 2 cols)
        # -------------------------------------------------------------
        self.grid_widget = QtWidgets.QWidget(self)
        self.grid_layout = QtWidgets.QGridLayout(self.grid_widget)
        self.grid_layout.setContentsMargins(0, 0, 0, 0)
        self.grid_layout.setSpacing(6)

        positions = [(0, 0), (0, 1), (1, 0), (1, 1)]
        for i, cfg in enumerate(DEFAULT_CAMERAS):
            tile = CameraTile(
                camera_key=cfg["key"],
                title=cfg["title"],
                default_topic=cfg["default_topic"],
                alt_topics=cfg["alt_topics"],
                bridge=self.bridge,
                node=self.node,
                recorder=self.recorder,
            )
            tile.double_clicked.connect(self.toggle_maximize_tile)
            r, c = positions[i]
            self.grid_layout.addWidget(tile, r, c)
            self.tiles.append(tile)

        self.main_layout.addWidget(self.grid_widget, stretch=1)

    def keyPressEvent(self, event):
        """Keyboard shortcuts: Space or Ctrl+R toggles recording."""
        if event.key() == QtCore.Qt.Key_Space:
            self.toggle_recording()
        elif event.key() == QtCore.Qt.Key_R and (event.modifiers() & QtCore.Qt.ControlModifier):
            self.toggle_recording()
        else:
            super().keyPressEvent(event)

    def toggle_recording(self):
        """Handle Start / Stop demonstration recording."""
        if self.countdown_timer.isActive():
            # If countdown active, click starts immediately
            self.countdown_timer.stop()
            self._do_start_recording()
            return

        if not self.recorder.is_recording:
            delay = self.delay_combo.currentData()
            if delay > 0:
                self.countdown_remaining = delay
                self.rec_button.setText(f"Starting in {self.countdown_remaining}s (Click = Now)")
                self.rec_button.setStyleSheet("""
                    QPushButton {
                        background-color: #f57f17;
                        color: #ffffff;
                        font-size: 12px;
                        font-weight: bold;
                        border-radius: 4px;
                    }
                """)
                self.status_label.setText(f"Countdown: {self.countdown_remaining}s...")
                self.status_label.setStyleSheet("color: #ffcc00; font-size: 12px; font-weight: bold;")
                self.countdown_timer.start(1000)
            else:
                self._do_start_recording()
        else:
            # Stop recording
            summary = self.recorder.stop_recording()
            self.rec_button.setText("● Record Demonstration")
            self.rec_button.setStyleSheet("""
                QPushButton {
                    background-color: #c62828;
                    color: #ffffff;
                    font-size: 12px;
                    font-weight: bold;
                    border-radius: 4px;
                    padding: 0 16px;
                }
                QPushButton:hover {
                    background-color: #d32f2f;
                }
            """)
            if summary and summary["sample_count"] > 0:
                self.status_label.setText(
                    f"✔ Saved {summary['episode_name']} ({summary['duration']:.1f}s, {summary['sample_count']} samples)"
                )
                self.status_label.setStyleSheet("color: #4caf50; font-size: 12px; font-weight: bold;")
            else:
                self.status_label.setText("Ready (0 samples recorded)")
                self.status_label.setStyleSheet("color: #88cc88; font-size: 12px; font-weight: bold;")

    def _on_countdown_tick(self):
        self.countdown_remaining -= 1
        if self.countdown_remaining > 0:
            self.rec_button.setText(f"Starting in {self.countdown_remaining}s (Click = Now)")
            self.status_label.setText(f"Countdown: {self.countdown_remaining}s...")
        else:
            self.countdown_timer.stop()
            self._do_start_recording()

    def _do_start_recording(self):
        self.recorder.start_recording()
        self.rec_button.setText("■ Stop Recording")
        self.rec_button.setStyleSheet("""
            QPushButton {
                background-color: #b71c1c;
                color: #ffffff;
                font-size: 12px;
                font-weight: bold;
                border-radius: 4px;
                padding: 0 16px;
            }
            QPushButton:hover {
                background-color: #d32f2f;
            }
        """)

    def _update_recording_ui(self):
        if self.recorder.is_recording:
            dur = self.recorder.recorded_duration
            mins = int(dur) // 60
            secs = dur % 60
            n_samples = len(self.recorder.records)
            ep = self.recorder.episode_name.split("_")[1] if "_" in self.recorder.episode_name else ""
            self.status_label.setText(
                f"● REC {mins:02d}:{secs:04.1f} | {n_samples} samples | ep_{ep}"
            )
            self.status_label.setStyleSheet("color: #ff5252; font-size: 12px; font-weight: bold;")

    def open_dataset_folder(self):
        """Open the record_dataset folder in the system file manager."""
        folder = os.path.abspath(DATASET_DIR)
        os.makedirs(folder, exist_ok=True)
        try:
            subprocess.Popen(["xdg-open", folder])
        except Exception as e:
            QtWidgets.QMessageBox.information(self, "Dataset Directory", f"Dataset path:\n{folder}")

    def toggle_maximize_tile(self, tile):
        if not self.is_maximized_tile:
            for t in self.tiles:
                if t != tile:
                    t.hide()
            self.is_maximized_tile = True
            self.maximized_tile = tile
        else:
            for t in self.tiles:
                t.show()
            self.is_maximized_tile = False
            self.maximized_tile = None

    def closeEvent(self, event):
        """Ensure recording is cleanly finalized on window close."""
        if self.recorder.is_recording:
            self.recorder.stop_recording()
        super().closeEvent(event)


def main():
    rclpy.init(args=sys.argv)
    ros_node = Node("multi_camera_grid_viewer")

    app = QtWidgets.QApplication(sys.argv)

    def sig_handler(signum, frame):
        app.quit()

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    viewer = CameraGridViewer(node=ros_node)
    viewer.show()

    # ROS 2 spin timer integrated cleanly with Qt event loop
    spin_timer = QtCore.QTimer()
    spin_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.001))
    spin_timer.start(10)  # 100 Hz pump for ultra-low latency

    ret = 0
    try:
        ret = app.exec_()
    except KeyboardInterrupt:
        ret = 0
    finally:
        spin_timer.stop()
        if rclpy.ok():
            try:
                ros_node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
        sys.exit(ret)


if __name__ == "__main__":
    main()
