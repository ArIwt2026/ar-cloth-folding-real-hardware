#!/usr/bin/env python3
"""
Multi-Camera 2x2 Grid Viewer for ROS 2.
Displays live streams from:
  1. Lucid Triton Color (/lucid/triton/image_color)
  2. Lucid Helios (/lucid/helios/image_raw)
  3. KUKA D455 Wrist (/iiwa7/d455/color/image_raw)
  4. Panda D455 Wrist (/panda/d455/color/image_raw)
"""

import sys
import signal
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from PyQt5 import QtCore, QtGui, QtWidgets


DEFAULT_CAMERAS = [
    {
        "title": "Lucid Triton Color",
        "default_topic": "/lucid/triton/image_color",
        "alt_topics": [
            "/lucid/triton/image_color",
            "/lucid/triton/image_raw",
        ],
    },
    {
        "title": "Lucid Helios",
        "default_topic": "/lucid/helios/image_raw",
        "alt_topics": [
            "/lucid/helios/image_raw",
        ],
    },
    {
        "title": "KUKA D455 Wrist",
        "default_topic": "/iiwa7/d455/color/image_raw",
        "alt_topics": [
            "/iiwa7/d455/color/image_raw",
            "/iiwa7/d455/depth/image_rect_raw",
            "/iiwa7/d455/aligned_depth_to_color/image_raw",
        ],
    },
    {
        "title": "Panda D455 Wrist",
        "default_topic": "/panda/d455/color/image_raw",
        "alt_topics": [
            "/panda/d455/color/image_raw",
            "/panda/d455/depth/image_rect_raw",
            "/panda/d455/aligned_depth_to_color/image_raw",
        ],
    },
]


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
    """Widget representing one camera cell in the grid."""

    double_clicked = QtCore.pyqtSignal(object)

    def __init__(self, title, default_topic, alt_topics, bridge, node, parent=None):
        super().__init__(parent)
        self.title = title
        self.current_topic = default_topic
        self.alt_topics = alt_topics
        self.bridge = bridge
        self.node = node
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

        # Header bar with Title and Topic Selector
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

        # Start subscription
        self.subscribe(self.current_topic)

    def subscribe(self, topic: str):
        if self.subscription is not None:
            self.node.destroy_subscription(self.subscription)
            self.subscription = None

        self.current_topic = topic
        self.image_label.set_status_text(f"Subscribed to {topic} ...")

        # QoS: Best effort subscriber is compatible with both Best Effort and Reliable publishers
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
                # Raw Bayer debayering
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
                if "rggb" in encoding:
                    cv_img = cv2.cvtColor(raw, cv2.COLOR_BayerRG2RGB)
                elif "bggr" in encoding:
                    cv_img = cv2.cvtColor(raw, cv2.COLOR_BayerBG2RGB)
                elif "gbrg" in encoding:
                    cv_img = cv2.cvtColor(raw, cv2.COLOR_BayerGB2RGB)
                else:
                    cv_img = cv2.cvtColor(raw, cv2.COLOR_BayerGR2RGB)
                fmt = QtGui.QImage.Format_RGB888
                bytes_per_line = 3 * msg.width
            elif encoding in ("bgr8", "8uc3"):
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                fmt = QtGui.QImage.Format_RGB888
                bytes_per_line = 3 * msg.width
            elif encoding in ("rgb8",):
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
                fmt = QtGui.QImage.Format_RGB888
                bytes_per_line = 3 * msg.width
            elif encoding in ("mono8", "8uc1"):
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
                fmt = QtGui.QImage.Format_Grayscale8
                bytes_per_line = msg.width
            elif encoding in ("mono16", "16uc1"):
                # Normalize 16-bit to 8-bit for clean contrast visualization (Helios / Depth)
                cv16 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                cv_img = cv2.normalize(cv16, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                fmt = QtGui.QImage.Format_Grayscale8
                bytes_per_line = msg.width
            elif encoding in ("32fc1",):
                cv32 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                cv32 = np.nan_to_num(cv32, nan=0.0)
                cv_img = cv2.normalize(cv32, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                fmt = QtGui.QImage.Format_Grayscale8
                bytes_per_line = msg.width
            else:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                if len(cv_img.shape) == 2:
                    cv_img = cv2.normalize(cv_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                    fmt = QtGui.QImage.Format_Grayscale8
                    bytes_per_line = msg.width
                else:
                    cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                    fmt = QtGui.QImage.Format_RGB888
                    bytes_per_line = 3 * msg.width

            h, w = cv_img.shape[:2]
            qimg = QtGui.QImage(cv_img.data, w, h, bytes_per_line, fmt).copy()
            pixmap = QtGui.QPixmap.fromImage(qimg)
            self.image_label.set_image(pixmap)
            self.frame_count += 1
        except Exception as e:
            self.image_label.set_status_text(f"Error decoding: {e}")

    def mouseDoubleClickEvent(self, event):
        self.double_clicked.emit(self)


class CameraGridViewer(QtWidgets.QMainWindow):
    """Main Window arranging 4 camera tiles in a 2x2 grid."""

    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.bridge = CvBridge()
        self.tiles = []
        self.is_maximized_tile = False
        self.maximized_tile = None

        self.setWindowTitle("Multi-Camera 2x2 Grid Viewer")
        self.resize(1280, 800)
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

        # Top Bar
        top_bar = QtWidgets.QHBoxLayout()
        title_label = QtWidgets.QLabel("Camera Grid Viewer (2x2)")
        title_label.setStyleSheet("font-size: 14px; font-weight: bold; color: #ffffff;")
        top_bar.addWidget(title_label)

        top_bar.addStretch(1)

        hint_label = QtWidgets.QLabel("Double-click any view to maximize / restore")
        hint_label.setStyleSheet("font-size: 11px; color: #777788;")
        top_bar.addWidget(hint_label)

        self.main_layout.addLayout(top_bar)

        # Grid Layout (2 rows x 2 cols)
        self.grid_widget = QtWidgets.QWidget(self)
        self.grid_layout = QtWidgets.QGridLayout(self.grid_widget)
        self.grid_layout.setContentsMargins(0, 0, 0, 0)
        self.grid_layout.setSpacing(6)

        positions = [(0, 0), (0, 1), (1, 0), (1, 1)]
        for i, cfg in enumerate(DEFAULT_CAMERAS):
            tile = CameraTile(
                title=cfg["title"],
                default_topic=cfg["default_topic"],
                alt_topics=cfg["alt_topics"],
                bridge=self.bridge,
                node=self.node,
            )
            tile.double_clicked.connect(self.toggle_maximize_tile)
            r, c = positions[i]
            self.grid_layout.addWidget(tile, r, c)
            self.tiles.append(tile)

        self.main_layout.addWidget(self.grid_widget, stretch=1)

    def toggle_maximize_tile(self, tile):
        if not self.is_maximized_tile:
            # Maximize clicked tile
            for t in self.tiles:
                if t != tile:
                    t.hide()
            self.is_maximized_tile = True
            self.maximized_tile = tile
        else:
            # Restore 2x2 grid
            for t in self.tiles:
                t.show()
            self.is_maximized_tile = False
            self.maximized_tile = None


def main():
    rclpy.init(args=sys.argv)
    ros_node = Node("multi_camera_grid_viewer")

    app = QtWidgets.QApplication(sys.argv)

    # Clean signal handling for SIGINT and SIGTERM
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
