#!/usr/bin/env python3
"""
navigator.py - Navigation Node
- Subscribes to /scanner_status → LOCKED gelince başlar
- Subscribes to /detection → JSON formatında detection listesi
- Subscribes to depth image → mesafe ölçer
- Subscribes to /odom → robot konumu (SLAM için)
- Publishes /cmd_vel → robotu hareket ettirir
- Publishes /navigator/status → NAVIGATING / ARRIVED / LOST
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import numpy as np
import json

SCANNER_STATUS_TOPIC = "/scanner_status"
DETECTION_TOPIC      = "/detection"
DEPTH_TOPIC          = "/a200_0000/sensors/camera_0/depth/image"
ODOM_TOPIC           = "/a200_0000/platform/odom"
CMD_VEL_TOPIC        = "/a200_0000/cmd_vel"
STATUS_TOPIC         = "/navigator/status"

IMAGE_W   = 640
STOP_DIST = 0.65
KP_ANG    = 0.003
KP_LIN    = 0.3
MAX_ANG   = 0.4
MAX_LIN   = 0.25


class State:
    IDLE       = "IDLE"
    NAVIGATING = "NAVIGATING"
    ARRIVED    = "ARRIVED"


class NavigatorNode(Node):
    def __init__(self):
        super().__init__("navigator")

        self.bridge      = CvBridge()
        self.state       = State.IDLE
        self.target      = None
        self.detections  = []   # JSON list
        self.depth_frame = None
        self.miss_count  = 0
        self.MISS_LIMIT  = 5

        # Publishers
        self.vel_pub    = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)

        # Subscribers
        self.create_subscription(String, SCANNER_STATUS_TOPIC, self._scanner_cb, 10)
        self.create_subscription(String, DETECTION_TOPIC, self._detection_cb, 10)
        self.create_subscription(Image, DEPTH_TOPIC, self._depth_cb, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, 10)

        self.create_timer(0.15, self._loop)

        self.get_logger().info("Navigator hazır → IDLE")

    # ── Callbacks ─────────────────────────────
    def _scanner_cb(self, msg):
        status = msg.data.strip()

        if status.startswith("FOUND:"):
            self.target = status.split(":")[1]
            self.get_logger().info(f"[NAVIGATOR] Hedef: {self.target}")

        elif status == "LOCKED":
            self.state = State.NAVIGATING
            self.miss_count = 0
            self.get_logger().info("[NAVIGATOR] LOCKED → NAVIGATING başlıyor")
            self._publish_status("NAVIGATING")

        elif status == "SCANNING":
            self.state = State.IDLE

    def _detection_cb(self, msg):
        try:
            self.detections = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Detection parse error: {e}")
            self.detections = []

    def _depth_cb(self, msg):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().error(f"Depth error: {e}")

    def _odom_cb(self, msg):
        # SLAM entegrasyonunda kullanılacak
        pass

    # ── Ana döngü ─────────────────────────────
    def _loop(self):
        if self.state == State.IDLE:
            return
        elif self.state == State.NAVIGATING:
            self._navigate()
        elif self.state == State.ARRIVED:
            self._publish_status("ARRIVED")

    # ── NAVIGATE ──────────────────────────────
    def _navigate(self):
        target_det = self._find_target()

        if target_det is None:
            self.miss_count += 1
            if self.miss_count >= self.MISS_LIMIT:
                self.get_logger().warn("[NAVIGATOR] Nesne kayboldu → IDLE")
                self._stop()
                self.miss_count = 0
                self.state = State.IDLE
                self._publish_status("LOST")
            return

        self.miss_count = 0

        cx = target_det["cx"]
        cy = target_det["cy"]

        depth = self._depth_at(int(cx), int(cy))
        if depth is None:
            self.get_logger().warn("[NAVIGATOR] Depth okunamadı.")
            return

        self.get_logger().info(f"[NAVIGATOR] depth={depth:.2f}m  cx={cx:.0f}")

        if depth <= STOP_DIST + 0.02:
            self.get_logger().info("[NAVIGATOR] Hedefe ulaşıldı → ARRIVED")
            self._stop()
            self.state = State.ARRIVED
            self._publish_status("ARRIVED")
            return

        # Visual servoing
        twist        = Twist()
        px_err       = (IMAGE_W / 2) - cx
        align_factor = 1.0 - min(abs(px_err) / (IMAGE_W / 2), 1.0)

        twist.angular.z = float(np.clip(KP_ANG * px_err, -MAX_ANG, MAX_ANG))
        twist.linear.x  = float(np.clip(KP_LIN * (depth - STOP_DIST) * align_factor, 0.0, MAX_LIN))

        self.vel_pub.publish(twist)

    # ── Yardımcılar ───────────────────────────
    def _find_target(self):
        """JSON detection listesinden hedef nesneyi bul."""
        if not self.detections:
            return None

        if not self.target:
            # Hedef yoksa en yüksek conf'luyı al
            return max(self.detections, key=lambda d: d.get("conf", 0))

        for det in self.detections:
            if det.get("label") == self.target:
                return det
        return None

    def _depth_at(self, cx, cy, p=5):
        if self.depth_frame is None:
            return None
        h, w = self.depth_frame.shape
        region = self.depth_frame[
            max(0, cy-p):min(h, cy+p),
            max(0, cx-p):min(w, cx+p)
        ]
        valid = region[np.isfinite(region) & (region > 0)]
        return float(np.median(valid)) if len(valid) > 0 else None

    def _stop(self):
        self.vel_pub.publish(Twist())

    def _publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()