#!/usr/bin/env python3
"""
scanner.py - Arm Scanner Node
- Subscribes to /llm_task → hangi nesneyi arayacağını öğrenir
- Subscribes to /detection → JSON formatında detection listesi
- shoulder_pan_joint ile sağa sola tarar
- Nesne bulununca arm kilitler
- /scanner_status publish eder → SCANNING / FOUND / LOCKED
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import json

ARM_TOPIC       = "/a200_0000/arm_0_joint_trajectory_controller/joint_trajectory"
DETECTION_TOPIC = "/detection"
LLM_TASK_TOPIC  = "/llm_task"
STATUS_TOPIC    = "/scanner_status"

JOINT_NAMES = [
    "arm_0_shoulder_pan_joint",
    "arm_0_shoulder_lift_joint",
    "arm_0_elbow_joint",
    "arm_0_wrist_1_joint",
    "arm_0_wrist_2_joint",
    "arm_0_wrist_3_joint",
]

HOME_JOINTS = [0.0, -1.57, 0.0, -1.57, -1.57, 0.0]

SCAN_MIN    = -3.5
SCAN_MAX    =  3.5
SCAN_STEP   =  0.05
SCAN_PERIOD =  0.15


class State:
    IDLE     = "IDLE"
    SCANNING = "SCANNING"
    FOUND    = "FOUND"
    LOCKED   = "LOCKED"


class ScannerNode(Node):
    def __init__(self):
        super().__init__("scanner")

        self.state      = State.IDLE
        self.target     = None
        self.scan_angle = 0.0
        self.scan_dir   = 1
        self.detections = []  # JSON list

        # Publishers
        self.arm_pub    = self.create_publisher(JointTrajectory, ARM_TOPIC, 10)
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)

        # Subscribers
        self.create_subscription(String, LLM_TASK_TOPIC, self._llm_cb, 10)
        self.create_subscription(String, DETECTION_TOPIC, self._detection_cb, 10)

        self.create_timer(SCAN_PERIOD, self._loop)

        self.get_logger().info("Scanner hazır → IDLE")
        self.get_logger().info(f"Hedef bekleniyor: {LLM_TASK_TOPIC}")

    # ── Callbacks ─────────────────────────────
    def _llm_cb(self, msg):
        self.target     = msg.data.strip()
        self.state      = State.SCANNING
        self.scan_angle = 0.0
        self.scan_dir   = 1
        self.get_logger().info(f"[LLM] Hedef: {self.target} → SCANNING")
        self._publish_status("SCANNING")

    def _detection_cb(self, msg):
        try:
            self.detections = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Detection parse error: {e}")
            self.detections = []

    # ── Ana döngü ─────────────────────────────
    def _loop(self):
        if self.state == State.IDLE:
            return
        elif self.state == State.SCANNING:
            self._scan()
        elif self.state == State.FOUND:
            self._lock()
            self._publish_status("ARRIVED")

    # ── SCAN ──────────────────────────────────
    def _scan(self):
        self.scan_angle += self.scan_dir * SCAN_STEP
        if self.scan_angle >= SCAN_MAX:
            self.scan_angle = SCAN_MAX
            self.scan_dir = -1
        elif self.scan_angle <= SCAN_MIN:
            self.scan_angle = SCAN_MIN
            self.scan_dir = 1

        joints = HOME_JOINTS.copy()
        joints[0] = self.scan_angle
        self._send_arm(joints, sec=1)

        if self._find_target():
            self.get_logger().info(f"[SCAN] {self.target} bulundu! → FOUND")
            self.state = State.FOUND
            self._publish_status(f"FOUND:{self.target}")

    # ── LOCK ──────────────────────────────────
    def _lock(self):
        lock_joints = HOME_JOINTS.copy()
        lock_joints[0] = self.scan_angle
        self._send_arm(lock_joints, sec=2)
        self.state = State.LOCKED
        self.get_logger().info(f"[LOCK] Arm kilitlendi. Açı: {self.scan_angle:.2f} rad")
        self._publish_status("LOCKED")

    # ── Yardımcılar ───────────────────────────
    def _find_target(self):
        if not self.target:
            return False
        for det in self.detections:
            if det.get("label") == self.target:
                return True
        return False

    def _send_arm(self, positions, sec=2):
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        pt.time_from_start = Duration(sec=sec)
        msg.points = [pt]
        self.arm_pub.publish(msg)

    def _publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = ScannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()