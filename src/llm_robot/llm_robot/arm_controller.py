#!/usr/bin/env python3
"""
Arm Controller Node
═══════════════════════════════════════════════════════════════════
State Machine:
  IDLE → BENDING → ALIGNING → PUSHING → GRIPPING → LIFTING → PICKED

BENDING  : REACH_JOINTS'e git, joint_states ile ulaşmayı bekle
ALIGNING : shoulder_pan (err_x) + wrist_1 (err_y) ile cx/cy merkeze çek
PUSHING  : Hibrit kontrol:
           - depth > OPEN_LOOP_DEPTH  → closed-loop (depth ile adım at)
           - depth <= OPEN_LOOP_DEPTH → open-loop (sabit adım sayısı)
           - open-loop biter          → GRIPPING
GRIPPING : gripper kapat → HOME_JOINTS → PICKED
═══════════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from cv_bridge import CvBridge
import numpy as np
import json
from datetime import datetime


class Config:

    NAVIGATOR_TOPIC = "/navigator/status"
    DETECTION_TOPIC = "/detection"
    DEPTH_TOPIC     = "/a200_0000/sensors/camera_0/depth/image"
    JOINT_TOPIC     = "/a200_0000/platform/joint_states"
    ARM_TOPIC       = "/a200_0000/arm_0_joint_trajectory_controller/joint_trajectory"
    GRIPPER_ACTION  = "/a200_0000/arm_0_gripper_controller/gripper_cmd"
    STATUS_TOPIC    = "/arm_controller/status"
    FREEZE_TOPIC    = "/detector/freeze"
    LOG_PATH        = "/tmp/arm_controller.log"

    # Joint names — sıra önemli
    JOINT_NAMES = [
        "arm_0_shoulder_pan_joint",   # [0] yatay döndürme
        "arm_0_shoulder_lift_joint",  # [1] öne/geri uzanma
        "arm_0_elbow_joint",          # [2]
        "arm_0_wrist_1_joint",        # [3] dikey eğim
        "arm_0_wrist_2_joint",        # [4]
        "arm_0_wrist_3_joint",        # [5]
    ]

    HOME_JOINTS  = [0.0, -1.57, 0.0, -2.0, -1.57, 0.0]
    REACH_JOINTS = [0.0, -1.32, 0.33, -2.0, -1.57, 0.0]

    IMAGE_CX = 320
    IMAGE_CY = 240


    BEND_TOL        = 0.05
    ALIGN_TOL       = 20

    OPEN_LOOP_DEPTH = 0.30
    OPEN_LOOP_STEPS = 15

    # KP parametreleri
    KP_PAN   = 0.001
    KP_WRIST = 0.003
    KP_LIFT  = 0.05

    GRIPPER_OPEN   = 0.0
    GRIPPER_CLOSE  = 0.8
    GRIPPER_EFFORT = 50.0

class State:
    IDLE     = "IDLE"
    BENDING  = "BENDING"
    ALIGNING = "ALIGNING"
    PUSHING  = "PUSHING"
    GRIPPING = "GRIPPING"
    LIFTING  = "LIFTING"
    PICKED   = "PICKED"


class FileLogger:
    def __init__(self, path: str):
        self.path = path
        with open(path, "w") as f:
            f.write(f"=== arm_controller log {datetime.now()} ===\n")

    def log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        with open(self.path, "a") as f:
            f.write(f"[{ts}] {msg}\n")


class ArmMath:

    @staticmethod
    def build_trajectory(positions: list, sec: int = 2) -> JointTrajectory:
        msg                = JointTrajectory()
        msg.joint_names    = Config.JOINT_NAMES
        pt                 = JointTrajectoryPoint()
        pt.positions       = [float(p) for p in positions]
        pt.time_from_start = Duration(sec=sec)
        msg.points         = [pt]
        return msg

    @staticmethod
    def joints_reached(current: list, target: list, tol: float) -> bool:
        return all(abs(c - g) <= tol for c, g in zip(current, target))

    @staticmethod
    def best_detection(detections: list) -> dict | None:
        if not detections:
            return None
        return max(detections, key=lambda d: d.get("conf", 0))

    @staticmethod
    def depth_at(frame, cx: int, cy: int, patch: int = 5) -> float | None:
        """cx, cy pikselindeki medyan depth değerini döndür."""
        if frame is None:
            return None
        h, w   = frame.shape
        region = frame[
            max(0, cy - patch):min(h, cy + patch),
            max(0, cx - patch):min(w, cx + patch),
        ]
        valid = region[np.isfinite(region) & (region > 0)]
        return float(np.median(valid)) if len(valid) else None


class StateHandler:
    def __init__(self, node: "ArmControllerNode"):
        self.node = node

    def execute(self):
        raise NotImplementedError


class BendingHandler(StateHandler):
    """
    REACH_JOINTS'e komut gönderildi.
    joint_states ile ulaşılıp ulaşılmadığını kontrol et.
    Ulaşınca gripper aç → ALIGNING.
    """

    def execute(self):
        if ArmMath.joints_reached(
            self.node.joint_positions, Config.REACH_JOINTS, Config.BEND_TOL
        ):
            self.node.set_gripper(Config.GRIPPER_OPEN)
            self.node.transition(State.ALIGNING)


class AligningHandler(StateHandler):
    """
    shoulder_pan → err_x sıfırla (yatay)
    wrist_1      → err_y sıfırla (dikey)
    Her ikisi ALIGN_TOL içine girince detection dondur → PUSHING.
    """

    def execute(self):
        det = ArmMath.best_detection(self.node.detections)
        if det is None:
            return

        cx, cy = int(det["cx"]), int(det["cy"])
        err_x  = cx - Config.IMAGE_CX
        err_y  = cy - Config.IMAGE_CY

        joints    = self.node.joint_positions.copy()
        joints[0] -= Config.KP_PAN   * err_x   # shoulder_pan
        joints[3] += Config.KP_WRIST * err_y   # wrist_1
        self.node.send_arm(joints, sec=1)

        self.node.flog.log(f"ALIGNING err_x={err_x} err_y={err_y}")
        self.node.get_logger().info(f"ALIGNING err_x={err_x} err_y={err_y}")

        if abs(err_x) < Config.ALIGN_TOL and abs(err_y) < Config.ALIGN_TOL:
            self.node.freeze_pub.publish(String(data="True"))
            self.node.transition(State.PUSHING)


class PushingHandler(StateHandler):
    """
    Hibrit kontrol:
    - depth > OPEN_LOOP_DEPTH  → closed-loop: depth okuyarak adım at
    - depth <= OPEN_LOOP_DEPTH → open-loop: sabit OPEN_LOOP_STEPS adım at
    - depth None (NaN)         → open-loop'a geç (kamera körleşti)
    - open-loop biter          → GRIPPING
    """

    def execute(self):
        if self.node.open_loop_steps > 0:
            if self.node.push_target is None or ArmMath.joints_reached(
                self.node.joint_positions, self.node.push_target, Config.BEND_TOL
            ):
                self.node.open_loop_steps -= 1
                self.node.flog.log(
                    f"OPEN-LOOP step, kalan={self.node.open_loop_steps} "
                    f"lift={self.node.joint_positions[1]:.3f}"
                )

                if self.node.open_loop_steps == 0:
                    self.node.flog.log("OPEN-LOOP done → GRIPPING")
                    self.node.transition(State.GRIPPING)
                    self.node.start_grip_sequence()
                    return

                joints    = self.node.joint_positions.copy()
                joints[1] += Config.KP_LIFT
                self.node.push_target = joints[:]
                self.node.send_arm(joints, sec=1)
            return

        det = ArmMath.best_detection(self.node.detections)
        if det is None:
            return

        cx, cy = int(det["cx"]), int(det["cy"])
        depth  = ArmMath.depth_at(self.node.depth_frame, cx, cy)

        if depth is None or depth <= Config.OPEN_LOOP_DEPTH:
            self.node.open_loop_steps = Config.OPEN_LOOP_STEPS
            self.node.push_target     = None
            self.node.flog.log(
                f"CLOSED→OPEN-LOOP geçiş "
                f"depth={'NaN' if depth is None else f'{depth:.3f}m'}"
            )
            self.node.get_logger().info("OPEN-LOOP moda geçildi")
            return

        self.node.flog.log(f"PUSHING depth={depth:.3f}m  lift={self.node.joint_positions[1]:.3f}")
        self.node.get_logger().info(f"PUSHING depth={depth:.3f}m")

        if self.node.push_target is not None:
            if not ArmMath.joints_reached(
                self.node.joint_positions, self.node.push_target, Config.BEND_TOL):
                return

        joints    = self.node.joint_positions.copy()
        joints[1] += Config.KP_LIFT
        self.node.push_target = joints[:]
        self.node.send_arm(joints, sec=1)
        self.node.flog.log(f"PUSH step → lift={joints[1]:.3f}")

class ArmControllerNode(Node):

    def __init__(self):
        super().__init__("arm_controller")

        # State
        self.state           = State.IDLE
        self.detections      = []
        self.depth_frame     = None
        self.joint_positions = Config.HOME_JOINTS.copy()
        self.push_target     = None
        self.open_loop_steps = 0

        self.bridge = CvBridge()
        self.flog   = FileLogger(Config.LOG_PATH)

        self._handlers = {
            State.BENDING:  BendingHandler(self),
            State.ALIGNING: AligningHandler(self),
            State.PUSHING:  PushingHandler(self),
        }

        self.arm_pub        = self.create_publisher(JointTrajectory, Config.ARM_TOPIC, 10)
        self.status_pub     = self.create_publisher(String, Config.STATUS_TOPIC, 10)
        self.freeze_pub     = self.create_publisher(String, Config.FREEZE_TOPIC, 10)
        self.gripper_client = ActionClient(self, GripperCommand, Config.GRIPPER_ACTION)

        self.create_subscription(String,     Config.NAVIGATOR_TOPIC, self._on_navigator, 10)
        self.create_subscription(String,     Config.DETECTION_TOPIC, self._on_detection, 10)
        self.create_subscription(Image,      Config.DEPTH_TOPIC,     self._on_depth,     10)
        self.create_subscription(JointState, Config.JOINT_TOPIC,     self._on_joints,    10)

        self.create_timer(0.2, self._loop)
        self.flog.log("ArmController ready → IDLE")
        self.get_logger().info(f"ArmController ready | log: {Config.LOG_PATH}")

    def _on_navigator(self, msg: String):
        if msg.data == "ARRIVED" and self.state == State.IDLE:
            self.send_arm(Config.REACH_JOINTS, sec=3)
            self.transition(State.BENDING)

    def _on_detection(self, msg: String):
        try:
            self.detections = json.loads(msg.data)
        except Exception:
            self.detections = []

    def _on_depth(self, msg: Image):
        try:
            self.depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().error(str(e))

    def _on_joints(self, msg: JointState):
        joint_map            = dict(zip(msg.name, msg.position))
        self.joint_positions = [joint_map.get(j, 0.0) for j in Config.JOINT_NAMES]

    def _loop(self):
        handler = self._handlers.get(self.state)
        if handler:
            handler.execute()

    def start_grip_sequence(self):
        self.set_gripper(Config.GRIPPER_CLOSE)
        t = self.create_timer(2.0, lambda: self._lift(t))

    def _lift(self, timer):
        timer.cancel()
        if self.state != State.GRIPPING:
            return
        self.transition(State.LIFTING)
        self.send_arm(Config.HOME_JOINTS, sec=3)
        t = self.create_timer(3.0, lambda: self._done(t))

    def _done(self, timer):
        timer.cancel()
        if self.state != State.LIFTING:
            return
        self.transition(State.PICKED)
        self.publish_status("PICKED")
        self.freeze_pub.publish(String(data="False"))
        self.flog.log("PICKED — mission complete")

    def transition(self, new_state: str):
        self.flog.log(f"{self.state} → {new_state}")
        self.get_logger().info(f"{self.state} → {new_state}")
        self.state = new_state

    def send_arm(self, positions: list, sec: int = 2):
        self.arm_pub.publish(ArmMath.build_trajectory(positions, sec))

    def set_gripper(self, position: float):
        goal                    = GripperCommand.Goal()
        goal.command.position   = position
        goal.command.max_effort = Config.GRIPPER_EFFORT
        self.gripper_client.wait_for_server()
        self.gripper_client.send_goal_async(goal)

    def publish_status(self, status: str):
        msg      = String()
        msg.data = status
        self.status_pub.publish(msg)

def main():
    rclpy.init()
    node = ArmControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()