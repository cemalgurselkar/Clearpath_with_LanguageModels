#!/usr/bin/env python3
"""
Arm Controller Node — TOP-DOWN (tepeden) kavrama
═══════════════════════════════════════════════════════════════════
State Machine:
  IDLE → BENDING → ALIGNING → DESCENDING → GRIPPING → LIFTING → PICKED

BENDING    : REACH_JOINTS'e git (top-down poz). Ulaşınca gripper aç,
             2 sn titreşim sönmesi için bekle → ALIGNING.
ALIGNING   : Look-then-move (dur-ölç) görsel hizalama.
             - pan  (err_x) ile küpü yatayda TARGET_CX'e çek
             - wrist_1 (err_y) ile dikeyde TARGET_CY'ye çek
             - TARGET_CY küpü görüntüde biraz YUKARIDA tutar; çünkü
               gripper kameradan önde, dik inince küpün üstüne oturur.
             - Önceki komut bitmeden ölçüm yok; ALIGN_OK_REQUIRED kez
               üst üste tolerans içinde → DESCENDING.
DESCENDING : Kalibre GRASP pozuna tek trajectory ile DİK İNİŞ.
             - lift/elbow/wrist_1 eşzamanlı interpole olur → düz iniş
               (tek eklem yayı yerine 3 eklem birlikte; gripper dik kalır)
             - pan ALIGNING'den geleni KORUNUR (küp masanın neresinde
               olursa olsun doğru sütuna iner) → küp konumu sabitlenmez.
             - DESCEND_DEPTH altında olduğu için görsel/derinlik kullanmaz;
               iki kalibre poz arası açık döngü interpolasyon.
GRIPPING   : Gripper kapat → LIFTING.
LIFTING    : HOME_JOINTS'e kaldır → PICKED.

KALİBRASYON (16 Haz 2026, yuvarlak masa, top-down):
  REACH (üst poz)  : [0.05, -0.83,  0.861, -1.75,  -1.57, 0.0]
  GRASP (kavrama)  : [0.05, -0.638, 0.958, -1.899, -1.57, 0.0]
  Hizalama hedefi  : cx=321, cy=145   (dikeyde yukarı offset dahil)
  Eksen eşlemesi   : pan→cx (yatay), wrist_1→cy (dikey)  [ölçümle doğrulandı]
═══════════════════════════════════════════════════════════════════
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
import json
from datetime import datetime


class Config:

    NAVIGATOR_TOPIC = "/navigator/status"
    DETECTION_TOPIC = "/detection"
    JOINT_TOPIC     = "/a200_0000/platform/joint_states"
    ARM_TOPIC       = "/a200_0000/arm_0_joint_trajectory_controller/joint_trajectory"
    GRIPPER_ACTION  = "/a200_0000/arm_0_gripper_controller/gripper_cmd"
    STATUS_TOPIC    = "/arm_controller/status"
    FREEZE_TOPIC    = "/detector/freeze"
    LOG_PATH        = "/tmp/arm_controller.log"

    # Joint names — sıra önemli
    JOINT_NAMES = [
        "arm_0_shoulder_pan_joint",   # [0] yatay döndürme (cx kontrol)
        "arm_0_shoulder_lift_joint",  # [1] omuz (iniş)
        "arm_0_elbow_joint",          # [2] dirsek (iniş)
        "arm_0_wrist_1_joint",        # [3] dikey eğim (cy kontrol + iniş telafi)
        "arm_0_wrist_2_joint",        # [4] sabit
        "arm_0_wrist_3_joint",        # [5] sabit
    ]

    # ── KALİBRE EDİLMİŞ POZLAR ──────────────────────────────────
    HOME_JOINTS  = [0.0,  -1.57,  0.65,  -1.57,  -1.57, 0.0]
    REACH_JOINTS = [0.05, -0.83,  0.861, -1.75,  -1.57, 0.0]   # üst/top-down poz
    GRASP_JOINTS = [0.05, -0.638, 0.958, -1.899, -1.57, 0.0]   # kavrama hizası
    # NOT: DESCENDING'de GRASP'ın pan'i (idx 0) KULLANILMAZ;
    #      o an ALIGNING'den gelen pan korunur. Diğerleri GRASP'tan alınır.

    # ── HİZALAMA HEDEFİ ─────────────────────────────────────────
    TARGET_CX = 321   # yatay: tam merkez
    TARGET_CY = 145   # dikey: küpü yukarıda tutar (gripper önde olduğu için)

    BEND_TOL          = 0.05
    ALIGN_TOL         = 8       # piksel
    ALIGN_OK_REQUIRED = 3       # arka arkaya temiz ölçüm şartı
    SETTLE_SEC        = 2.0     # REACH sonrası titreşim sönme beklemesi

    # Hizalama kazançları (look-then-move; top-down için ölçekli)
    KP_PAN   = 0.0008
    KP_WRIST = 0.0015

    ALIGN_SEC   = 2     # her hizalama adımı trajectory süresi
    DESCEND_SEC = 4     # dik iniş trajectory süresi (yumuşak)
    DESCEND_TOL = 0.02  # GRASP'a ulaşma toleransı (rad).
    # 0.08 fazla gevşekti: kol küpe ~birkaç cm varmadan "ulaşıldı" sayıp
    # gripper'ı erken kapatıyordu. 0.02 ile kol GRASP'a gerçekten oturunca
    # kapanır. use_sim_time sayesinde sıkı tolerans artık timeout yaratmaz.
    DESCEND_TIMEOUT = 20.0  # sim saniyesi; iniş takılırsa güvenlik (use_sim_time)

    GRIPPER_OPEN   = 0.0
    GRIPPER_CLOSE  = 0.8
    GRIPPER_EFFORT = 50.0


class State:
    IDLE       = "IDLE"
    BENDING    = "BENDING"
    ALIGNING   = "ALIGNING"
    DESCENDING = "DESCENDING"
    GRIPPING   = "GRIPPING"
    LIFTING    = "LIFTING"
    PICKED     = "PICKED"


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


class StateHandler:
    """Base state handler; ROS arayüzüne node referansı üzerinden erişir."""
    def __init__(self, node: "ArmControllerNode"):
        self.node = node

    def execute(self):
        raise NotImplementedError


class BendingHandler(StateHandler):
    """
    REACH_JOINTS'e ulaşılmasını bekle. Ulaşınca:
      gripper aç → SETTLE_SEC bekle (titreşim sönsün) → ALIGNING.
    """

    def execute(self):
        if not ArmMath.joints_reached(
            self.node.joint_positions, Config.REACH_JOINTS, Config.BEND_TOL
        ):
            return

        # REACH'e ulaşıldı. Henüz bekleme başlamadıysa başlat.
        if not self.node.settling:
            self.node.set_gripper(Config.GRIPPER_OPEN)
            self.node.settling = True
            self.node.flog.log("REACH ulaşıldı → gripper açıldı, settle bekleniyor")
            self.node._settle_timer = self.node.create_timer(
                Config.SETTLE_SEC, self.node._on_settle_done
            )


class AligningHandler(StateHandler):
    """
    Look-then-move görsel hizalama:
      pan  ← err_x = cx - TARGET_CX   (yatay)
      wrist_1 ← err_y = cy - TARGET_CY (dikey)

    Dur-ölç: önceki komut bitmeden ölçüm yapma.
    ALIGN_OK_REQUIRED kez üst üste tolerans içinde → DESCENDING.
    """

    def execute(self):
        # Önceki hizalama komutu hâlâ uygulanıyorsa bekle
        if self.node.align_target is not None:
            if not ArmMath.joints_reached(
                self.node.joint_positions, self.node.align_target, Config.BEND_TOL
            ):
                return
            self.node.align_target = None

        det = ArmMath.best_detection(self.node.detections)
        if det is None:
            return

        cx, cy = int(det["cx"]), int(det["cy"])
        err_x  = cx - Config.TARGET_CX
        err_y  = cy - Config.TARGET_CY

        self.node.flog.log(
            f"ALIGNING cx={cx} cy={cy} err_x={err_x} err_y={err_y} "
            f"ok={self.node.align_ok_count}"
        )
        self.node.get_logger().info(f"ALIGNING err_x={err_x} err_y={err_y}")

        # Debounce: arka arkaya temiz ölçüm
        if abs(err_x) < Config.ALIGN_TOL and abs(err_y) < Config.ALIGN_TOL:
            self.node.align_ok_count += 1
            if self.node.align_ok_count >= Config.ALIGN_OK_REQUIRED:
                self.node.align_ok_count = 0
                self.node.flog.log("ALIGN OK (debounced) → DESCENDING")
                self.node.start_descent()
            return
        else:
            self.node.align_ok_count = 0

        # Düzeltme komutu (eksen eşlemesi ölçümle doğrulandı)
        joints     = self.node.joint_positions.copy()
        joints[0] -= Config.KP_PAN   * err_x   # shoulder_pan  → yatay
        joints[3] += Config.KP_WRIST * err_y   # wrist_1       → dikey
        self.node.align_target = joints[:]
        self.node.send_arm(joints, sec=Config.ALIGN_SEC)


class DescendingHandler(StateHandler):
    """
    Dik iniş izleme: kol GRASP hedefine ULAŞANA kadar bekler.
    Süre tahminine güvenmez — joints_reached ile gerçek varışı kontrol eder.
    - Hedefe ulaşılınca → GRIPPING (kavra).
    - DESCEND_TIMEOUT aşılırsa (kol bir yere takıldıysa) → yine GRIPPING,
      ama loga 'timeout' yazılır ki teşhis edilebilsin.
    """

    def execute(self):
        target = self.node.descent_target
        if target is None:
            return

        reached = ArmMath.joints_reached(
            self.node.joint_positions, target, Config.DESCEND_TOL
        )
        # Sim-time uyumlu süre ölçümü (RTF düşük olsa bile doğru)
        elapsed = (self.node.get_clock().now() - self.node.descent_start).nanoseconds / 1e9

        if reached:
            self.node.flog.log(
                f"DESCENDING tamam (ulaşıldı, {elapsed:.1f}s) → GRIPPING"
            )
            self.node.descent_target = None
            self.node.transition(State.GRIPPING)
            self.node.start_grip_sequence()
            return

        if elapsed >= Config.DESCEND_TIMEOUT:
            # Hangi eklem(ler) hedefe ulaşamadı, farkları yaz
            diffs = [
                f"{Config.JOINT_NAMES[i].replace('arm_0_','').replace('_joint','')}="
                f"{self.node.joint_positions[i] - target[i]:+.3f}"
                for i in range(len(target))
            ]
            self.node.flog.log(
                f"DESCENDING TIMEOUT ({elapsed:.1f}s) farklar: {' '.join(diffs)} → GRIPPING"
            )
            self.node.get_logger().warn("DESCENDING timeout — yine de kavranıyor")
            self.node.descent_target = None
            self.node.transition(State.GRIPPING)
            self.node.start_grip_sequence()


class ArmControllerNode(Node):

    def __init__(self):
        super().__init__("arm_controller")

        # Simülasyon saatini kullan: RTF düşük olduğunda (sim gerçek zamandan
        # yavaş) tüm zamanlayıcılar ve süre ölçümleri sim saatine göre çalışır.
        # Aksi halde gerçek-zaman timeout'ları kol daha inerken dolup
        # erken kavramaya yol açıyordu.
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # State
        self.state           = State.IDLE
        self.detections      = []
        self.joint_positions = Config.HOME_JOINTS.copy()

        # BENDING settle
        self.settling      = False
        self._settle_timer = None

        # ALIGNING dur-ölç + debounce
        self.align_target   = None
        self.align_ok_count = 0

        # DESCENDING izleme
        self.descent_target = None
        self.descent_start  = None

        self.flog = FileLogger(Config.LOG_PATH)

        self._handlers = {
            State.BENDING:    BendingHandler(self),
            State.ALIGNING:   AligningHandler(self),
            State.DESCENDING: DescendingHandler(self),
        }

        self.arm_pub        = self.create_publisher(JointTrajectory, Config.ARM_TOPIC, 10)
        self.status_pub     = self.create_publisher(String, Config.STATUS_TOPIC, 10)
        self.freeze_pub     = self.create_publisher(String, Config.FREEZE_TOPIC, 10)
        self.gripper_client = ActionClient(self, GripperCommand, Config.GRIPPER_ACTION)

        self.create_subscription(String,     Config.NAVIGATOR_TOPIC, self._on_navigator, 10)
        self.create_subscription(String,     Config.DETECTION_TOPIC, self._on_detection, 10)
        self.create_subscription(JointState, Config.JOINT_TOPIC,     self._on_joints,    10)

        self.create_timer(0.2, self._loop)
        self.flog.log("ArmController ready → IDLE")
        self.get_logger().info(f"ArmController ready | log: {Config.LOG_PATH}")

    # ── Aboneler ────────────────────────────────────────────────
    def _on_navigator(self, msg: String):
        if msg.data == "ARRIVED" and self.state == State.IDLE:
            self.send_arm(Config.REACH_JOINTS, sec=3)
            self.transition(State.BENDING)

    def _on_detection(self, msg: String):
        try:
            self.detections = json.loads(msg.data)
        except Exception:
            self.detections = []

    def _on_joints(self, msg: JointState):
        joint_map            = dict(zip(msg.name, msg.position))
        self.joint_positions = [joint_map.get(j, 0.0) for j in Config.JOINT_NAMES]

    # ── Ana döngü ───────────────────────────────────────────────
    def _loop(self):
        handler = self._handlers.get(self.state)
        if handler:
            handler.execute()

    # ── BENDING settle tamamlandı ───────────────────────────────
    def _on_settle_done(self):
        if self._settle_timer:
            self._settle_timer.cancel()
            self._settle_timer = None
        if self.state == State.BENDING:
            self.flog.log("Settle bitti → ALIGNING")
            self.transition(State.ALIGNING)

    # ── DESCENDING: dik iniş (interpolasyon) ────────────────────
    def start_descent(self):
        """
        GRASP pozuna in, ama pan'i ALIGNING'den geldiği gibi koru.
        lift/elbow/wrist_1/wrist_2/wrist_3 = GRASP, pan = mevcut.
        Tek trajectory → 3 eklem eşzamanlı → düz dik iniş.
        Kavrama, süreyle değil, GRASP'a ULAŞMAYLA tetiklenir (DescendingHandler).
        """
        self.transition(State.DESCENDING)
        self.freeze_pub.publish(String(data="True"))

        target    = list(Config.GRASP_JOINTS)
        target[0] = self.joint_positions[0]   # pan korunur (yatay hizalama)
        self.descent_target = target
        self.descent_start  = self.get_clock().now()
        self.send_arm(target, sec=Config.DESCEND_SEC)
        self.flog.log(f"DESCENDING → {['%.3f' % p for p in target]}")

    # ── GRIPPING → LIFTING → PICKED ─────────────────────────────
    def start_grip_sequence(self):
        self.set_gripper(Config.GRIPPER_CLOSE)
        self.flog.log("GRIPPING: gripper kapatılıyor")
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

    # ── Yardımcılar ─────────────────────────────────────────────
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