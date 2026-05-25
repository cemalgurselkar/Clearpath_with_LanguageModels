#!/usr/bin/env python3
"""
Detector Node
Subscribes /camera_view → runs YOLOv8 → publishes /detection as JSON
Supports /detector/freeze to lock last detection
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import json


# ── Config ─────────────────────────────────────────────────────────────────────

class Config:
    CAMERA_TOPIC    = "/camera_view"
    DETECTION_TOPIC = "/detection"
    FREEZE_TOPIC    = "/detector/freeze"
    MODEL_PATH      = "/home/cemal/ros2_ws/src/llm_robot/llm_robot/best.pt"
    CONF_THRESHOLD  = 0.2
    DISPLAY_HZ      = 0.033


# ── DetectionHelper ────────────────────────────────────────────────────────────

class DetectionHelper:

    @staticmethod
    def parse_boxes(results, model) -> list:
        detections = []
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                detections.append({
                    "label": model.names[int(box.cls[0])],
                    "conf":  round(float(box.conf[0]), 2),
                    "cx":    float((x1 + x2) / 2),
                    "cy":    float((y1 + y2) / 2),
                    "w":     float(x2 - x1),
                    "h":     float(y2 - y1),
                })
        return detections

    @staticmethod
    def draw_boxes(frame, results, model):
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                label = model.names[int(box.cls[0])]
                conf  = float(box.conf[0])
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {conf:.2f}",
                            (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        return frame


# ── Node ───────────────────────────────────────────────────────────────────────

class DetectorNode(Node):

    def __init__(self):
        super().__init__("detector")

        self.bridge      = CvBridge()
        self.color_frame = None
        self.frozen      = False
        self.frozen_msg  = None

        self.get_logger().info(f"Loading model: {Config.MODEL_PATH}")
        self.model = YOLO(Config.MODEL_PATH)
        self.get_logger().info("Model ready.")

        self.detection_pub = self.create_publisher(String, Config.DETECTION_TOPIC, 10)

        self.create_subscription(Image,  Config.CAMERA_TOPIC, self._on_image,  10)
        self.create_subscription(String, Config.FREEZE_TOPIC, self._on_freeze, 10)

        self.create_timer(Config.DISPLAY_HZ, self._display)

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def _on_image(self, msg):
        try:
            self.color_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self._detect()
        except Exception as e:
            self.get_logger().error(str(e))

    def _on_freeze(self, msg):
        self.frozen = msg.data == "True"
        self.get_logger().info(f"Freeze: {self.frozen}")

    # ── Detection ──────────────────────────────────────────────────────────────

    def _detect(self):
        if self.color_frame is None:
            return

        if self.frozen:
            if self.frozen_msg:
                self.detection_pub.publish(self.frozen_msg)
            return

        results    = self.model(self.color_frame, verbose=False, conf=Config.CONF_THRESHOLD)
        detections = DetectionHelper.parse_boxes(results, self.model)

        out      = String()
        out.data = json.dumps(detections)
        self.frozen_msg = out
        self.detection_pub.publish(out)

    # ── Display ────────────────────────────────────────────────────────────────

    def _display(self):
        if self.color_frame is None:
            return
        frame   = self.color_frame.copy()
        results = self.model(frame, verbose=False, conf=Config.CONF_THRESHOLD)
        frame   = DetectionHelper.draw_boxes(frame, results, self.model)
        cv2.imshow("Detector", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            rclpy.shutdown()


# ── Entry ──────────────────────────────────────────────────────────────────────

def main():
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()