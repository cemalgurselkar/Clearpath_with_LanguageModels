#!/usr/bin/env python3
"""
camera.py - Wrist Camera Node
- Streams an RGB camera feed. 
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime

class CameraViewer(Node):
    def __init__(self):
        super().__init__("camera_viewer")
        self.bridge      = CvBridge()
        self.color_frame = None
        self.frame_count = 0
        self.color_topic = "/a200_0000/sensors/camera_0/color/image"
        self.publisher_name = "camera_view"
        self.data_dir = os.path.expanduser("~/ros2_ws/src/llm_robot/data2")
        os.makedirs(self.data_dir, exist_ok=True)

        self.create_subscription(Image, self.color_topic, self._color_cb, 10)
        self.pub = self.create_publisher(Image, self.publisher_name, 10)
        self.create_timer(0.033, self._display)

        self.get_logger().info(f"Subscribing: {self.color_topic}")
        self.get_logger().info(f"Publishing: {self.publisher_name}")

    def _color_cb(self, msg):
        try:
            self.color_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Kamera hatası: {e}")

    def _display(self):
        if self.color_frame is None:
            return

        frame = self.color_frame.copy()

        cv2.putText(frame, f"Kaydedilen: {self.frame_count}",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, "'s' kaydet | 'q' cikis",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
 
        cv2.imshow("Camera", frame)
        key = cv2.waitKey(1) & 0xFF
 
        if key == ord("s"):
            self._save_frame()
        elif key == ord("q"):
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def _save_frame(self):
        if self.color_frame is None:
            return
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename  = os.path.join(self.data_dir, f"frame_{timestamp}.jpg")
        cv2.imwrite(filename, self.color_frame)
        self.frame_count += 1
        self.get_logger().info(f"[{self.frame_count}] Kaydedildi: {filename}")

def main():
    rclpy.init()
    node = CameraViewer()
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