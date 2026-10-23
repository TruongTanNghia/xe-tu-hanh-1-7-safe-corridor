#!/usr/bin/env python3

from pathlib import Path

from cv_bridge import CvBridge
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class SaveImageNode(Node):
    def __init__(self):
        super().__init__("save_image")
        self.declare_parameter("topic", "/line/fitting_image")
        self.declare_parameter("output_path", "/tmp/lane_fitting.jpg")
        self.topic = str(self.get_parameter("topic").value)
        self.output_path = Path(str(self.get_parameter("output_path").value))
        self.bridge = CvBridge()
        self.saved = False
        self.subscription = self.create_subscription(
            Image,
            self.topic,
            self._callback,
            10,
        )

    def _callback(self, message):
        if self.saved:
            return
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(self.output_path), image):
            raise RuntimeError(f"Cannot write image: {self.output_path}")
        self.saved = True
        self.get_logger().info(f"Saved {self.topic} to {self.output_path}")


def main(args=None):
    rclpy.init(args=args)
    node = SaveImageNode()
    try:
        while rclpy.ok() and not node.saved:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
