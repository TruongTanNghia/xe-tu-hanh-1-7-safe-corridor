#!/usr/bin/env python3

import threading
from pathlib import Path

from cv_bridge import CvBridge
from picamera2 import Picamera2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
import yaml


class HardwareCameraNode(Node):
    def __init__(self):
        super().__init__("hardware_camera")

        self.declare_parameter("width", 640)
        self.declare_parameter("height", 360)
        self.declare_parameter("frame_rate", 30.0)
        self.declare_parameter("sensor_width", 2304)
        self.declare_parameter("sensor_height", 1296)
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("manual_controls", False)
        self.declare_parameter("exposure_time_us", 8000)
        self.declare_parameter("analogue_gain", 1.0)
        self.declare_parameter("colour_gain_r", 1.5)
        self.declare_parameter("colour_gain_b", 1.5)
        self.declare_parameter(
            "calibration_file",
            "",
        )

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.frame_rate = float(self.get_parameter("frame_rate").value)
        self.sensor_width = int(self.get_parameter("sensor_width").value)
        self.sensor_height = int(self.get_parameter("sensor_height").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.manual_controls = bool(self.get_parameter("manual_controls").value)
        self.exposure_time_us = int(self.get_parameter("exposure_time_us").value)
        self.analogue_gain = float(self.get_parameter("analogue_gain").value)
        self.colour_gain_r = float(self.get_parameter("colour_gain_r").value)
        self.colour_gain_b = float(self.get_parameter("colour_gain_b").value)
        calibration_file = Path(
            str(self.get_parameter("calibration_file").value)
        )

        if self.width <= 0 or self.height <= 0 or self.frame_rate <= 0.0:
            raise ValueError("Camera width, height and frame_rate must be positive")
        if not calibration_file.is_file():
            raise FileNotFoundError(
                f"Calibration file does not exist: {calibration_file}"
            )

        self.camera_info = self._load_camera_info(calibration_file)
        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(
            Image,
            "/camera/front_camera/image_raw",
            10,
        )
        self.info_pub = self.create_publisher(
            CameraInfo,
            "/camera/front_camera/camera_info",
            10,
        )

        self.picam = Picamera2()
        controls = {"FrameRate": self.frame_rate}
        if self.manual_controls:
            controls.update(
                {
                    "AeEnable": False,
                    "AwbEnable": False,
                    "ExposureTime": self.exposure_time_us,
                    "AnalogueGain": self.analogue_gain,
                    "ColourGains": (self.colour_gain_r, self.colour_gain_b),
                }
            )

        config = self.picam.create_video_configuration(
            main={
                "size": (self.width, self.height),
                "format": "RGB888",
            },
            sensor={
                "output_size": (self.sensor_width, self.sensor_height),
                "bit_depth": 10,
            },
            controls=controls,
            buffer_count=6,
            queue=False,
        )
        self.picam.configure(config)
        self.picam.start()

        self._frame = None
        self._frame_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            name="imx708_capture",
            daemon=True,
        )
        self._capture_thread.start()
        self._timer = self.create_timer(
            1.0 / self.frame_rate,
            self._publish_frame,
        )

        self.get_logger().info(
            "IMX708 camera started: "
            f"sensor={self.sensor_width}x{self.sensor_height}, "
            f"output={self.width}x{self.height}@{self.frame_rate:.1f} FPS"
        )
        if self.manual_controls:
            self.get_logger().info(
                "Manual camera controls enabled: "
                f"ExposureTime={self.exposure_time_us} us, "
                f"AnalogueGain={self.analogue_gain:.2f}, "
                f"ColourGains=({self.colour_gain_r:.2f}, "
                f"{self.colour_gain_b:.2f})"
            )

    def _load_camera_info(self, calibration_file: Path) -> CameraInfo:
        with calibration_file.open("r", encoding="utf-8") as stream:
            calibration = yaml.safe_load(stream)

        source_width = int(calibration["image_width"])
        source_height = int(calibration["image_height"])
        scale_x = self.width / source_width
        scale_y = self.height / source_height
        source_k = list(calibration["camera_matrix"]["data"])

        message = CameraInfo()
        message.width = self.width
        message.height = self.height
        message.distortion_model = calibration["distortion_model"]
        message.d = list(calibration["distortion_coefficients"]["data"])
        message.k = [
            source_k[0] * scale_x,
            source_k[1],
            source_k[2] * scale_x,
            source_k[3],
            source_k[4] * scale_y,
            source_k[5] * scale_y,
            source_k[6],
            source_k[7],
            source_k[8],
        ]
        message.r = list(calibration["rectification_matrix"]["data"])
        message.p = [
            message.k[0],
            0.0,
            message.k[2],
            0.0,
            0.0,
            message.k[4],
            message.k[5],
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
        ]
        return message

    def _capture_loop(self):
        while not self._stop_event.is_set() and rclpy.ok():
            frame = self.picam.capture_array("main")
            with self._frame_lock:
                self._frame = frame

    def _publish_frame(self):
        with self._frame_lock:
            if self._frame is None:
                return
            frame = self._frame.copy()

        stamp = self.get_clock().now().to_msg()
        image_message = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        image_message.header.stamp = stamp
        image_message.header.frame_id = self.frame_id

        self.camera_info.header.stamp = stamp
        self.camera_info.header.frame_id = self.frame_id
        self.image_pub.publish(image_message)
        self.info_pub.publish(self.camera_info)

    def destroy_node(self):
        self._stop_event.set()
        if hasattr(self, "_capture_thread"):
            self._capture_thread.join(timeout=2.0)
        if hasattr(self, "picam"):
            try:
                self.picam.stop()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HardwareCameraNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
