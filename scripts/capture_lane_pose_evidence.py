#!/usr/bin/env python3

import argparse
import json
import re
import select
import sys
import termios
import threading
import time
import tty
from datetime import datetime
from pathlib import Path

from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64, Float64MultiArray, String


ROI_TOP_RATIO = 0.23
ROI_BOTTOM_RATIO = 0.46


def safe_name(text):
    cleaned = re.sub(r"[^A-Za-z0-9_-]+", "_", text.strip())
    return cleaned.strip("_") or "snapshot"


def stamp_to_dict(stamp):
    return {
        "sec": int(stamp.sec),
        "nanosec": int(stamp.nanosec),
        "time_sec": float(stamp.sec) + float(stamp.nanosec) * 1e-9,
    }


def now_label():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


class LanePoseEvidence(Node):
    def __init__(self, output_dir, labels, jpeg_quality):
        super().__init__("lane_pose_evidence_capture")
        self.output_dir = Path(output_dir)
        self.labels = labels
        self.jpeg_quality = int(jpeg_quality)
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.images = {}
        self.image_headers = {}
        self.values = {}
        self.capture_index = 0

        self.create_subscription(
            Image,
            "/camera/front_camera/image_raw",
            lambda msg: self._image_callback("raw", msg),
            10,
        )
        self.create_subscription(
            Image,
            "/line/debug_image",
            lambda msg: self._image_callback("debug", msg),
            10,
        )
        self.create_subscription(
            Image,
            "/line/fitting_image",
            lambda msg: self._image_callback("fitting", msg),
            10,
        )

        self.create_subscription(
            CameraInfo,
            "/camera/front_camera/camera_info",
            self._camera_info_callback,
            10,
        )
        self.create_subscription(
            String,
            "/line/status",
            lambda msg: self._store_value("/line/status", msg.data),
            10,
        )
        for topic in (
            "/line/confidence",
            "/line/cte",
            "/line/heading_angle",
            "/line/heading_error",
            "/line/kappa",
            "/line/evaluation_forward",
        ):
            self.create_subscription(
                Float64,
                topic,
                lambda msg, topic=topic: self._store_value(topic, float(msg.data)),
                10,
            )
        for topic in (
            "/line/reference_path",
            "/line/poly_coeffs",
            "/line/s_metric",
            "/line/kappa_s",
        ):
            self.create_subscription(
                Float64MultiArray,
                topic,
                lambda msg, topic=topic: self._store_value(
                    topic,
                    [float(value) for value in msg.data],
                ),
                10,
            )

    def _image_callback(self, name, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:
            self.get_logger().error(f"Cannot decode {name}: {error}")
            return
        with self.lock:
            self.images[name] = image.copy()
            self.image_headers[name] = {
                "frame_id": message.header.frame_id,
                "stamp": stamp_to_dict(message.header.stamp),
                "encoding": message.encoding,
                "width": int(message.width),
                "height": int(message.height),
            }

    def _camera_info_callback(self, message):
        payload = {
            "header": {
                "frame_id": message.header.frame_id,
                "stamp": stamp_to_dict(message.header.stamp),
            },
            "width": int(message.width),
            "height": int(message.height),
            "distortion_model": message.distortion_model,
            "d": [float(value) for value in message.d],
            "k": [float(value) for value in message.k],
            "r": [float(value) for value in message.r],
            "p": [float(value) for value in message.p],
        }
        self._store_value("/camera/front_camera/camera_info", payload)

    def _store_value(self, topic, value):
        with self.lock:
            self.values[topic] = {
                "received_wall_time": time.time(),
                "value": value,
            }

    def latest_status_summary(self):
        with self.lock:
            status = self.values.get("/line/status", {}).get("value", "missing")
            confidence = self.values.get("/line/confidence", {}).get("value", None)
            cte = self.values.get("/line/cte", {}).get("value", None)
            heading = self.values.get("/line/heading_angle", {}).get("value", None)
            images = sorted(self.images.keys())
        return status, confidence, cte, heading, images

    def capture_next(self):
        if self.capture_index < len(self.labels):
            label = self.labels[self.capture_index]
        else:
            label = f"extra_{self.capture_index + 1:02d}"
        self.capture_index += 1
        return self.capture(label)

    def capture(self, label):
        with self.lock:
            images = {name: image.copy() for name, image in self.images.items()}
            image_headers = dict(self.image_headers)
            values = json.loads(json.dumps(self.values))

        timestamp = now_label()
        folder = self.output_dir / f"{self.capture_index:02d}_{timestamp}_{safe_name(label)}"
        folder.mkdir(parents=True, exist_ok=True)

        for name, image in images.items():
            self._write_jpeg(folder / f"{name}.jpg", image)

        panel = self._build_panel(images)
        self._write_jpeg(folder / "panel.jpg", panel)

        telemetry = {
            "label": label,
            "capture_index": self.capture_index,
            "capture_wall_time": time.time(),
            "capture_local_time": datetime.now().isoformat(timespec="milliseconds"),
            "topics": values,
            "image_headers": image_headers,
        }
        (folder / "telemetry.json").write_text(
            json.dumps(telemetry, indent=2, sort_keys=True),
            encoding="utf-8",
        )
        (folder / "echo.txt").write_text(self._format_echo(values), encoding="utf-8")

        return folder

    def _write_jpeg(self, path, image):
        if image is None or image.size == 0:
            return
        params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        if not cv2.imwrite(str(path), image, params):
            raise RuntimeError(f"Cannot write image: {path}")

    def _build_panel(self, images):
        raw = images.get("raw")
        debug = images.get("debug")
        fitting = images.get("fitting")
        roi = None
        if raw is not None and raw.size > 0:
            top = int(raw.shape[0] * ROI_TOP_RATIO)
            bottom = int(raw.shape[0] * ROI_BOTTOM_RATIO)
            roi = raw[top:bottom, :]
        roi_label = f"Processor ROI {ROI_TOP_RATIO * 100:.0f}-{ROI_BOTTOM_RATIO * 100:.0f}%"
        return np.vstack(
            [
                np.hstack(
                    [
                        self._panel(raw, "Raw Camera"),
                        self._panel(roi, roi_label),
                    ]
                ),
                np.hstack(
                    [
                        self._panel(debug, "Debug (scan bands)"),
                        self._panel(fitting, "Poly Fitting"),
                    ]
                ),
            ]
        )

    @staticmethod
    def _panel(image, label):
        width, height = 480, 270
        if image is None or image.size == 0:
            panel = np.full((height, width, 3), 35, dtype=np.uint8)
            cv2.putText(
                panel,
                "Waiting for topic...",
                (15, height // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )
        else:
            panel = cv2.resize(image, (width, height))
        cv2.rectangle(panel, (0, 0), (width, 34), (0, 0, 0), -1)
        cv2.putText(
            panel,
            label,
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )
        return panel

    @staticmethod
    def _format_echo(values):
        lines = []
        ordered_topics = (
            "/line/status",
            "/line/confidence",
            "/line/cte",
            "/line/heading_angle",
            "/line/heading_error",
            "/line/kappa",
            "/line/evaluation_forward",
            "/line/reference_path",
            "/line/poly_coeffs",
            "/line/s_metric",
            "/line/kappa_s",
            "/camera/front_camera/camera_info",
        )
        for topic in ordered_topics:
            lines.append(f"=== {topic} ===")
            if topic not in values:
                lines.append("missing")
                continue
            value = values[topic]["value"]
            lines.append(json.dumps(value, indent=2, ensure_ascii=False))
        lines.append("")
        return "\n".join(lines)


def print_prompt(node):
    status, confidence, cte, heading, images = node.latest_status_summary()
    confidence_text = "missing" if confidence is None else f"{confidence:.3f}"
    cte_text = "missing" if cte is None else f"{cte:.4f} m"
    heading_text = "missing" if heading is None else f"{heading:.4f} rad"
    if node.capture_index < len(node.labels):
        next_label = node.labels[node.capture_index]
    else:
        next_label = f"extra_{node.capture_index + 1:02d}"
    print(
        "\r"
        f"Next={next_label} | status={status} | conf={confidence_text} | "
        f"cte={cte_text} | heading={heading_text} | images={','.join(images) or 'none'}",
        end="",
        flush=True,
    )


def keyboard_loop(node, spin_thread):
    print()
    print("Đặt xe vào vị trí cần test rồi nhấn Space để chụp evidence.")
    print("Thứ tự mặc định: straight -> enter_curve -> exit_curve. Nhấn q để thoát.")
    print()

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    last_prompt = 0.0
    try:
        tty.setcbreak(fd)
        while rclpy.ok() and spin_thread.is_alive():
            now = time.monotonic()
            if now - last_prompt > 0.5:
                print_prompt(node)
                last_prompt = now
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not readable:
                continue
            key = sys.stdin.read(1)
            if key == " ":
                print()
                folder = node.capture_next()
                print(f"Saved: {folder}")
            elif key.lower() == "q":
                print()
                break
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Interactive lane detection evidence capture."
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Evidence session directory. Default: evidence/lane_pose_<time>",
    )
    parser.add_argument(
        "--labels",
        default="straight,enter_curve,exit_curve",
        help="Comma-separated capture labels, consumed one per Space press.",
    )
    parser.add_argument("--jpeg-quality", type=int, default=92)
    return parser.parse_args()


def main():
    args = parse_args()
    project_root = Path(__file__).resolve().parents[1]
    output_dir = (
        Path(args.output_dir)
        if args.output_dir
        else project_root / "evidence" / f"lane_pose_{now_label()}"
    )
    labels = [safe_name(label) for label in args.labels.split(",") if label.strip()]
    if not labels:
        labels = ["snapshot"]

    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "session.json").write_text(
        json.dumps(
            {
                "created_local_time": datetime.now().isoformat(timespec="milliseconds"),
                "labels": labels,
                "roi_top_ratio": ROI_TOP_RATIO,
                "roi_bottom_ratio": ROI_BOTTOM_RATIO,
            },
            indent=2,
        ),
        encoding="utf-8",
    )

    rclpy.init()
    node = LanePoseEvidence(output_dir, labels, args.jpeg_quality)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    try:
        keyboard_loop(node, spin_thread)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)
        print(f"Evidence session: {output_dir}")


if __name__ == "__main__":
    main()
