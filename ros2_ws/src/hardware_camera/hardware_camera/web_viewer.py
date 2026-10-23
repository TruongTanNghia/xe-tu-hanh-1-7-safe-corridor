#!/usr/bin/env python3

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time

from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String


ROI_TOP_RATIO = 0.40
ROI_BOTTOM_RATIO = 0.63


class LaneWebViewer(Node):
    def __init__(self):
        super().__init__("lane_web_viewer")
        self.declare_parameter("port", 8080)
        self.declare_parameter("max_fps", 5.0)
        self.declare_parameter("jpeg_quality", 60)
        self.declare_parameter("show_segmentation", False)
        self.declare_parameter("roi_top_ratio", ROI_TOP_RATIO)
        self.declare_parameter("roi_bottom_ratio", ROI_BOTTOM_RATIO)

        self.port = int(self.get_parameter("port").value)
        self.max_fps = float(self.get_parameter("max_fps").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.show_segmentation = bool(
            self.get_parameter("show_segmentation").value
        )
        self.roi_top_ratio = float(
            self.get_parameter("roi_top_ratio").value
        )
        self.roi_bottom_ratio = float(
            self.get_parameter("roi_bottom_ratio").value
        )
        if not 1 <= self.port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        if self.max_fps <= 0.0:
            raise ValueError("max_fps must be positive")
        if not 30 <= self.jpeg_quality <= 95:
            raise ValueError("jpeg_quality must be between 30 and 95")
        if not 0.0 <= self.roi_top_ratio < self.roi_bottom_ratio <= 1.0:
            raise ValueError("roi ratios must satisfy 0 <= top < bottom <= 1")

        self.bridge = CvBridge()
        self.frames = {}
        self.metrics = {}
        self.frame_lock = threading.Lock()
        self.stream_condition = threading.Condition()
        self.latest_jpegs = {"tracking": None}
        if self.show_segmentation:
            self.latest_jpegs["segmentation"] = None
        self.sequence = 0
        self.last_encode_time = 0.0

        # Tracking / Output is the lightweight default page. The segmentation
        # topics are optional because decoding all intermediate masks at camera
        # rate can stall the Pi even when the browser is on the second tab.
        tracking_topics = (
            ("debug", "/line/debug_image"),
            ("fitting", "/line/fitting_image"),
            ("boundary", "/line/debug/boundary_overlay"),
        )
        segmentation_topics = (
            ("raw", "/camera/front_camera/image_raw"),
            ("yellow", "/line/debug/yellow_mask"),
            ("dark", "/line/debug/dark_mask"),
            ("corridor", "/line/debug/corridor_mask"),
            ("scan_mask", "/line/debug/scan_mask"),
        )
        for name, topic in tracking_topics:
            self.create_subscription(
                Image,
                topic,
                lambda msg, name=name: self._image_callback(name, msg),
                2,
            )
        if self.show_segmentation:
            for name, topic in segmentation_topics:
                self.create_subscription(
                    Image,
                    topic,
                    lambda msg, name=name: self._image_callback(name, msg),
                    2,
                )
        self.create_subscription(
            String,
            "/line/debug/mode",
            lambda msg: self._metric_callback("mode", msg.data),
            10,
        )
        self.create_subscription(
            String,
            "/line/status",
            lambda msg: self._metric_callback("status", msg.data),
            10,
        )
        self.create_subscription(
            Float64,
            "/line/confidence",
            lambda msg: self._metric_callback("confidence", msg.data),
            10,
        )
        self.create_subscription(
            Float64,
            "/line/cte",
            lambda msg: self._metric_callback("cte", msg.data),
            10,
        )
        self.create_subscription(
            Float64,
            "/line/heading_angle",
            lambda msg: self._metric_callback("heading", msg.data),
            10,
        )
        self.create_subscription(
            Float64,
            "/line/kappa",
            lambda msg: self._metric_callback("kappa", msg.data),
            10,
        )

        handler = self._make_handler()
        self.http_server = ThreadingHTTPServer(("127.0.0.1", self.port), handler)
        self.http_server.daemon_threads = True
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            name="lane_web_viewer_http",
            daemon=True,
        )
        self.http_thread.start()
        self.get_logger().info(
            f"Lane web viewer: http://127.0.0.1:{self.port} "
            f"(max {self.max_fps:.1f} FPS)"
        )

    def _image_callback(self, name, message):
        try:
            image = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding="bgr8",
            )
        except Exception as error:
            self.get_logger().error(f"Cannot decode {name}: {error}")
            return

        with self.frame_lock:
            self.frames[name] = image.copy()
        self._update_stream_frame()

    def _metric_callback(self, name, value):
        with self.frame_lock:
            self.metrics[name] = value
        self._update_stream_frame()

    def _update_stream_frame(self):
        now = time.monotonic()
        if now - self.last_encode_time < 1.0 / self.max_fps:
            return
        self.last_encode_time = now

        with self.frame_lock:
            frames = {name: frame.copy() for name, frame in self.frames.items()}
            metrics = dict(self.metrics)

        raw = frames.get("raw")
        roi = None
        if raw is not None:
            top = int(raw.shape[0] * self.roi_top_ratio)
            bottom = int(raw.shape[0] * self.roi_bottom_ratio)
            roi = raw[top:bottom, :]
        roi_label = (
            f"Processor ROI {self.roi_top_ratio * 100:.0f}-"
            f"{self.roi_bottom_ratio * 100:.0f}%"
        )

        tracking = np.vstack([
            np.hstack([
                self._panel(frames.get("boundary"), "Boundary Geometry"),
                self._panel(frames.get("debug"), "Scan Bands"),
            ]),
            np.hstack([
                self._panel(frames.get("fitting"), "Poly Fitting"),
                self._metrics_panel(metrics),
            ]),
        ])

        tracking_jpeg = self._encode_jpeg(tracking)
        if tracking_jpeg is None:
            return

        with self.stream_condition:
            self.latest_jpegs["tracking"] = tracking_jpeg
            if self.show_segmentation:
                segmentation = np.vstack([
                    np.hstack([
                        self._panel(raw, "Raw Camera"),
                        self._panel(roi, roi_label),
                        self._panel(frames.get("yellow"), "Yellow Mask"),
                    ]),
                    np.hstack([
                        self._panel(frames.get("dark"), "Otsu Dark Mask"),
                        self._panel(frames.get("corridor"), "Corridor Mask"),
                        self._panel(frames.get("scan_mask"), "Final Scan Mask"),
                    ]),
                ])
                segmentation_jpeg = self._encode_jpeg(segmentation)
                if segmentation_jpeg is not None:
                    self.latest_jpegs["segmentation"] = segmentation_jpeg
            self.sequence += 1
            self.stream_condition.notify_all()

    def _encode_jpeg(self, image):
        ok, encoded = cv2.imencode(
            ".jpg",
            image,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
        )
        if not ok:
            return None
        return encoded.tobytes()

    @staticmethod
    def _panel(image, label):
        width, height = 480, 270
        if image is None or image.size == 0:
            panel = np.full((height, width, 3), 40, dtype=np.uint8)
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
    def _metrics_panel(metrics):
        width, height = 480, 270
        panel = np.full((height, width, 3), 35, dtype=np.uint8)
        cv2.rectangle(panel, (0, 0), (width, 34), (0, 0, 0), -1)
        cv2.putText(
            panel,
            "Metrics / Debug Mode",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )
        lines = [
            f"mode: {metrics.get('mode', 'WAITING')}",
            f"status: {metrics.get('status', 'WAITING')}",
            f"confidence: {metrics.get('confidence', '--')}",
            f"cte: {metrics.get('cte', '--')}",
            f"heading: {metrics.get('heading', '--')}",
            f"kappa: {metrics.get('kappa', '--')}",
        ]
        for index, line in enumerate(lines):
            if isinstance(line, str):
                text = line
            else:
                text = str(line)
            cv2.putText(
                panel,
                text,
                (18, 70 + index * 31),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (230, 230, 230),
                1,
            )
        return panel

    def _make_handler(self):
        viewer = self

        class ViewerHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    if viewer.show_segmentation:
                        page_controls = (
                            "<div class='tabs'>"
                            "<button id='seg-btn' class='active' "
                            "onclick=showPage('segmentation')>Segmentation</button>"
                            "<button id='trk-btn' "
                            "onclick=showPage('tracking')>Tracking / Output</button>"
                            "</div>"
                        )
                        stream_script = (
                            "<script>function showPage(page){"
                            "const img=document.getElementById('stream');"
                            "const seg=document.getElementById('seg-btn');"
                            "const trk=document.getElementById('trk-btn');"
                            "img.src='/stream/'+page+'.mjpg?'+Date.now();"
                            "seg.className=page==='segmentation'?'active':'';"
                            "trk.className=page==='tracking'?'active':'';"
                            "}</script>"
                        )
                        initial_stream = "/stream/segmentation.mjpg"
                    else:
                        page_controls = "<div class='tabs'>Tracking / Output</div>"
                        stream_script = ""
                        initial_stream = "/stream/tracking.mjpg"
                    content = (
                        "<!doctype html><html><head>"
                        "<meta charset='utf-8'>"
                        "<title>Lane Detection V2</title>"
                        "<style>body{margin:0;background:#111;color:#eee;"
                        "font-family:sans-serif;text-align:center}"
                        "h2{margin:12px 0 6px}.tabs{margin:0 0 8px}"
                        "button{background:#333;color:#eee;border:1px solid #666;"
                        "padding:8px 16px;margin:0 4px;border-radius:4px;"
                        "cursor:pointer}button.active{background:#1769aa}"
                        "img{max-width:100vw;max-height:88vh}</style>"
                        "</head><body><h2>Lane Detection V2</h2>"
                        f"{page_controls}<img id='stream' src='{initial_stream}'>"
                        f"{stream_script}</body></html>"
                    ).encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(content)))
                    self.end_headers()
                    self.wfile.write(content)
                    return

                stream_name = None
                if (
                    viewer.show_segmentation
                    and self.path.startswith("/stream/segmentation.mjpg")
                ):
                    stream_name = "segmentation"
                elif self.path.startswith("/stream/tracking.mjpg"):
                    stream_name = "tracking"
                if stream_name is None:
                    self.send_error(404)
                    return

                self.send_response(200)
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=frame",
                )
                self.send_header("Cache-Control", "no-cache")
                self.end_headers()
                last_sequence = -1
                try:
                    while rclpy.ok():
                        with viewer.stream_condition:
                            viewer.stream_condition.wait_for(
                                lambda: (
                                    viewer.latest_jpegs[stream_name] is not None
                                    and viewer.sequence != last_sequence
                                ),
                                timeout=1.0,
                            )
                            if viewer.latest_jpegs[stream_name] is None:
                                continue
                            frame = viewer.latest_jpegs[stream_name]
                            last_sequence = viewer.sequence
                        self.wfile.write(b"--frame\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            f"Content-Length: {len(frame)}\r\n\r\n".encode()
                        )
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def log_message(self, _format, *_args):
                return

        return ViewerHandler

    def destroy_node(self):
        if hasattr(self, "http_server"):
            self.http_server.shutdown()
            self.http_server.server_close()
        if hasattr(self, "http_thread"):
            self.http_thread.join(timeout=2.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneWebViewer()
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
