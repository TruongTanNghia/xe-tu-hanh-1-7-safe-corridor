#!/usr/bin/env python3

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from rc_validation.lidar_render import (
    payload_to_json,
    payload_to_svg,
    scan_to_payload,
)


INDEX_HTML = """<!doctype html>
<html lang="vi">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>RC Car LiDAR Viewer</title>
  <style>
    body { margin:0; background:#101418; color:#eef3f5;
           font-family:system-ui,sans-serif; }
    header { padding:12px 18px; display:flex; gap:20px; align-items:center; }
    #status { color:#9fb0bc; }
    canvas { display:block; width:min(94vw,850px); height:min(94vw,850px);
             margin:auto; background:#101418; border:1px solid #33404a; }
    a { color:#35e88b; }
  </style>
</head>
<body>
  <header>
    <strong>LiDAR /scan</strong>
    <span id="status">Đang chờ dữ liệu…</span>
    <a href="/snapshot.svg" target="_blank">Lưu SVG hiện tại</a>
  </header>
  <canvas id="view" width="900" height="900"></canvas>
  <script>
    const canvas = document.getElementById("view");
    const ctx = canvas.getContext("2d");
    const statusNode = document.getElementById("status");
    function draw(scan) {
      const size = canvas.width, c = size / 2;
      const maxRange = Math.max(1, Math.min(scan.range_max, 12));
      const scale = size * 0.46 / maxRange;
      ctx.fillStyle = "#101418"; ctx.fillRect(0, 0, size, size);
      ctx.strokeStyle = "#33404a"; ctx.lineWidth = 1;
      for (let r = 1; r <= Math.floor(maxRange); r++) {
        ctx.beginPath(); ctx.arc(c, c, r * scale, 0, 2 * Math.PI); ctx.stroke();
        ctx.fillStyle = "#9fb0bc"; ctx.fillText(`${r} m`, c + 6, c-r*scale+16);
      }
      ctx.strokeStyle = "#607080";
      ctx.beginPath(); ctx.moveTo(c,0); ctx.lineTo(c,size);
      ctx.moveTo(0,c); ctx.lineTo(size,c); ctx.stroke();
      ctx.fillStyle = "#35e88b";
      for (const [forward,left] of scan.points) {
        ctx.beginPath();
        ctx.arc(c-left*scale, c-forward*scale, 2.2, 0, 2*Math.PI);
        ctx.fill();
      }
      ctx.fillStyle = "#ffb020"; ctx.fillRect(c-8,c-16,16,32);
      ctx.beginPath(); ctx.moveTo(c,c-30); ctx.lineTo(c-8,c-18);
      ctx.lineTo(c+8,c-18); ctx.closePath(); ctx.fill();
      ctx.fillStyle = "#eef3f5"; ctx.font = "18px system-ui";
      ctx.fillText("forward", c+12, c-24);
      statusNode.textContent =
        `frame=${scan.frame_id} | ${scan.scan_rate_hz.toFixed(2)} Hz | ` +
        `valid=${scan.valid_count}/${scan.total_count}`;
    }
    async function update() {
      try {
        const response = await fetch("/api/scan", {cache:"no-store"});
        if (!response.ok) throw new Error("Chưa nhận /scan");
        draw(await response.json());
      } catch (error) {
        statusNode.textContent = error.message;
      }
    }
    setInterval(update, 100); update();
  </script>
</body>
</html>
"""


class LidarWebViewer(Node):
    def __init__(self):
        super().__init__("lidar_web_viewer")
        self.declare_parameter("topic", "/scan")
        self.declare_parameter("port", 8090)
        self.declare_parameter("bind_address", "127.0.0.1")

        self.topic = str(self.get_parameter("topic").value)
        self.port = int(self.get_parameter("port").value)
        self.bind_address = str(self.get_parameter("bind_address").value)
        if not 1 <= self.port <= 65535:
            raise ValueError("port must be between 1 and 65535")

        self.payload = None
        self.payload_lock = threading.Lock()
        self.last_scan_time = None
        self.scan_rate_hz = 0.0

        self.subscription = self.create_subscription(
            LaserScan,
            self.topic,
            self._scan_callback,
            qos_profile_sensor_data,
        )

        handler = self._make_handler()
        self.http_server = ThreadingHTTPServer(
            (self.bind_address, self.port),
            handler,
        )
        self.http_server.daemon_threads = True
        self.http_thread = threading.Thread(
            target=self.http_server.serve_forever,
            name="lidar_web_viewer_http",
            daemon=True,
        )
        self.http_thread.start()
        self.get_logger().info(
            f"LiDAR viewer listening on http://{self.bind_address}:{self.port}; "
            f"topic={self.topic}, QoS=sensor_data"
        )

    def _scan_callback(self, message: LaserScan):
        now = time.monotonic()
        if self.last_scan_time is not None:
            delta = now - self.last_scan_time
            if delta > 0.0:
                instantaneous_rate = 1.0 / delta
                if self.scan_rate_hz == 0.0:
                    self.scan_rate_hz = instantaneous_rate
                else:
                    self.scan_rate_hz = (
                        0.85 * self.scan_rate_hz + 0.15 * instantaneous_rate
                    )
        self.last_scan_time = now

        payload = scan_to_payload(
            angle_min=message.angle_min,
            angle_increment=message.angle_increment,
            ranges=message.ranges,
            range_min=message.range_min,
            range_max=message.range_max,
            frame_id=message.header.frame_id,
            stamp_sec=message.header.stamp.sec,
            stamp_nanosec=message.header.stamp.nanosec,
            scan_rate_hz=self.scan_rate_hz,
        )
        with self.payload_lock:
            self.payload = payload

    def _current_payload(self):
        with self.payload_lock:
            return None if self.payload is None else dict(self.payload)

    def _make_handler(self):
        viewer = self

        class ViewerHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    self._send(
                        200,
                        "text/html; charset=utf-8",
                        INDEX_HTML.encode("utf-8"),
                    )
                    return

                payload = viewer._current_payload()
                if self.path == "/api/scan":
                    if payload is None:
                        self._send(
                            503,
                            "application/json",
                            b'{"error":"waiting_for_scan"}',
                        )
                    else:
                        self._send(
                            200,
                            "application/json",
                            payload_to_json(payload),
                        )
                    return

                if self.path == "/snapshot.svg":
                    if payload is None:
                        self._send(
                            503,
                            "text/plain; charset=utf-8",
                            "Chưa nhận được /scan\n".encode("utf-8"),
                        )
                    else:
                        self._send(
                            200,
                            "image/svg+xml",
                            payload_to_svg(payload).encode("utf-8"),
                        )
                    return

                self._send(404, "text/plain", b"Not found\n")

            def _send(self, status, content_type, body):
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

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
        node = LidarWebViewer()
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
