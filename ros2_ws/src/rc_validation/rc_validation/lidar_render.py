import json
import math
from typing import Iterable


def scan_to_payload(
    *,
    angle_min: float,
    angle_increment: float,
    ranges: Iterable[float],
    range_min: float,
    range_max: float,
    frame_id: str,
    stamp_sec: int,
    stamp_nanosec: int,
    scan_rate_hz: float,
) -> dict:
    """Convert LaserScan fields into a compact, browser-safe payload."""
    points = []
    total_count = 0

    for index, raw_range in enumerate(ranges):
        total_count += 1
        distance = float(raw_range)
        if (
            not math.isfinite(distance)
            or distance < range_min
            or distance > range_max
        ):
            continue

        angle = angle_min + index * angle_increment
        points.append(
            [
                round(distance * math.cos(angle), 4),
                round(distance * math.sin(angle), 4),
                round(distance, 4),
            ]
        )

    return {
        "frame_id": frame_id,
        "stamp_sec": int(stamp_sec),
        "stamp_nanosec": int(stamp_nanosec),
        "scan_rate_hz": round(float(scan_rate_hz), 2),
        "range_min": float(range_min),
        "range_max": float(range_max),
        "total_count": total_count,
        "valid_count": len(points),
        "points": points,
    }


def payload_to_json(payload: dict) -> bytes:
    return json.dumps(
        payload,
        ensure_ascii=False,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")


def payload_to_svg(payload: dict, size: int = 900) -> str:
    if size < 320:
        raise ValueError("SVG size must be at least 320 pixels")

    max_distance = max(1.0, min(float(payload["range_max"]), 12.0))
    center = size / 2.0
    pixels_per_meter = (size * 0.46) / max_distance

    point_elements = []
    for x_forward, y_left, _ in payload["points"]:
        pixel_x = center - y_left * pixels_per_meter
        pixel_y = center - x_forward * pixels_per_meter
        point_elements.append(
            f'<circle cx="{pixel_x:.1f}" cy="{pixel_y:.1f}" r="2.2"/>'
        )

    grid_elements = []
    for radius_m in range(1, int(max_distance) + 1):
        radius_px = radius_m * pixels_per_meter
        grid_elements.append(
            f'<circle cx="{center:.1f}" cy="{center:.1f}" '
            f'r="{radius_px:.1f}" class="grid"/>'
        )
        grid_elements.append(
            f'<text x="{center + 6:.1f}" y="{center - radius_px + 16:.1f}" '
            f'class="label">{radius_m} m</text>'
        )

    metadata = (
        f'frame={payload["frame_id"]} | '
        f'rate={payload["scan_rate_hz"]:.2f} Hz | '
        f'valid={payload["valid_count"]}/{payload["total_count"]}'
    )

    return f"""<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{size}" height="{size}"
     viewBox="0 0 {size} {size}">
  <style>
    .background {{ fill: #101418; }}
    .grid {{ fill: none; stroke: #33404a; stroke-width: 1; }}
    .axis {{ stroke: #607080; stroke-width: 1; }}
    .scan {{ fill: #35e88b; }}
    .car {{ fill: #ffb020; }}
    .label {{ fill: #aebbc5; font: 14px sans-serif; }}
    .title {{ fill: #f4f7f8; font: 18px sans-serif; }}
  </style>
  <rect class="background" width="{size}" height="{size}"/>
  {''.join(grid_elements)}
  <line class="axis" x1="{center:.1f}" y1="0" x2="{center:.1f}" y2="{size}"/>
  <line class="axis" x1="0" y1="{center:.1f}" x2="{size}" y2="{center:.1f}"/>
  <g class="scan">{''.join(point_elements)}</g>
  <rect class="car" x="{center - 8:.1f}" y="{center - 16:.1f}"
        width="16" height="32" rx="3"/>
  <path class="car" d="M {center:.1f} {center - 30:.1f}
        l -8 12 h 16 z"/>
  <text class="title" x="18" y="30">LiDAR /scan — forward is up</text>
  <text class="label" x="18" y="{size - 18}">{metadata}</text>
</svg>
"""
