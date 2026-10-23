import json
import math

from rc_validation.lidar_render import (
    payload_to_json,
    payload_to_svg,
    scan_to_payload,
)


def test_scan_filters_invalid_ranges_and_preserves_orientation():
    payload = scan_to_payload(
        angle_min=0.0,
        angle_increment=math.pi / 2.0,
        ranges=[1.0, 2.0, float("inf"), 0.01, 11.0],
        range_min=0.1,
        range_max=10.0,
        frame_id="laser",
        stamp_sec=12,
        stamp_nanosec=34,
        scan_rate_hz=9.8,
    )

    assert payload["total_count"] == 5
    assert payload["valid_count"] == 2
    assert payload["points"][0] == [1.0, 0.0, 1.0]
    assert payload["points"][1] == [0.0, 2.0, 2.0]


def test_json_and_svg_are_finite_and_self_describing():
    payload = scan_to_payload(
        angle_min=-math.pi / 2.0,
        angle_increment=math.pi / 2.0,
        ranges=[1.0, 1.0, 1.0],
        range_min=0.1,
        range_max=8.0,
        frame_id="laser",
        stamp_sec=1,
        stamp_nanosec=2,
        scan_rate_hz=10.0,
    )

    encoded = payload_to_json(payload)
    assert json.loads(encoded)["frame_id"] == "laser"
    assert b"NaN" not in encoded

    svg = payload_to_svg(payload)
    assert "LiDAR /scan" in svg
    assert "forward is up" in svg
    assert "<circle" in svg
