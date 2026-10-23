import math

import numpy as np

from rc_safe_corridor.scan_clustering import (
    adaptive_breakpoint_clusters,
    apply_lane_gate,
    apply_roi,
    detect_obstacles,
    fit_cluster_shape,
    lateral_distance_to_polyline,
    scan_to_points,
)
from rc_safe_corridor.synthetic import SimObstacle, synthetic_scan


LASER_POSE = (-0.005, 0.0, 0.0)


def _points_for(obstacles, **kwargs):
    ranges, angle_min, inc = synthetic_scan(obstacles, laser_pose=LASER_POSE, **kwargs)
    points = scan_to_points(ranges, angle_min, inc, 0.15, 12.0, laser_pose=LASER_POSE)
    return apply_roi(points), inc


def test_scan_to_points_drops_invalid_and_transforms_to_base_link():
    ranges = np.array([1.0, math.inf, math.nan, 0.01, 2.0])
    points = scan_to_points(ranges, 0.0, math.pi / 2, 0.1, 10.0, laser_pose=(0.5, 0.0, 0.0))
    assert points.xy.shape == (2, 2)
    assert np.allclose(points.xy[0], [1.5, 0.0])
    # index 4 -> angle 2*pi -> along +x again
    assert np.allclose(points.xy[1], [2.5, 0.0], atol=1e-9)


def test_single_pole_in_front_is_detected_as_circle():
    points, inc = _points_for([SimObstacle(x=1.0, y=0.0, radius=0.04)])
    shapes = detect_obstacles(points, inc)
    assert len(shapes) == 1
    shape = shapes[0]
    assert shape.shape == "circle"
    # The visible face biases the centroid towards the sensor by ~radius/2.
    assert abs(shape.center[0] - 1.0) < 0.05
    assert abs(shape.center[1]) < 0.02
    assert shape.radius <= 0.06


def test_two_separate_objects_make_two_clusters():
    points, inc = _points_for(
        [SimObstacle(x=1.0, y=0.25, radius=0.05), SimObstacle(x=1.6, y=-0.25, radius=0.05)]
    )
    shapes = detect_obstacles(points, inc)
    assert len(shapes) == 2
    ys = sorted(float(s.center[1]) for s in shapes)
    assert ys[0] < -0.15 and ys[1] > 0.15


def test_box_obstacle_is_fitted_as_box_with_reasonable_size():
    points, inc = _points_for([SimObstacle(x=1.2, y=0.0, length=0.30, width=0.20)])
    shapes = detect_obstacles(points, inc)
    assert len(shapes) == 1
    shape = shapes[0]
    assert shape.shape == "box"
    # Only the front face (0.20 m wide) is visible.
    assert 0.15 <= max(shape.length, shape.width) <= 0.26


def test_walls_outside_lane_are_removed_by_lane_gate():
    obstacles = [SimObstacle(x=1.0, y=0.0, radius=0.05)]
    ranges, angle_min, inc = synthetic_scan(
        obstacles, laser_pose=LASER_POSE, walls_y=(0.9, -0.9)
    )
    points = apply_roi(
        scan_to_points(ranges, angle_min, inc, 0.15, 12.0, laser_pose=LASER_POSE),
        y_abs_max=1.5,
    )
    assert len(detect_obstacles(points, inc, max_cluster_extent_m=10.0)) >= 2
    centerline = np.column_stack([np.linspace(-0.5, 3.0, 36), np.zeros(36)])
    gated = apply_lane_gate(points, centerline, half_width=0.45, margin=0.15)
    shapes = detect_obstacles(gated, inc)
    assert len(shapes) == 1
    assert abs(shape_center_x(shapes[0]) - 1.0) < 0.05


def shape_center_x(shape):
    return float(shape.center[0])


def test_breakpoint_split_on_range_jump():
    angles = np.deg2rad(np.arange(0, 10, 1.0))
    ranges = np.concatenate([np.full(5, 1.0), np.full(5, 2.0)])
    xy = np.column_stack([ranges * np.cos(angles), ranges * np.sin(angles)])
    from rc_safe_corridor.scan_clustering import ScanPoints

    pts = ScanPoints(xy=xy, ranges=ranges, angles=angles)
    clusters = adaptive_breakpoint_clusters(pts, math.radians(1.0))
    assert len(clusters) == 2
    assert len(clusters[0]) == 5 and len(clusters[1]) == 5


def test_lateral_distance_sign_is_positive_left():
    polyline = np.array([[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]])
    d, s, inside = lateral_distance_to_polyline(
        np.array([[1.0, 0.2], [1.5, -0.3], [3.0, 0.0]]), polyline
    )
    assert np.allclose(d[:2], [0.2, -0.3])
    assert np.allclose(s[:2], [1.0, 1.5])
    assert inside[0] and inside[1] and not inside[2]


def test_fit_cluster_shape_min_radius_clamp():
    xy = np.array([[1.0, 0.0], [1.005, 0.002], [1.0, 0.004]])
    shape = fit_cluster_shape(xy, [1.0, 1.0, 1.0], min_radius_m=0.03)
    assert shape.shape == "circle"
    assert shape.radius == 0.03
