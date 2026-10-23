"""Block A1-A3: LiDAR preprocessing, adaptive breakpoint clustering, shape fit.

Everything here is pure numpy so it can be unit-tested and benchmarked
without ROS. Coordinates are 2D; +x forward, +y left (base_link).
"""

from dataclasses import dataclass, field
import math

import numpy as np

from rc_safe_corridor.geometry import transform_points


@dataclass
class ScanPoints:
    """Valid LiDAR returns, ordered by bearing."""

    xy: np.ndarray          # (N,2) in the requested frame
    ranges: np.ndarray      # (N,) measured distance in laser frame
    angles: np.ndarray      # (N,) bearing in laser frame


@dataclass
class ClusterShape:
    center: np.ndarray                      # (2,)
    radius: float                           # bounding radius from center
    length: float
    width: float
    yaw: float
    shape: str                              # "circle" or "box"
    point_count: int
    min_range: float
    points: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))


def scan_to_points(
    ranges,
    angle_min,
    angle_increment,
    range_min,
    range_max,
    laser_pose=(0.0, 0.0, 0.0),
):
    """Convert LaserScan arrays to ordered cartesian points.

    laser_pose = (x, y, yaw) of the laser frame in the output frame.
    Invalid (nan/inf/out of range) returns are dropped but the bearing
    ordering is kept so breakpoint clustering can see angular gaps.
    """
    ranges = np.asarray(ranges, dtype=float)
    index = np.arange(ranges.size)
    angles = float(angle_min) + index * float(angle_increment)
    valid = np.isfinite(ranges)
    valid &= ranges >= float(range_min)
    valid &= ranges <= float(range_max)
    ranges = ranges[valid]
    angles = angles[valid]
    local = np.column_stack(
        [ranges * np.cos(angles), ranges * np.sin(angles)]
    )
    xy = transform_points(local, *laser_pose)
    return ScanPoints(xy=xy, ranges=ranges, angles=angles)


def apply_roi(
    points: ScanPoints,
    x_min=0.12,
    x_max=2.5,
    y_abs_max=1.0,
    body_x_min=-0.40,
    body_x_max=0.40,
    body_y_abs=0.18,
):
    """Keep returns inside the forward region of interest.

    The body box removes reflections from the chassis, cables and the
    camera mast; it is applied before the ROI so the ROI can start right
    in front of the bumper.
    """
    xy = points.xy
    if xy.size == 0:
        return points
    inside_body = (
        (xy[:, 0] >= body_x_min)
        & (xy[:, 0] <= body_x_max)
        & (np.abs(xy[:, 1]) <= body_y_abs)
    )
    keep = (
        (~inside_body)
        & (xy[:, 0] >= x_min)
        & (xy[:, 0] <= x_max)
        & (np.abs(xy[:, 1]) <= y_abs_max)
    )
    return ScanPoints(
        xy=xy[keep], ranges=points.ranges[keep], angles=points.angles[keep]
    )


def lateral_distance_to_polyline(points_xy, polyline_xy):
    """Signed lateral distance (+left) and arc position of points to a polyline.

    Returns (d, s, inside) where inside is False when the projection falls
    outside the polyline extent.
    """
    points_xy = np.asarray(points_xy, dtype=float).reshape(-1, 2)
    polyline_xy = np.asarray(polyline_xy, dtype=float).reshape(-1, 2)
    if points_xy.size == 0 or polyline_xy.shape[0] < 2:
        n = points_xy.shape[0]
        return np.zeros(n), np.zeros(n), np.zeros(n, dtype=bool)
    a = polyline_xy[:-1]
    b = polyline_xy[1:]
    ab = b - a
    seg_len = np.linalg.norm(ab, axis=1)
    seg_len = np.where(seg_len < 1e-9, 1e-9, seg_len)
    cum = np.concatenate([[0.0], np.cumsum(seg_len)])
    # (P, S) projections
    ap = points_xy[:, None, :] - a[None, :, :]
    t = np.einsum("psk,sk->ps", ap, ab) / (seg_len**2)[None, :]
    t_clamped = np.clip(t, 0.0, 1.0)
    proj = a[None, :, :] + t_clamped[:, :, None] * ab[None, :, :]
    dist = np.linalg.norm(points_xy[:, None, :] - proj, axis=2)
    best = np.argmin(dist, axis=1)
    rows = np.arange(points_xy.shape[0])
    tangent = ab[best] / seg_len[best][:, None]
    rel = points_xy - proj[rows, best]
    d = tangent[:, 0] * rel[:, 1] - tangent[:, 1] * rel[:, 0]
    s = cum[best] + t_clamped[rows, best] * seg_len[best]
    inside = (t[rows, best] >= 0.0) & (t[rows, best] <= 1.0)
    return d, s, inside


def apply_lane_gate(points: ScanPoints, centerline_xy, half_width, margin=0.15):
    """Drop returns outside the lane band (walls, feet of bystanders)."""
    if points.xy.size == 0 or centerline_xy is None:
        return points
    d, _s, inside = lateral_distance_to_polyline(points.xy, centerline_xy)
    keep = inside & (np.abs(d) <= float(half_width) + float(margin))
    return ScanPoints(
        xy=points.xy[keep], ranges=points.ranges[keep], angles=points.angles[keep]
    )


def adaptive_breakpoint_clusters(
    points: ScanPoints,
    angle_increment,
    lambda_deg=10.0,
    sigma_r=0.01,
    min_points=3,
    max_angle_gap_factor=3.0,
):
    """Borges & Aldon adaptive breakpoint detector (O(n)).

    Consecutive returns belong to the same cluster while
    ||p_i - p_{i-1}|| <= r_{i-1} * sin(dphi) / sin(lambda - dphi) + 3 sigma_r.
    Angular gaps larger than max_angle_gap_factor * angle_increment always
    split (missing returns between two objects).
    """
    n = points.xy.shape[0]
    if n == 0:
        return []
    lam = math.radians(float(lambda_deg))
    dphi_nominal = abs(float(angle_increment))
    clusters = []
    current = [0]
    for i in range(1, n):
        dphi = abs(float(points.angles[i] - points.angles[i - 1]))
        if dphi > max_angle_gap_factor * dphi_nominal or dphi >= lam:
            threshold = -1.0
        else:
            threshold = (
                points.ranges[i - 1] * math.sin(dphi) / math.sin(lam - dphi)
                + 3.0 * float(sigma_r)
            )
        gap = float(np.linalg.norm(points.xy[i] - points.xy[i - 1]))
        if threshold < 0.0 or gap > threshold:
            clusters.append(current)
            current = [i]
        else:
            current.append(i)
    clusters.append(current)
    return [
        np.asarray(c, dtype=int) for c in clusters if len(c) >= int(min_points)
    ]


def fit_cluster_shape(
    cluster_xy,
    cluster_ranges,
    small_object_max_m=0.12,
    min_radius_m=0.03,
    min_box_side_m=0.06,
):
    """Fit a circle (small objects) or a PCA oriented box (long objects).

    A 2D LiDAR only sees the faces turned towards it, so the box is padded to
    min_box_side_m and callers should additionally inflate by the configured
    minimum obstacle size when generating the corridor.
    """
    xy = np.asarray(cluster_xy, dtype=float).reshape(-1, 2)
    count = xy.shape[0]
    centroid = xy.mean(axis=0)
    centered = xy - centroid
    if count >= 3:
        cov = centered.T @ centered / count
        eigval, eigvec = np.linalg.eigh(cov)
        major = eigvec[:, int(np.argmax(eigval))]
    else:
        major = np.array([1.0, 0.0])
    yaw = math.atan2(major[1], major[0])
    axis_u = np.array([math.cos(yaw), math.sin(yaw)])
    axis_v = np.array([-math.sin(yaw), math.cos(yaw)])
    proj_u = centered @ axis_u
    proj_v = centered @ axis_v
    extent_u = float(proj_u.max() - proj_u.min())
    extent_v = float(proj_v.max() - proj_v.min())
    extent = max(extent_u, extent_v)
    min_range = float(np.min(cluster_ranges)) if len(cluster_ranges) else 0.0

    if extent <= float(small_object_max_m):
        radius = max(
            float(np.max(np.linalg.norm(centered, axis=1))),
            float(min_radius_m),
        )
        return ClusterShape(
            center=centroid,
            radius=radius,
            length=2.0 * radius,
            width=2.0 * radius,
            yaw=0.0,
            shape="circle",
            point_count=count,
            min_range=min_range,
            points=xy,
        )

    length = max(extent_u, float(min_box_side_m))
    width = max(extent_v, float(min_box_side_m))
    center = (
        centroid
        + axis_u * 0.5 * (proj_u.max() + proj_u.min())
        + axis_v * 0.5 * (proj_v.max() + proj_v.min())
    )
    radius = 0.5 * math.hypot(length, width)
    return ClusterShape(
        center=center,
        radius=radius,
        length=length,
        width=width,
        yaw=yaw,
        shape="box",
        point_count=count,
        min_range=min_range,
        points=xy,
    )


def detect_obstacles(
    points: ScanPoints,
    angle_increment,
    lambda_deg=10.0,
    sigma_r=0.01,
    min_points=3,
    max_cluster_extent_m=1.2,
    small_object_max_m=0.12,
    min_radius_m=0.03,
):
    """Cluster + fit in one call. Clusters larger than max_cluster_extent_m
    (walls, table edges) are discarded."""
    shapes = []
    for index in adaptive_breakpoint_clusters(
        points, angle_increment, lambda_deg, sigma_r, min_points
    ):
        shape = fit_cluster_shape(
            points.xy[index],
            points.ranges[index],
            small_object_max_m=small_object_max_m,
            min_radius_m=min_radius_m,
        )
        if max(shape.length, shape.width) > float(max_cluster_extent_m):
            continue
        shapes.append(shape)
    return shapes
