"""Small 2D geometry helpers shared by perception and planning."""

import math

import numpy as np


def wrap_angle(angle):
    """Wrap angle to (-pi, pi]."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def rotation_matrix(yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def transform_points(points, x, y, yaw):
    """Map points (N,2) from a child frame into its parent.

    (x, y, yaw) is the pose of the child frame expressed in the parent.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if points.size == 0:
        return points
    return points @ rotation_matrix(yaw).T + np.array([x, y])


def inverse_transform_points(points, x, y, yaw):
    """Map points (N,2) from a parent frame into the child frame."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if points.size == 0:
        return points
    return (points - np.array([x, y])) @ rotation_matrix(yaw)


def rotate_vectors(vectors, yaw):
    vectors = np.asarray(vectors, dtype=float).reshape(-1, 2)
    if vectors.size == 0:
        return vectors
    return vectors @ rotation_matrix(yaw).T


def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def box_corners(cx, cy, yaw, length, width):
    """Return the 4 corners (4,2) of an oriented box."""
    half_l = 0.5 * float(length)
    half_w = 0.5 * float(width)
    local = np.array(
        [
            [half_l, half_w],
            [half_l, -half_w],
            [-half_l, -half_w],
            [-half_l, half_w],
        ]
    )
    return transform_points(local, cx, cy, yaw)


def circle_samples(cx, cy, radius, count=12):
    angles = np.linspace(0.0, 2.0 * math.pi, int(count), endpoint=False)
    return np.column_stack(
        [cx + radius * np.cos(angles), cy + radius * np.sin(angles)]
    )
