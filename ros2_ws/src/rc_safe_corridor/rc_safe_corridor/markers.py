"""RViz2 MarkerArray builders for obstacles, corridor and reference path."""

import math

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


def _color(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def _base(frame_id, stamp, ns, marker_id, marker_type, lifetime_s=0.5):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = ns
    marker.id = int(marker_id)
    marker.type = marker_type
    marker.action = Marker.ADD
    marker.pose.orientation.w = 1.0
    marker.lifetime.sec = int(lifetime_s)
    marker.lifetime.nanosec = int((lifetime_s - int(lifetime_s)) * 1e9)
    return marker


def _yaw_quaternion(marker, yaw):
    marker.pose.orientation.z = math.sin(0.5 * yaw)
    marker.pose.orientation.w = math.cos(0.5 * yaw)


def obstacle_markers(obstacle_msgs, frame_id, stamp, height=0.15):
    array = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    clear.ns = "obstacles"
    array.markers.append(clear)
    for obs in obstacle_msgs:
        color = _color(1.0, 0.3, 0.1, 0.8) if obs.is_dynamic else _color(1.0, 0.75, 0.0, 0.8)
        if obs.shape == obs.SHAPE_BOX:
            marker = _base(frame_id, stamp, "obstacles", obs.id, Marker.CUBE)
            marker.scale.x = max(float(obs.length), 0.02)
            marker.scale.y = max(float(obs.width), 0.02)
            _yaw_quaternion(marker, float(obs.yaw))
        else:
            marker = _base(frame_id, stamp, "obstacles", obs.id, Marker.CYLINDER)
            marker.scale.x = max(2.0 * float(obs.radius), 0.02)
            marker.scale.y = marker.scale.x
        marker.scale.z = height
        marker.pose.position.x = float(obs.x)
        marker.pose.position.y = float(obs.y)
        marker.pose.position.z = 0.5 * height
        marker.color = color
        array.markers.append(marker)

        text = _base(frame_id, stamp, "obstacle_ids", obs.id, Marker.TEXT_VIEW_FACING)
        text.text = f"#{obs.id} {'dyn' if obs.is_dynamic else 'static'}"
        text.scale.z = 0.08
        text.pose.position.x = float(obs.x)
        text.pose.position.y = float(obs.y)
        text.pose.position.z = height + 0.08
        text.color = _color(1.0, 1.0, 1.0, 0.9)
        array.markers.append(text)

        if obs.is_dynamic:
            arrow = _base(frame_id, stamp, "obstacle_velocity", obs.id, Marker.ARROW)
            arrow.points = [
                Point(x=float(obs.x), y=float(obs.y), z=0.5 * height),
                Point(x=float(obs.x + obs.vx), y=float(obs.y + obs.vy), z=0.5 * height),
            ]
            arrow.scale.x = 0.02
            arrow.scale.y = 0.04
            arrow.scale.z = 0.04
            arrow.color = _color(1.0, 0.2, 0.2, 0.9)
            array.markers.append(arrow)
    return array


def corridor_markers(path, corridor, frame_id, stamp, s_values=None):
    """Draw reference path, corridor bounds (lines), corridor fill and d_ref.

    path: ReferencePath, corridor: CorridorResult (both in base_link).
    """
    array = MarkerArray()
    clear = Marker()
    clear.action = Marker.DELETEALL
    clear.ns = "corridor"
    array.markers.append(clear)

    def point_at(s, d, z=0.01):
        x, y, heading, _k = path.pose_at([s])
        nx = -math.sin(float(heading[0]))
        ny = math.cos(float(heading[0]))
        return Point(x=float(x[0] + d * nx), y=float(y[0] + d * ny), z=z)

    ref = _base(frame_id, stamp, "corridor", 0, Marker.LINE_STRIP)
    ref.scale.x = 0.01
    ref.color = _color(0.3, 0.6, 1.0, 0.9)
    ref.points = [point_at(s, 0.0) for s in corridor.s]
    array.markers.append(ref)

    lower = _base(frame_id, stamp, "corridor", 1, Marker.LINE_STRIP)
    lower.scale.x = 0.015
    lower.color = _color(0.1, 0.9, 0.2, 0.9) if corridor.feasible else _color(0.9, 0.1, 0.1, 0.9)
    lower.points = [point_at(s, d) for s, d in zip(corridor.s, corridor.lower)]
    array.markers.append(lower)

    upper = _base(frame_id, stamp, "corridor", 2, Marker.LINE_STRIP)
    upper.scale.x = 0.015
    upper.color = lower.color
    upper.points = [point_at(s, d) for s, d in zip(corridor.s, corridor.upper)]
    array.markers.append(upper)

    fill = _base(frame_id, stamp, "corridor", 3, Marker.TRIANGLE_LIST)
    fill.scale.x = fill.scale.y = fill.scale.z = 1.0
    fill.color = _color(0.1, 0.8, 0.3, 0.25) if corridor.feasible else _color(0.8, 0.2, 0.2, 0.25)
    for i in range(len(corridor.s) - 1):
        if corridor.v_max[i] <= 0.0:
            break
        a = point_at(corridor.s[i], corridor.lower[i], 0.005)
        b = point_at(corridor.s[i], corridor.upper[i], 0.005)
        c = point_at(corridor.s[i + 1], corridor.upper[i + 1], 0.005)
        d = point_at(corridor.s[i + 1], corridor.lower[i + 1], 0.005)
        fill.points.extend([a, b, c, a, c, d])
    if fill.points:
        array.markers.append(fill)

    dref = _base(frame_id, stamp, "corridor", 4, Marker.LINE_STRIP)
    dref.scale.x = 0.012
    dref.color = _color(1.0, 1.0, 1.0, 0.9)
    dref.points = [point_at(s, d, 0.02) for s, d in zip(corridor.s, corridor.d_ref)]
    array.markers.append(dref)

    if corridor.blocked_at_s >= 0.0:
        stop = _base(frame_id, stamp, "corridor", 5, Marker.CUBE)
        stop_point = point_at(corridor.blocked_at_s, 0.0, 0.05)
        stop.pose.position = stop_point
        stop.scale.x = 0.03
        stop.scale.y = 0.9
        stop.scale.z = 0.1
        stop.color = _color(1.0, 0.0, 0.0, 0.7)
        array.markers.append(stop)
    return array


def predicted_trajectory_marker(points_xy, frame_id, stamp, marker_id=0):
    marker = _base(frame_id, stamp, "nmpc_prediction", marker_id, Marker.LINE_STRIP)
    marker.scale.x = 0.012
    marker.color = _color(1.0, 0.4, 1.0, 0.9)
    marker.points = [Point(x=float(x), y=float(y), z=0.03) for x, y in points_xy]
    return marker
