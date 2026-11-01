"""Planar geometry for perception's oriented obstacle boxes."""

from __future__ import annotations

import math


def box_corners(obj, *, minimum_size_m: float = 0.0) -> tuple[tuple[float, float], ...]:
    x, y = float(obj.pose.position.x), float(obj.pose.position.y)
    length, width = float(obj.size.x), float(obj.size.y)
    q = obj.pose.orientation
    values = (x, y, length, width, float(q.x), float(q.y), float(q.z), float(q.w))
    if not all(math.isfinite(value) for value in values) or min(length, width) < 0.0:
        return ()
    length, width = max(minimum_size_m, length), max(minimum_size_m, width)
    if min(length, width) <= 0.0:
        return ()
    norm2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    # Legacy observations leave the quaternion zero-initialized.
    yaw = math.atan2(
        2.0 * (q.w*q.z + q.x*q.y), norm2 - 2.0 * (q.y*q.y + q.z*q.z)
    ) if norm2 > 1.0e-12 else 0.0
    return rectangle_corners(x, y, yaw, -length*0.5, length*0.5, width*0.5)


def rectangle_corners(x, y, yaw, rear, front, half_width):
    c, s = math.cos(yaw), math.sin(yaw)
    return tuple(
        (x + c*u - s*v, y + s*u + c*v)
        for u, v in ((rear, -half_width), (front, -half_width),
                     (front, half_width), (rear, half_width))
    )


def ego_box_extents(obj) -> tuple[float, float, float, float] | None:
    corners = box_corners(obj, minimum_size_m=0.1)
    if not corners:
        return None
    return (min(p[0] for p in corners), max(p[0] for p in corners),
            min(p[1] for p in corners), max(p[1] for p in corners))


def polygon_clearance(a, b) -> float:
    """Euclidean gap, or a nonpositive SAT overlap, between convex rectangles."""
    separating = False
    overlap = math.inf
    for polygon in (a, b):
        for p, q in zip(polygon, (*polygon[1:], polygon[0])):
            dx, dy = q[0]-p[0], q[1]-p[1]
            norm = math.hypot(dx, dy)
            if norm < 1.0e-9:
                continue
            nx, ny = -dy/norm, dx/norm
            pa = [nx*x + ny*y for x, y in a]
            pb = [nx*x + ny*y for x, y in b]
            gap = max(min(pa)-max(pb), min(pb)-max(pa))
            separating |= gap > 0.0
            overlap = min(overlap, -gap)
    if not separating:
        return -max(0.0, overlap)
    minimum = math.inf
    for vertices, edges in ((a, b), (b, a)):
        for x, y in vertices:
            for p, q in zip(edges, (*edges[1:], edges[0])):
                dx, dy = q[0]-p[0], q[1]-p[1]
                length2 = dx*dx + dy*dy
                t = min(1.0, max(0.0, ((x-p[0])*dx + (y-p[1])*dy)/max(length2, 1.0e-12)))
                minimum = min(minimum, math.hypot(x-p[0]-t*dx, y-p[1]-t*dy))
    return minimum
