"""Controller-compatible lane readiness checks, without ROS."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PathRules:
    frame_id: str = 'front_axle_bev'
    minimum_points: int = 8
    minimum_span: float = .12
    minimum_x: float = .05
    maximum_x: float = 2.20
    maximum_gap: float = .15
    x_max: float = 3.
    y_max: float = .6
    mpp: float = .01

    def __post_init__(self):
        if (not self.frame_id or self.minimum_points < 3 or
                not all(math.isfinite(x) and x > 0 for x in (
                    self.minimum_span, self.maximum_x, self.maximum_gap, self.x_max, self.y_max, self.mpp)) or
                not math.isfinite(self.minimum_x) or not 0 <= self.minimum_x < self.maximum_x <= self.x_max):
            raise ValueError('Invalid tunnel path geometry rules')


def stamp_ns(message):
    capture_ns = getattr(message, 'capture_ns', None)  # RawLane: no decode needed
    if capture_ns is not None:
        return capture_ns
    return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec


def metric_path(message, rules):
    """Use the controller's canvas, support and contiguous-path requirements."""
    try:
        width, height = message.source_width, message.source_height
        sx = message.centerline_bev_width_m / width
        sy = message.centerline_bev_height_m / height
        close = lambda a, b: math.isclose(a, b, rel_tol=1e-4, abs_tol=0.)
        if (not message.centerline_valid or message.centerline_sample_limit_reached or
                message.state not in (1, 2, 3) or message.header.frame_id != rules.frame_id or
                width <= 0 or height <= 0 or
                not all(math.isfinite(v) and v > 0 for v in (sx, sy)) or
                not close(sx, rules.mpp) or not close(sy, rules.mpp) or
                not close(width * sx, 2 * rules.y_max) or not close(height * sy, rules.x_max) or
                not 2 <= len(message.centerline_points) <= 10000 or
                len(message.centerline_points) != len(message.centerline_support)):
            return None
        raw = []
        for p, support in zip(message.centerline_points, message.centerline_support):
            if (support not in (1, 2, 3, 4) or
                    not all(math.isfinite(v) for v in (p.x, p.y, p.z)) or abs(p.z) > 1e-6 or
                    not 0 <= p.x < width + message.padding_left + message.padding_right or
                    not 0 <= p.y < height):
                return None
            raw.append((rules.x_max - (p.y + .5) * sy,
                        rules.y_max - (p.x - message.padding_left + .5) * sx, support))
        points, span = [], 0.
        for p in raw:
            if not rules.minimum_x <= p[0] <= rules.maximum_x:
                if points:
                    break
                continue
            if points:
                gap = math.hypot(p[0] - points[-1][0], p[1] - points[-1][1])
                if gap < 1e-5:
                    continue
                if gap > rules.maximum_gap:
                    break
                span += gap
            points.append(p)
        return points if len(points) >= rules.minimum_points and span >= rules.minimum_span else None
    except (AttributeError, TypeError, ValueError, ZeroDivisionError, OverflowError):
        return None
