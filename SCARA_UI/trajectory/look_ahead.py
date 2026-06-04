"""上位机真实几何轨迹规划器。

规划器以真实几何段为输入，先按段长和真实尖角计算入口/出口速度，再按弧长采样输出
带 F 的 G1 点流。圆弧段不会被当成大量短折线拐角，因此 F 不会在圆弧内部周期性跌落。
"""

from dataclasses import dataclass
import math
from typing import List, Tuple


Point2D = Tuple[float, float]


@dataclass
class PlannerPoint:
    """发送给下位机的一条 G1 目标点。"""

    x: float
    y: float
    feed_mm_min: float
    silent: bool = False


@dataclass
class GeometrySegment:
    """真实几何段。kind 为 line 或 arc。"""

    kind: str
    start: Point2D
    end: Point2D
    length: float
    feed_mm_s: float = 0.0
    entry_speed: float = 0.0
    exit_speed: float = 0.0
    max_entry_speed: float = 0.0
    center: Point2D = (0.0, 0.0)
    radius: float = 0.0
    start_angle: float = 0.0
    delta_angle: float = 0.0

    def point_at(self, distance: float) -> Point2D:
        distance = max(0.0, min(self.length, distance))
        if self.kind == "arc":
            ratio = 0.0 if self.length <= 0.0 else distance / self.length
            angle = self.start_angle + self.delta_angle * ratio
            return (
                self.center[0] + self.radius * math.cos(angle),
                self.center[1] + self.radius * math.sin(angle),
            )

        ratio = 0.0 if self.length <= 0.0 else distance / self.length
        return (
            self.start[0] + (self.end[0] - self.start[0]) * ratio,
            self.start[1] + (self.end[1] - self.start[1]) * ratio,
        )

    def tangent_at_start(self) -> Point2D:
        return self._tangent_at(0.0)

    def tangent_at_end(self) -> Point2D:
        return self._tangent_at(self.length)

    def _tangent_at(self, distance: float) -> Point2D:
        if self.kind == "arc":
            ratio = 0.0 if self.length <= 0.0 else max(0.0, min(self.length, distance)) / self.length
            angle = self.start_angle + self.delta_angle * ratio
            sign = 1.0 if self.delta_angle >= 0.0 else -1.0
            return (-math.sin(angle) * sign, math.cos(angle) * sign)

        dx = self.end[0] - self.start[0]
        dy = self.end[1] - self.start[1]
        length = math.hypot(dx, dy)
        if length <= 0.0:
            return (1.0, 0.0)
        return (dx / length, dy / length)


class LookAheadPlanner:
    """按真实弧长输出带速度 F 的 G1 点流。"""

    def __init__(
        self,
        accel_mm_s2: float = 100.0,
        junction_deviation: float = 0.02,
        sample_dt: float = 0.04,
        max_segment_mm: float = 0.8,
        min_segment_mm: float = 0.05,
    ):
        self.accel_mm_s2 = max(1.0, accel_mm_s2)
        self.junction_deviation = max(0.001, junction_deviation)
        self.sample_dt = max(0.005, sample_dt)
        self.max_segment_mm = max(0.05, max_segment_mm)
        self.min_segment_mm = max(0.005, min_segment_mm)

    def plan_polyline(
        self,
        points: List[Point2D],
        feed_mm_s: float,
        start_speed: float = 0.0,
        end_speed: float = 0.0,
        silent_first: bool = False,
    ) -> List[PlannerPoint]:
        """规划一条折线。真实折线尖角会降速，圆弧请使用 plan_arc 或 plan_segments。"""
        segments = []
        for p0, p1 in zip(points, points[1:]):
            segment = self.line_segment(p0, p1)
            if segment is not None:
                segments.append(segment)
        return self.plan_segments(segments, feed_mm_s, start_speed, end_speed, silent_first=silent_first)

    def plan_line(
        self,
        start: Point2D,
        end: Point2D,
        feed_mm_s: float,
        start_speed: float = 0.0,
        end_speed: float = 0.0,
        silent: bool = False,
    ) -> List[PlannerPoint]:
        """规划单条直线。"""
        segment = self.line_segment(start, end)
        return self.plan_segments([segment] if segment else [], feed_mm_s, start_speed, end_speed, silent_first=silent)

    def plan_arc(
        self,
        start: Point2D,
        end: Point2D,
        radius: float,
        clockwise: bool,
        feed_mm_s: float,
        start_speed: float = 0.0,
        end_speed: float = 0.0,
        silent: bool = False,
    ) -> List[PlannerPoint]:
        """规划一段指定半径的短圆弧。"""
        segment = self.arc_segment(start, end, radius, clockwise)
        return self.plan_segments([segment], feed_mm_s, start_speed, end_speed, silent_first=silent)

    def plan_segments(
        self,
        segments: List[GeometrySegment],
        feed_mm_s: float,
        start_speed: float = 0.0,
        end_speed: float = 0.0,
        silent_first: bool = False,
    ) -> List[PlannerPoint]:
        segments = [segment for segment in segments if segment and segment.length > 0.001]
        if not segments:
            return []

        feed = max(0.1, feed_mm_s)
        for segment in segments:
            segment.feed_mm_s = feed
            segment.max_entry_speed = feed
            segment.entry_speed = feed
            segment.exit_speed = feed

        self._compute_junction_limits(segments, start_speed, end_speed)
        self._reverse_pass(segments, end_speed)
        self._forward_pass(segments, start_speed)

        planned = []
        for index, segment in enumerate(segments):
            segment.exit_speed = segments[index + 1].entry_speed if index + 1 < len(segments) else min(end_speed, feed)
            planned.extend(self._sample_segment(segment, silent=(silent_first and index == 0)))
        return planned

    def line_segment(self, start: Point2D, end: Point2D) -> GeometrySegment:
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length < 0.001:
            return None
        return GeometrySegment(kind="line", start=start, end=end, length=length)

    def arc_segment(self, start: Point2D, end: Point2D, radius: float, clockwise: bool) -> GeometrySegment:
        sx, sy = start
        ex, ey = end
        dx = ex - sx
        dy = ey - sy
        chord = math.hypot(dx, dy)
        if chord < 0.001:
            raise ValueError("圆弧起点和终点重合；当前界面未定义整圆圆心")

        min_radius = chord * 0.5
        if radius < min_radius:
            raise ValueError(f"圆弧半径过小，至少需要 {min_radius:.2f}mm，当前为 {radius:.2f}mm")

        mx = (sx + ex) * 0.5
        my = (sy + ey) * 0.5
        h = math.sqrt(max(0.0, radius * radius - min_radius * min_radius))
        nx = -dy / chord
        ny = dx / chord
        centers = ((mx + h * nx, my + h * ny), (mx - h * nx, my - h * ny))

        candidates = []
        for cx, cy in centers:
            a0 = math.atan2(sy - cy, sx - cx)
            a1 = math.atan2(ey - cy, ex - cx)
            if clockwise:
                delta = -((a0 - a1) % (2.0 * math.pi))
            else:
                delta = (a1 - a0) % (2.0 * math.pi)
            if abs(delta) < 1e-9:
                delta = -2.0 * math.pi if clockwise else 2.0 * math.pi
            candidates.append((abs(delta), cx, cy, a0, delta))

        _, cx, cy, a0, delta = min(candidates, key=lambda item: item[0])
        return GeometrySegment(
            kind="arc",
            start=start,
            end=end,
            length=abs(delta) * radius,
            center=(cx, cy),
            radius=radius,
            start_angle=a0,
            delta_angle=delta,
        )

    def _compute_junction_limits(self, segments: List[GeometrySegment], start_speed: float, end_speed: float) -> None:
        feed = segments[0].feed_mm_s
        segments[0].max_entry_speed = min(feed, start_speed)
        for index in range(1, len(segments)):
            prev = segments[index - 1]
            current = segments[index]
            dot = self._clamp(
                prev.tangent_at_end()[0] * current.tangent_at_start()[0]
                + prev.tangent_at_end()[1] * current.tangent_at_start()[1],
                -0.999999,
                0.999999,
            )
            if dot > 0.999:
                junction_speed = feed
            else:
                sin_theta_half = math.sqrt(max(0.0, 0.5 * (1.0 - dot)))
                junction_speed = math.sqrt(
                    max(
                        0.0,
                        self.accel_mm_s2
                        * self.junction_deviation
                        * sin_theta_half
                        / max(1e-6, 1.0 - sin_theta_half),
                    )
                )
            current.max_entry_speed = min(feed, junction_speed)
        segments[-1].exit_speed = min(end_speed, feed)

    def _reverse_pass(self, segments: List[GeometrySegment], end_speed: float) -> None:
        next_entry = min(end_speed, segments[-1].feed_mm_s)
        for segment in reversed(segments):
            allowed = math.sqrt(max(0.0, next_entry * next_entry + 2.0 * self.accel_mm_s2 * segment.length))
            segment.entry_speed = min(segment.max_entry_speed, segment.feed_mm_s, allowed)
            next_entry = segment.entry_speed

    def _forward_pass(self, segments: List[GeometrySegment], start_speed: float) -> None:
        prev_entry = min(start_speed, segments[0].feed_mm_s)
        segments[0].entry_speed = min(segments[0].entry_speed, prev_entry)
        for index in range(1, len(segments)):
            prev = segments[index - 1]
            current = segments[index]
            allowed = math.sqrt(max(0.0, prev.entry_speed * prev.entry_speed + 2.0 * self.accel_mm_s2 * prev.length))
            current.entry_speed = min(current.entry_speed, allowed)

    def _sample_segment(self, segment: GeometrySegment, silent: bool) -> List[PlannerPoint]:
        points = []
        distance = 0.0
        while distance < segment.length:
            current_speed = self._speed_at_distance(segment, distance)
            step = min(self.max_segment_mm, max(self.min_segment_mm, current_speed * self.sample_dt))
            next_distance = min(segment.length, distance + step)
            speed = self._speed_at_distance(segment, next_distance)
            x, y = segment.point_at(next_distance)
            points.append(PlannerPoint(x=x, y=y, feed_mm_min=max(1.0, speed * 60.0), silent=silent))
            distance = next_distance
            if len(points) > 50000:
                break
        return points

    def _speed_at_distance(self, segment: GeometrySegment, distance: float) -> float:
        distance = max(0.0, min(segment.length, distance))
        accel_speed = math.sqrt(max(0.0, segment.entry_speed**2 + 2.0 * self.accel_mm_s2 * distance))
        decel_speed = math.sqrt(
            max(0.0, segment.exit_speed**2 + 2.0 * self.accel_mm_s2 * (segment.length - distance))
        )
        return max(0.1, min(segment.feed_mm_s, accel_speed, decel_speed))

    @staticmethod
    def _clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))
