#!/usr/bin/env python3

from dataclasses import dataclass
import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import MarkerArray
from .visualizer import Visualizer


@dataclass
class Config:
    # width of the car
    car_width_m: float = 0.5

    # maximum steering angle of the car
    max_steering_angle_in_radians: float = 0.42

    # speed cone in degrees
    # => decreasing this results in more aggressive speed
    speed_cone_in_degrees: float = 0

    # maximum speed
    max_speed: float = 3

    # minimum speed
    min_speed: float = 0.1

    # minimum distance ahead of car to go full speed
    max_speed_distance: float = 4

    # visualizations
    vis_speed_cone_in_degrees_color: tuple[float, float, float] | None = None
    vis_scan_color: tuple[float, float, float] | None = None
    vis_scan_filtered_color: tuple[float, float, float] | None = None
    vis_direction_and_speed_color: tuple[float, float, float] | None = None
    vis_max_speed_distance_color: tuple[float, float, float] | None = None
    vis_forward_distance_color: tuple[float, float, float] | None = None


class DisparityExtender(Node):
    def __init__(self):
        super().__init__("disparity_extender")

        self.config = Config()

        # ---- subscribers ----
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )

        # ---- publishers ----
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.gap_publisher = self.create_publisher(MarkerArray, "/gap_vis", 1)

        self.visualizer = Visualizer(self, self.gap_publisher)

    def scan_callback(self, scan_msg: LaserScan):
        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        raw_ranges = list(scan_msg.ranges)

        # replace NaN / Inf with the reported maximum range
        ranges_unfiltered = [
            r if math.isfinite(r) else scan_msg.range_max for r in raw_ranges
        ]

        filtered = self._extend_disparities(ranges_unfiltered, angle_increment)

        goal_angle = self._find_furthest_point(filtered, angle_min, angle_increment)

        forward_distance = self._forward_distance(
            ranges_unfiltered, angle_min, angle_increment, center_angle=goal_angle
        )

        speed = self._compute_speed(forward_distance)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(
            max(
                -self.config.max_steering_angle_in_radians,
                min(goal_angle, self.config.max_steering_angle_in_radians),
            )
        )
        drive_msg.drive.speed = float(speed)
        self.drive_pub.publish(drive_msg)

        self.visualize(
            angle_min=angle_min,
            angle_increment=angle_increment,
            ranges=ranges_unfiltered,
            ranges_filtered=filtered,
            goal_angle=goal_angle,
            speed=speed,
            forward_distance=forward_distance,
        )

    def _extend_disparities(self, ranges, angle_increment):
        new_ranges = list(ranges)

        for i in range(1, len(ranges)):
            left, right = ranges[i - 1], ranges[i]
            shorter, longer = (i - 1, i) if left < right else (i, i - 1)

            # number of indices to bubble outward from the shorter reading
            bubble = int(
                math.asin(
                    min(1.0, (self.config.car_width_m / 2) / max(ranges[shorter], 1e-6))
                )
                / angle_increment
            )

            lo = max(0, shorter - bubble)
            hi = min(len(ranges) - 1, shorter + bubble)
            for j in range(lo, hi + 1):
                new_ranges[j] = min(new_ranges[j], ranges[shorter])

        return new_ranges

    def _find_furthest_point(self, filtered, angle_min, angle_increment):
        best_idx = -1
        best_dist = -1.0

        for i, dist in enumerate(filtered):
            angle = angle_min + i * angle_increment
            if abs(angle) <= math.pi / 2:  # only forward-facing rays
                if dist > best_dist:
                    best_dist = dist
                    best_idx = i

        if best_idx == -1:
            # Fallback: just go straight
            goal_angle = 0.0
        else:
            goal_angle = angle_min + best_idx * angle_increment

        return goal_angle

    def _forward_distance(
        self,
        ranges_unfiltered: list[float],
        angle_min: float,
        angle_increment: float,
        center_angle: float = 0.0,
    ) -> float:
        """
        Return the minimum filtered distance within a cone centred on
        `center_angle` (default: straight ahead).

        By passing goal_angle as center_angle the car judges its speed
        based on the space in the direction it is actually heading, not
        the wall it is turning away from at a corner.
        """
        if self.config.speed_cone_in_degrees == 0:
            return ranges_unfiltered[len(ranges_unfiltered) // 2]

        half_cone = math.radians(self.config.speed_cone_in_degrees) / 2
        min_dist = math.inf
        for i, dist in enumerate(ranges_unfiltered):
            angle = angle_min + i * angle_increment
            if abs(angle - center_angle) <= half_cone:
                if dist < min_dist:
                    min_dist = dist
        return min_dist if math.isfinite(min_dist) else 0.0

    def _compute_speed(self, forward_dist):
        return max(
            self.config.min_speed,
            self.config.max_speed
            * min(1, forward_dist / self.config.max_speed_distance),
        )

    def visualize(
        self,
        angle_min,
        angle_increment,
        ranges,
        ranges_filtered,
        goal_angle,
        speed,
        forward_distance,
    ):
        if self.config.vis_scan_color is not None:
            self.visualizer.polygon(
                [
                    (dist * math.cos(angle), dist * math.sin(angle))
                    for i, dist in enumerate(ranges)
                    for angle in [angle_min + i * angle_increment]
                    if abs(angle) <= math.pi / 2
                ],
                color=self.config.vis_scan_color,
            )

        if self.config.vis_scan_filtered_color is not None:
            self.visualizer.polygon(
                [
                    (dist * math.cos(angle), dist * math.sin(angle))
                    for i, dist in enumerate(ranges_filtered)
                    for angle in [angle_min + i * angle_increment]
                    if abs(angle) <= math.pi / 2
                ],
                color=self.config.vis_scan_filtered_color,
            )

        if self.config.vis_speed_cone_in_degrees_color is not None:
            half = math.radians(self.config.speed_cone_in_degrees) / 2
            self.visualizer.polygon(
                [(0, 0)]
                + [
                    (dist * math.cos(angle), dist * math.sin(angle))
                    for i, dist in enumerate(ranges_filtered)
                    for angle in [angle_min + i * angle_increment]
                    if half <= angle <= half
                ]
                + [(0, 0)],
                color=self.config.vis_speed_cone_in_degrees_color,
            )

        if self.config.vis_max_speed_distance_color is not None:
            self.visualizer.vector(
                0,
                self.config.max_speed_distance,
                color=self.config.vis_max_speed_distance_color,
            )

        if self.config.vis_forward_distance_color is not None:
            self.visualizer.vector(
                0, forward_distance, color=self.config.vis_forward_distance_color
            )

        if self.config.vis_direction_and_speed_color is not None:
            self.visualizer.vector(
                goal_angle, speed, color=self.config.vis_direction_and_speed_color
            )

        self.visualizer.publish()


def main(args=None):
    rclpy.init(args=args)
    node = DisparityExtender()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
