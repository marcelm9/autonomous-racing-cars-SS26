#!/usr/bin/env python3
"""
Disparity Extender Algorithm
Based on: https://www.nathanotterness.com/2019/04/the-disparity-extender-algorithm-and.html

Drop-in replacement for reactive.py — subscribes/publishes to the same topics.
"""

from dataclasses import dataclass
import math
from random import randint
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from .visualizer import Visualizer


@dataclass
class Config:
    # maximum steering angle of the car
    max_steering_angle_in_radians: float = 0.42

    # angle over which the minimum distance is checked
    # => decreasing this results in more aggressive steering (=> steering closer to the wall)
    disparity_angle_in_degrees: float = 10

    # speed cone in degrees
    # => decreasing this results in more aggressive speed
    speed_cone_in_degrees: float = 10

    # maximum speed
    max_speed: float = 3

    # minimum distance ahead of car to go full speed
    max_speed_distance: float = 3

    # visualizations
    vis_disparity_angle_in_degrees_color: tuple[float, float, float] | None = None
    vis_speed_cone_in_degrees_color: tuple[float, float, float] | None = None
    vis_scan_color: tuple[float, float, float] | None = None
    vis_scan_filtered_color: tuple[float, float, float] | None = None
    vis_direction_color: tuple[float, float, float] | None = None


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

    # ------------------------------------------------------------------ #
    #  SCAN CALLBACK — main pipeline                                       #
    # ------------------------------------------------------------------ #
    def scan_callback(self, scan_msg: LaserScan):
        self.scan_msg = scan_msg

        angle_min = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        raw_ranges = list(scan_msg.ranges)
        num_lasers = len(raw_ranges)

        # Replace NaN / Inf with the reported maximum range
        max_range = scan_msg.range_max
        ranges = [r if math.isfinite(r) else max_range for r in raw_ranges]

        filtered = self._extend_disparities(ranges, angle_increment)

        goal_angle = self._find_furthest_point(filtered, angle_min, angle_increment)

        forward_dist = self._forward_distance(
            filtered, angle_min, angle_increment, center_angle=goal_angle
        )
        self.speed = self._compute_speed(forward_dist)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = float(
            max(
                -self.config.max_steering_angle_in_radians,
                min(goal_angle, self.config.max_steering_angle_in_radians),
            )
        )
        drive_msg.drive.speed = float(self.speed)
        self.drive_pub.publish(drive_msg)

        self.visualize(
            angle_min=angle_min,
            angle_increment=angle_increment,
            ranges=ranges,
            ranges_filtered=filtered,
            goal_angle=goal_angle,
        )

    # ------------------------------------------------------------------ #
    #  DISPARITY EXTENSION                                                 #
    # ------------------------------------------------------------------ #
    def _extend_disparities(
        self,
        ranges: list[float],
        angle_increment: float,
    ) -> list[float]:
        check_left_right_n = (
            int(math.radians(self.config.disparity_angle_in_degrees) / angle_increment)
            + 1
        )

        new_ranges = list(ranges)
        for i in range(len(ranges)):
            lo = max(0, i - check_left_right_n)
            hi = min(len(ranges), i + check_left_right_n)
            min_neighbour = min(ranges[lo:hi])
            new_ranges[i] = min(new_ranges[i], min_neighbour)

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
        filtered: list[float],
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
        min_dist = math.inf
        for i, dist in enumerate(filtered):
            angle = angle_min + i * angle_increment
            if abs(angle - center_angle) <= self.config.speed_cone_in_degrees / 2:
                if dist < min_dist:
                    min_dist = dist
        return min_dist if math.isfinite(min_dist) else 0.0

    def _compute_speed(self, forward_dist):
        return self.config.max_speed * min(
            1, forward_dist / self.config.max_speed_distance
        )

    def visualize(
        self, angle_min, angle_increment, ranges, ranges_filtered, goal_angle
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

        if self.config.vis_disparity_angle_in_degrees_color is not None:
            half = math.radians(self.config.disparity_angle_in_degrees) / 2
            self.visualizer.polygon(
                [
                    (dist * math.cos(angle), dist * math.sin(angle))
                    for i, dist in enumerate(ranges_filtered)
                    for angle in [angle_min + i * angle_increment]
                    if goal_angle - half <= angle <= goal_angle + half
                ],
                color=self.config.vis_disparity_angle_in_degrees_color,
            )

        if self.config.vis_speed_cone_in_degrees_color is not None:
            half = math.radians(self.config.speed_cone_in_degrees) / 2
            self.visualizer.polygon(
                [
                    (dist * math.cos(angle), dist * math.sin(angle))
                    for i, dist in enumerate(ranges_filtered)
                    for angle in [angle_min + i * angle_increment]
                    if goal_angle - half <= angle <= goal_angle + half
                ],
                color=self.config.vis_speed_cone_in_degrees_color,
            )

        if self.config.vis_direction_color is not None:
            self.visualizer.vector(
                goal_angle, self.speed, color=self.config.vis_direction_color
            )

        self.visualizer.publish()


def main(args=None):
    rclpy.init(args=args)
    node = DisparityExtender()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
