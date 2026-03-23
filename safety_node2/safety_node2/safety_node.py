#!/usr/bin/env python3
import math
from typing import List, Optional

import numpy as np
import numpy.typing as npt
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Float64, Bool
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """

    def __init__(self) -> None:
        super().__init__('safety_node')

        """
        One publisher should publish to the /drive_out topic with a AckermannDriveStamped drive message.
        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle,
        and to /drive_in to get the drive commands.
        The subscribers should use the provided odom_callback and scan_callback as callback methods
        """

        self.speed: float = 0.
        self.brake_engaged: bool = False
        self.current_ttc: float = float('inf')

        self.declare_parameter('ttc_threshold', 0.7)
        self.add_on_set_parameters_callback(self._on_param_change)

        # subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.drive_in_sub = self.create_subscription(
            AckermannDriveStamped, '/drive_in', self.drive_callback, 10)

        # publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive_out', 10)
        self.ttc_pub = self.create_publisher(Float64, '/diagnostics/ttc', 10)
        self.min_dist_pub = self.create_publisher(Float64, '/diagnostics/min_dist', 10)
        self.brake_pub = self.create_publisher(Bool, '/diagnostics/brake_engaged', 10)
        self.marker_pub = self.create_publisher(Marker, '/diagnostics/safety_marker', 10)

        # service to reset the safety node
        self.reset_srv = self.create_service(
            Trigger, '/reset_safety_node', self.reset_callback)

        self.get_logger().info(
            f'SafetyNode started. TTC threshold = '
            f'{self._ttc_threshold:.2f} s')
    
    @property
    def _ttc_threshold(self) -> float:
        return self.get_parameter('ttc_threshold').value

    def _on_param_change(self, params: List[Parameter]) -> SetParametersResult:
        for p in params:
            if p.name == 'ttc_threshold':
                if p.value <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='ttc_threshold must be positive')
                self.ttc_threshold = p.value
                self.get_logger().info(
                    f'TTC threshold updated to {p.value:.3f} s')
        return SetParametersResult(successful=True)

    def odom_callback(self, odom_msg: Odometry) -> None:
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg: LaserScan) -> None:
        ranges, angles = self._get_valid_ranges_and_angles(scan_msg)
 
        min_dist: float = float(np.min(ranges)) if len(ranges) > 0 else float('inf')
        min_ttc: float = self._compute_min_ttc(ranges, angles)
        self.current_ttc = min_ttc
 
        local_ttc_threshold = self._ttc_threshold
        if min_ttc < local_ttc_threshold:
            if not self.brake_engaged:
                self.get_logger().warn(f'AEB engaged (TTC = {min_ttc:.3f} s < threshold {local_ttc_threshold:.3f} s)')
            self.brake_engaged = True
 
        # diagnostics
        self._publish_float64(self.ttc_pub, min_ttc)
        self._publish_float64(self.min_dist_pub, min_dist)
        self._publish_bool(self.brake_pub, self.brake_engaged)
 
        self._publish_marker()

    def _get_valid_ranges_and_angles(
        self,
        scan_msg: LaserScan,
    ) -> tuple[npt.NDArray[np.float64], npt.NDArray[np.float64]]:
        """returns (ranges, angles) arrays with invalid (range is infinite and between range_min and range_max) beams filtered out"""

        ranges: npt.NDArray[np.float64] = np.array(scan_msg.ranges, dtype=np.float64)
        angles: npt.NDArray[np.float64] = (scan_msg.angle_min + np.arange(len(ranges), dtype=np.float64) * scan_msg.angle_increment)
 
        valid: npt.NDArray[np.bool_] = (np.isfinite(ranges) & (ranges >= scan_msg.range_min) & (ranges <= scan_msg.range_max))
 
        return ranges[valid], angles[valid]

    def _compute_min_ttc(
        self,
        ranges: npt.NDArray[np.float64],
        angles: npt.NDArray[np.float64],
    ) -> float:
        if len(ranges) == 0:
            return float('inf')

        if self.speed <= 0.0:
            return float('inf')

        # Nur Frontkegel betrachten
        front_mask = np.abs(angles) <= np.deg2rad(5)
        if not np.any(front_mask):
            return float('inf')

        ranges = ranges[front_mask]
        angles = angles[front_mask]

        r_dot: npt.NDArray[np.float64] = self.speed * np.cos(angles)

        closing: npt.NDArray[np.bool_] = r_dot > 0.0
        if not np.any(closing):
            return float('inf')

        ttc_per_beam: npt.NDArray[np.float64] = ranges[closing] / r_dot[closing]
        return float(np.min(ttc_per_beam))

    def _publish_float64(self, publisher: rclpy.publisher.Publisher, value: float) -> None:
        msg = Float64()
        msg.data = value if math.isfinite(value) else -1.0
        publisher.publish(msg)

    def _publish_bool(self, publisher: rclpy.publisher.Publisher, value: bool) -> None:
        msg = Bool()
        msg.data = value
        publisher.publish(msg)

    def drive_callback(self, drive_msg: AckermannDriveStamped) -> None:
        """Pass drive commands through - or replace with brake when engaged."""
        self.get_logger().info(f'Received drive command: {drive_msg.drive.speed:.3f} m/s')
        if self.brake_engaged:
            brake_cmd = AckermannDriveStamped()
            brake_cmd.drive.speed = 0.0
            self.get_logger().warn(f'Braking: {brake_cmd.drive.speed:.3f} m/s')
            self.drive_pub.publish(brake_cmd)
        else:
            self.get_logger().info(f'Driving: {drive_msg.drive.speed:.3f} m/s')
            self.drive_pub.publish(drive_msg)

    def reset_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """
        Disengage the safety node via /reset_safety_node.
        Fails if TTC is still below threshold.
        Always succeeds (no-op) if brake was not engaged.
        """
        ttc = self.current_ttc
        threshold = self._ttc_threshold

        if ttc < threshold:
            response.success = False
            response.message = f'Reset denied: TTC ({ttc:.3f} s) is still below threshold ({threshold:.3f} s).'
            self.get_logger().warn(response.message)
        else:
            was_engaged = self.brake_engaged
            self.brake_engaged = False
            if was_engaged:
                response.success = True
                response.message = 'Safety node disengaged successfully'
            else:
                response.success = True
                response.message = f'Safety node was not engaged. TTC = {ttc:.3f} s is safe.'
            self.get_logger().info(response.message)

        return response

    def _publish_marker(self) -> None:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'ego_racecar/base_link' # makes it move with the car automatically
        marker.ns = 'safety_node'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
 
        # relative to car (+x => forward)
        marker.pose.position.x = 0.15
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.15
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.15
        marker.scale.y = 0.15
        marker.scale.z = 0.15

        if self.brake_engaged:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
        marker.color.a = 1.0

        # lifetime=0 means the marker persists until replaced
        marker.lifetime = Duration(sec=0, nanosec=0)

        self.marker_pub.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
