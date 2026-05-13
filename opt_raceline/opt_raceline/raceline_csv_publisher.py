#!/usr/bin/env python3

import math
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node

from msgs.msg import Raceline, RacelinePoint

from nav_msgs.msg import Path as PathMsg
from geometry_msgs.msg import PoseStamped


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw


class RacelineCsvPublisher(Node):
    def __init__(self):
        super().__init__("raceline_csv_publisher")

        self.declare_parameter(
            "csv_path",
            "/arc2026/ws/src/your_code/opt_raceline/opt_raceline_python/outputs/crazy_map/traj_race_cl.csv",
        )
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate", 1.0)

        self.csv_path = self.get_parameter("csv_path").value
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        self.raceline_pub = self.create_publisher(
            Raceline,
            "/raceline/trajectory",
            10,
        )

        self.path_pub = self.create_publisher(
            PathMsg,
            "/raceline/path",
            10,
        )

        self.raceline_msg = self.load_raceline_csv(self.csv_path)
        self.path_msg = self.create_path_msg(self.raceline_msg)

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish,
        )

        self.get_logger().info(f"Loaded raceline CSV: {self.csv_path}")
        self.get_logger().info(f"Number of raceline points: {len(self.raceline_msg.points)}")
        self.get_logger().info("Publishing:")
        self.get_logger().info("  /raceline/trajectory  msgs/msg/Raceline")
        self.get_logger().info("  /raceline/path        nav_msgs/msg/Path")

    def load_raceline_csv(self, csv_path: str) -> Raceline:
        path = Path(csv_path)

        if not path.exists():
            raise FileNotFoundError(f"Raceline CSV does not exist: {csv_path}")

        data = np.loadtxt(
            str(path),
            delimiter=";",
            comments="#",
        )

        if data.ndim == 1:
            data = data.reshape(1, -1)

        if data.shape[1] < 7:
            raise ValueError(
                f"Expected CSV with 7 columns: "
                f"s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2. "
                f"Got {data.shape[1]} columns."
            )

        msg = Raceline()
        msg.header.frame_id = self.frame_id

        for row in data:
            point = RacelinePoint()
            point.s_m = float(row[0])
            point.x_m = float(row[1])
            point.y_m = float(row[2])
            point.psi_rad = float(row[3])
            point.kappa_radpm = float(row[4])
            point.vx_mps = float(row[5])
            point.ax_mps2 = float(row[6])

            msg.points.append(point)

        return msg

    def create_path_msg(self, raceline_msg: Raceline) -> PathMsg:
        path_msg = PathMsg()
        path_msg.header.frame_id = self.frame_id

        for point in raceline_msg.points:
            pose = PoseStamped()
            pose.header.frame_id = self.frame_id

            pose.pose.position.x = point.x_m
            pose.pose.position.y = point.y_m
            pose.pose.position.z = 0.0

            qx, qy, qz, qw = yaw_to_quaternion(point.psi_rad)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            path_msg.poses.append(pose)

        return path_msg

    def publish(self):
        stamp = self.get_clock().now().to_msg()

        self.raceline_msg.header.stamp = stamp
        self.path_msg.header.stamp = stamp

        for pose in self.path_msg.poses:
            pose.header.stamp = stamp

        self.raceline_pub.publish(self.raceline_msg)
        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)

    node = RacelineCsvPublisher()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()