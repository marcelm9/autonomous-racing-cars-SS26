#!/usr/bin/env python3

import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node

from msgs.msg import Raceline, RacelinePoint
from nav_msgs.msg import Path as PathMsg
from geometry_msgs.msg import PoseStamped

from ament_index_python.packages import (
    get_package_share_directory,
    PackageNotFoundError,
)

REQUIRED_COLUMNS = (
    "s_m",
    "x_m",
    "y_m",
    "psi_rad",
    "kappa_radpm",
    "vx_mps",
    "ax_mps2",
)


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return 0.0, 0.0, qz, qw


class RacelineCsvPublisher(Node):
    def __init__(self):
        super().__init__("raceline_csv_publisher")

        self.declare_parameter("csv_path")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate", 1.0)

        self.csv_path = self.resolve_csv_path(self.get_parameter("csv_path").value)
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

        try:
            self.raceline_msg = self.load_raceline_csv(self.csv_path)
            self.path_msg = self.create_path_msg(self.raceline_msg)
        except Exception as exc:
            self.get_logger().error(
                f"Failed to load raceline CSV '{self.csv_path}': {exc}"
            )
            self.raceline_msg = Raceline()
            self.raceline_msg.header.frame_id = self.frame_id
            self.path_msg = PathMsg()
            self.path_msg.header.frame_id = self.frame_id

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish,
        )

        if self.raceline_msg.points:
            self.get_logger().info(f"Loaded raceline CSV: {self.csv_path}")
        self.get_logger().info(
            f"Number of raceline points: {len(self.raceline_msg.points)}"
        )
        self.get_logger().info("Publishing:")
        self.get_logger().info("  /raceline/trajectory  msgs/msg/Raceline")
        self.get_logger().info("  /raceline/path        nav_msgs/msg/Path")

    def resolve_csv_path(self, csv_path: str) -> str:
        path = Path(csv_path)
        candidates = []

        # 1. Search in launch_orchestrator -> maps
        orchestrator_share = Path(get_package_share_directory("launch_orchestrator"))
        map_dir = orchestrator_share / "maps"

        # If user passes exactly what's there (e.g., "map1.csv" or "map1/traj.csv")
        candidates.append(map_dir / path)

        # If user passes just "map1" without the extension, append .csv
        if not path.suffix:
            candidates.append(map_dir / f"{path}.csv")

        for candidate in candidates:
            if candidate.exists():
                return str(candidate)

        return csv_path

    def load_raceline_csv(self, csv_path: str) -> Raceline:
        path = Path(csv_path)
        if not path.exists():
            raise FileNotFoundError(f"Raceline CSV does not exist: {csv_path}")

        header = None
        rows = []
        with path.open(newline="") as csv_file:
            for line_number, raw_line in enumerate(csv_file, start=1):
                stripped = raw_line.strip()
                if not stripped:
                    continue

                if stripped.startswith("#"):
                    maybe_header = next(
                        csv.reader([stripped[1:].strip()], delimiter=";")
                    )
                    maybe_header = [field.strip() for field in maybe_header]
                    if (
                        "s_m" in maybe_header
                        and "x_m" in maybe_header
                        and "y_m" in maybe_header
                    ):
                        header = maybe_header
                    continue

                fields = next(csv.reader([stripped], delimiter=";"))
                fields = [field.strip() for field in fields]

                if (
                    header is None
                    and "s_m" in fields
                    and "x_m" in fields
                    and "y_m" in fields
                ):
                    header = fields
                    continue

                if header is None:
                    if len(fields) < len(REQUIRED_COLUMNS):
                        raise ValueError(
                            f"Line {line_number}: expected at least "
                            f"{len(REQUIRED_COLUMNS)} semicolon-separated columns."
                        )
                    header = list(REQUIRED_COLUMNS)

                column_index = {name: index for index, name in enumerate(header)}
                missing = [
                    name for name in REQUIRED_COLUMNS if name not in column_index
                ]
                if missing:
                    raise ValueError(
                        f"Raceline CSV is missing required columns: {', '.join(missing)}"
                    )

                try:
                    rows.append(
                        {
                            name: float(fields[column_index[name]])
                            for name in REQUIRED_COLUMNS
                        }
                    )
                except (IndexError, ValueError) as exc:
                    raise ValueError(
                        f"Line {line_number}: malformed raceline row."
                    ) from exc

        if not rows:
            raise ValueError(f"Raceline CSV contains no data rows: {csv_path}")
        for previous, current in zip(rows, rows[1:]):
            if current["s_m"] <= previous["s_m"]:
                raise ValueError("Raceline CSV s_m column must be strictly increasing.")

        msg = Raceline()
        msg.header.frame_id = self.frame_id

        for row in rows:
            point = RacelinePoint()
            point.s_m = row["s_m"]
            point.x_m = row["x_m"]
            point.y_m = row["y_m"]
            point.psi_rad = row["psi_rad"]
            point.kappa_radpm = row["kappa_radpm"]
            point.vx_mps = row["vx_mps"]
            point.ax_mps2 = row["ax_mps2"]

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
