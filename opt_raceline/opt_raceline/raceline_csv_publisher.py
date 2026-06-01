#!/usr/bin/env python3

import csv
import math
import os
import threading
from pathlib import Path
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node

from msgs.msg import Raceline, RacelinePoint
from nav_msgs.msg import Path as PathMsg
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from ament_index_python.packages import (
    get_package_share_directory,
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
        self.get_logger().info("Initializing RacelineCsvPublisher node...")

        self.declare_parameter("map_name", "NO MAP SPECIFIED")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate", 1.0)

        # HTTP Server configuration
        self.declare_parameter("allow_http_updates", True)
        self.declare_parameter("http_port", 8000)

        self.frame_id = self.get_parameter("frame_id").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.allow_http_updates = self.get_parameter("allow_http_updates").value
        self.http_port = self.get_parameter("http_port").value

        self.get_logger().info(
            f"Configuration: frame_id='{self.frame_id}', publish_rate={self.publish_rate}Hz, allow_http_updates={self.allow_http_updates}"
        )

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

        self.speed_marker_pub = self.create_publisher(
            MarkerArray, "/raceline/speed_markers", 10
        )

        # Initialize with empty messages to prevent crashes before data arrives
        self.raceline_msg = Raceline()
        self.raceline_msg.header.frame_id = self.frame_id
        self.path_msg = PathMsg()
        self.path_msg.header.frame_id = self.frame_id

        # --- 1. Base Initialization (Always attempt to load CSV first) ---
        map_name = self.get_parameter("map_name").value
        try:
            self.csv_path = self.resolve_csv_path(map_name)
            self.get_logger().info(
                f"Attempting to load base CSV raceline from: {self.csv_path}"
            )
            self.raceline_msg = self.load_raceline_csv(self.csv_path)
            self.path_msg = self.create_path_msg(self.raceline_msg)

            self.get_logger().info(
                f"Successfully loaded base raceline! Total points: {len(self.raceline_msg.points)}"
            )
        except Exception as exc:
            self.get_logger().error(
                f"Failed to load base raceline CSV '{map_name}': {exc}"
            )

        # --- 2. Live Update Initialization (Optional HTTP Server) ---
        if self.allow_http_updates:
            self.get_logger().info(
                f"HTTP live updates enabled. Booting up HTTP server on port {self.http_port}..."
            )
            self.start_http_server()
        else:
            self.get_logger().info(
                "HTTP live updates disabled via parameter. Running in read-only CSV mode."
            )

        # --- 3. Start standard publishing loop ---
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish,
        )

        self.get_logger().info("Publishing pipelines established:")
        self.get_logger().info(" -> /raceline/trajectory  (msgs/msg/Raceline)")
        self.get_logger().info(" -> /raceline/path        (nav_msgs/msg/Path)")
        self.get_logger().info(
            " -> /raceline/speed_markers (visualization_msgs/msg/MarkerArray)"
        )

    def resolve_csv_path(self, map_name: str) -> str:
        path = os.path.join(
            get_package_share_directory("launch_orchestrator"),
            "maps",
            f"{map_name}.csv",
        )
        if not os.path.exists(path):
            raise FileNotFoundError(f"Raceline CSV does not exist: {path}")
        return path

    def load_raceline_csv(self, csv_path: str) -> Raceline:
        path = Path(csv_path)
        if not path.exists():
            raise FileNotFoundError(f"Raceline CSV does not exist: {csv_path}")

        with path.open(newline="", encoding="utf-8") as csv_file:
            lines = csv_file.readlines()
            return self.parse_csv_lines(lines)

    def parse_csv_lines(self, lines: list) -> Raceline:
        """Core parsing logic decoupled from file I/O so it works with HTTP payloads"""
        header = None
        rows = []
        for line_number, raw_line in enumerate(lines, start=1):
            stripped = raw_line.strip()
            if not stripped:
                continue

            if stripped.startswith("#"):
                maybe_header = next(csv.reader([stripped[1:].strip()], delimiter=";"))
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
            missing = [name for name in REQUIRED_COLUMNS if name not in column_index]
            if missing:
                raise ValueError(
                    f"Raceline payload is missing required columns: {', '.join(missing)}"
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
            raise ValueError("Raceline payload contains no data rows.")
        for previous, current in zip(rows, rows[1:]):
            if current["s_m"] <= previous["s_m"]:
                raise ValueError("Raceline s_m column must be strictly increasing.")

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
        # Do not publish if we have no points yet
        if not self.raceline_msg.points:
            return

        stamp = self.get_clock().now().to_msg()

        self.raceline_msg.header.stamp = stamp
        self.path_msg.header.stamp = stamp

        for pose in self.path_msg.poses:
            pose.header.stamp = stamp

        speed_marker_array = self.create_speed_marker_array(self.raceline_msg, stamp)

        self.raceline_pub.publish(self.raceline_msg)
        self.path_pub.publish(self.path_msg)
        self.speed_marker_pub.publish(speed_marker_array)

    def create_speed_marker_array(self, raceline_msg: Raceline, stamp) -> MarkerArray:
        marker_array = MarkerArray()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        step = 30
        marker_id = 0

        for i, point in enumerate(raceline_msg.points):
            if i % step != 0:
                continue

            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = stamp
            marker.ns = "raceline_speed"
            marker.id = marker_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = point.x_m
            marker.pose.position.y = point.y_m
            marker.pose.position.z = 0.25
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.18
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.text = f"{point.vx_mps:.1f}"

            marker_array.markers.append(marker)
            marker_id += 1

        return marker_array

    # ---------------------------------------------------------
    # HTTP SERVER INTEGRATION
    # ---------------------------------------------------------
    def start_http_server(self):
        try:
            handler = self.make_http_handler()
            self.httpd = HTTPServer(("0.0.0.0", self.http_port), handler)
            # Run server in a daemon thread so it doesn't block ROS spin and dies with the node
            self.server_thread = threading.Thread(
                target=self.httpd.serve_forever, daemon=True
            )
            self.server_thread.start()
            self.get_logger().info(
                f"HTTP server successfully started on 0.0.0.0:{self.http_port}"
            )
        except Exception as e:
            self.get_logger().error(
                f"Failed to start HTTP server on port {self.http_port}: {e}"
            )

    def make_http_handler(self):
        node = self  # Capture reference to the ROS node

        class HTTPRequestHandler(BaseHTTPRequestHandler):
            def log_message(self, format, *args):
                # Redirect standard HTTP access logs to the ROS logger (as debug to avoid spam)
                node.get_logger().debug(
                    f"[HTTP] {self.client_address[0]} - {format % args}"
                )

            def do_OPTIONS(self):
                # Essential for allowing the Svelte Web App to POST data (CORS preflight)
                node.get_logger().debug(
                    f"[HTTP] Received OPTIONS (CORS preflight) from {self.client_address[0]}"
                )
                self.send_response(200)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "POST, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.end_headers()

            def do_POST(self):
                client_ip = self.client_address[0]
                node.get_logger().info(f"[HTTP] Received POST request from {client_ip}")

                content_length = int(self.headers.get("Content-Length", 0))
                node.get_logger().debug(f"[HTTP] Payload size: {content_length} bytes")

                post_data = self.rfile.read(content_length).decode("utf-8")
                lines = post_data.splitlines()

                try:
                    new_raceline_msg = node.parse_csv_lines(lines)
                    new_path_msg = node.create_path_msg(new_raceline_msg)

                    # Update node state thread-safely (reference assignment is atomic in Python)
                    node.raceline_msg = new_raceline_msg
                    node.path_msg = new_path_msg

                    node.get_logger().info(
                        f"[HTTP] Successfully updated live raceline! New point count: {len(new_raceline_msg.points)}"
                    )

                    self.send_response(200)
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(b"Raceline successfully updated.")

                except Exception as e:
                    node.get_logger().error(
                        f"[HTTP] Failed to parse raceline from {client_ip}: {e}"
                    )
                    self.send_response(400)
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    self.wfile.write(f"Error parsing raceline: {e}".encode("utf-8"))

        return HTTPRequestHandler

    def destroy_node(self):
        # Clean up HTTP server resources when node is shut down
        self.get_logger().info("Shutting down node...")
        if hasattr(self, "httpd"):
            self.get_logger().info("Stopping HTTP server...")
            self.httpd.shutdown()
        super().destroy_node()
        self.get_logger().info("Node successfully destroyed.")


def main(args=None):
    rclpy.init(args=args)

    node = RacelineCsvPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
