import os

import rclpy
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Point
from std_msgs.msg import String
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class Strategist(Node):
    def __init__(self):
        super().__init__("strategist")

        # 1. Declare parameters
        self.declare_parameter("map_name")

        map_strategy_path = os.path.join(
            get_package_share_directory("launch_orchestrator"),
            "maps",
            f"{self.get_parameter('map_name').value}.strategy",
        )
        if not os.path.exists(map_strategy_path):
            raise FileNotFoundError(
                f"Raceline strategy file does not exist: {map_strategy_path}"
            )

        self.declare_parameter(
            "frame_id",
            "map",
        )

        self.declare_parameter(
            "base_frame",
            "ego_racecar/base_link",
        )

        self.declare_parameter(
            "update_rate_hz",
            100.0,
        )

        # 2. Read parameters
        self.frame_id = self.get_parameter("frame_id").value
        self.base_frame = self.get_parameter("base_frame").value
        update_rate_hz = self.get_parameter("update_rate_hz").value

        self.get_logger().info(
            f"Looking up TF from {self.frame_id} to {self.base_frame}"
        )

        # 3. Setup TF Listener & Publishers
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/strategist/markers",
            10,
        )

        self.strategy_pub = self.create_publisher(
            String,
            "/strategist/strategy",
            10,
        )

        # 4. Parse comma-separated lines into dictionaries
        self.checkpoints = []
        self.get_logger().info(f"Loading strategies from: {map_strategy_path}")

        with open(map_strategy_path, "r") as f:
            for line in [l for l in f.readlines() if l.strip() != ""]:
                parts = line.strip().split(",")

                if len(parts) != 5:
                    self.get_logger().warn(f"Skipping malformed line: {line.strip()}")
                    continue

                strategy_name = parts[0].strip()

                self.checkpoints.append(
                    {
                        "strategy": strategy_name,
                        "p1": (float(parts[1].strip()), float(parts[2].strip())),
                        "p2": (float(parts[3].strip()), float(parts[4].strip())),
                    }
                )

        if len(self.checkpoints) == 0:
            raise ValueError("No checkpoints found in strategy file.")

        self.get_logger().info(f"Loaded {len(self.checkpoints)} strategy checkpoints.")

        # 5. State Variables
        self.target_index = 0
        self.prev_pos = None
        self.current_strategy = self.checkpoints[0]["strategy"]

        # 6. Timers
        self.marker_timer = self.create_timer(
            1.0,
            self.publish_markers,
        )

        self.strategy_timer = self.create_timer(
            0.1,
            self.publish_strategy,
        )

        self.position_timer = self.create_timer(
            1.0 / update_rate_hz, self.position_callback
        )

    def position_callback(self):
        try:
            # Look up the latest transform between map and base_link
            t = self.tf_buffer.lookup_transform(
                self.frame_id, self.base_frame, rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"Could not transform {self.frame_id} to {self.base_frame}: {ex}",
                throttle_duration_sec=2.0,
            )
            return

        # Extract X and Y position from the transform
        curr_pos = (t.transform.translation.x, t.transform.translation.y)

        if self.prev_pos is not None:
            target_cp = self.checkpoints[self.target_index]

            if self.intersects(
                self.prev_pos, curr_pos, target_cp["p1"], target_cp["p2"]
            ):

                # Update current strategy and print it
                self.current_strategy = target_cp["strategy"]
                self.get_logger().info(
                    f"🟥 STRATEGY UPDATED: {self.current_strategy} (Crossed line {self.target_index})"
                )

                self.advance_target()

        self.prev_pos = curr_pos

    def advance_target(self):
        self.target_index += 1
        if self.target_index >= len(self.checkpoints):
            self.target_index = 0

    def intersects(self, p1, p2, p3, p4):
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

        return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)

    def publish_strategy(self):
        """Continually publishes the active strategy string."""
        msg = String()
        msg.data = self.current_strategy
        self.strategy_pub.publish(msg)

    def publish_markers(self):
        """Generates and publishes RViz lines and text for the strategy checkpoints."""
        marker_array = MarkerArray()

        for i, cp in enumerate(self.checkpoints):
            # --- 1. Create the Line Marker ---
            line_marker = Marker()
            line_marker.header.frame_id = self.frame_id
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "strategy_lines"
            line_marker.id = i
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD

            # Line width thickness
            line_marker.scale.x = 0.15

            # Color configuration (Highlight current target line)
            if i == self.target_index:
                # Bright Green for the Active Target Line
                line_marker.color.r = 0.0
                line_marker.color.g = 1.0
                line_marker.color.b = 0.0
                line_marker.color.a = 1.0
            else:
                # Cyan/Blue for Inactive Checkpoints
                line_marker.color.r = 0.0
                line_marker.color.g = 0.6
                line_marker.color.b = 1.0
                line_marker.color.a = 0.8

            # Define line endpoints
            p1 = Point(x=cp["p1"][0], y=cp["p1"][1], z=0.0)
            p2 = Point(x=cp["p2"][0], y=cp["p2"][1], z=0.0)
            line_marker.points = [p1, p2]

            marker_array.markers.append(line_marker)

            # --- 2. Create the Text Marker ---
            text_marker = Marker()
            text_marker.header.frame_id = self.frame_id
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "strategy_text"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            # Calculate midpoint for text placement
            mid_x = (cp["p1"][0] + cp["p2"][0]) / 2.0
            mid_y = (cp["p1"][1] + cp["p2"][1]) / 2.0

            text_marker.pose.position.x = float(mid_x)
            text_marker.pose.position.y = float(mid_y)
            text_marker.pose.position.z = 0.1

            # Set text scale (height of the uppercase letters)
            text_marker.scale.z = 0.2

            # Make text pure white
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            # Set the text to the strategy name from the file
            text_marker.text = cp["strategy"]

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    strategy_publisher = Strategist()
    rclpy.spin(strategy_publisher)
    strategy_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
