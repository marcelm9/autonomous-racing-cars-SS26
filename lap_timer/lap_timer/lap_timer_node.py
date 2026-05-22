import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import json


class LapTimerNode(Node):
    def __init__(self):
        super().__init__("lap_timer_node")

        # 1. Declare parameters
        self.declare_parameter(
            "track_layout",
            '{"start": [2.0, 2.0, -2.0, -2.0], "checkpoints": []}',
        )

        self.declare_parameter(
            "frame_id",
            "map",
        )

        self.declare_parameter(
            "base_frame",
            "/ego_racecar/odom",
        )

        # 2. Read parameters
        layout_str = self.get_parameter("track_layout").value
        self.frame_id = self.get_parameter("frame_id").value
        self.odom_topic = self.get_parameter("base_frame").value

        layout = json.loads(layout_str)

        self.get_logger().info(f"Subscribing to odometry topic: {self.odom_topic}")

        # 3. Subscriptions and Publishers
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/track_markers",
            10,
        )

        # 4. Parse coordinates into lines
        self.lines = []

        sl = layout["start"]
        self.lines.append(((sl[0], sl[1]), (sl[2], sl[3])))

        for cp in layout.get("checkpoints", []):
            self.lines.append(((cp[0], cp[1]), (cp[2], cp[3])))

        self.get_logger().info(
            f"Loaded track with 1 Start Line and {len(self.lines) - 1} Checkpoints."
        )

        # 5. State Variables
        self.target_index = 0
        self.lap_started = False
        self.prev_pos = None
        self.last_lap_time = self.get_clock().now()
        self.sector_start_time = self.get_clock().now()
        self.lap_count = 1
        self.min_lap_time = 3.0

        # 6. Timer for RViz markers
        self.marker_timer = self.create_timer(
            1.0,
            self.publish_markers,
        )

    def odom_callback(self, msg):
        curr_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        if self.prev_pos is not None:
            target_line = self.lines[self.target_index]

            if self.intersects(self.prev_pos, curr_pos, target_line[0], target_line[1]):
                current_time = self.get_clock().now()

                if self.target_index == 0:
                    if self.lap_started:
                        lap_time = (current_time - self.last_lap_time).nanoseconds / 1e9
                        if lap_time > self.min_lap_time:
                            sector_time = (
                                current_time - self.sector_start_time
                            ).nanoseconds / 1e9
                            if len(self.lines) > 0:
                                # at least one checkpoint exists, so print sector time
                                self.get_logger().info(
                                    f"🟦 LAP {self.lap_count:02d} CP {len(self.lines)}/"
                                    + f"{len(self.lines)} {sector_time:.3f} s"
                                )
                            self.get_logger().info(
                                f"🔳 LAP {self.lap_count:02d} FINISH {lap_time:.3f} s"
                            )
                            self.last_lap_time = current_time
                            self.sector_start_time = current_time
                            self.lap_count += 1
                            self.advance_target()
                    else:
                        self.get_logger().info("🟩 START")
                        self.lap_started = True
                        self.last_lap_time = current_time
                        self.sector_start_time = current_time
                        self.advance_target()

                else:
                    sector_time = (
                        current_time - self.sector_start_time
                    ).nanoseconds / 1e9
                    self.get_logger().info(
                        f"🟦 LAP {self.lap_count:02d} CP {self.target_index}/"
                        + f"{len(self.lines)} {sector_time:.3f} s"
                    )
                    self.sector_start_time = current_time
                    self.advance_target()

        self.prev_pos = curr_pos

    def advance_target(self):
        self.target_index += 1
        if self.target_index >= len(self.lines):
            self.target_index = 0

    def intersects(self, p1, p2, p3, p4):
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

        return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)

    def publish_markers(self):
        """Generates and publishes RViz lines and text for the start and checkpoints."""
        marker_array = MarkerArray()

        for i, line in enumerate(self.lines):
            # --- 1. Create the Line Marker ---
            line_marker = Marker()
            line_marker.header.frame_id = self.frame_id
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "lap_timer_lines"
            line_marker.id = i
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD

            # Line width thickness
            line_marker.scale.x = 0.15

            # Color configuration
            if i == 0:
                # Bright Green for Start Line
                line_marker.color.r = 0.0
                line_marker.color.g = 1.0
                line_marker.color.b = 0.0
                line_marker.color.a = 1.0
            else:
                # Cyan/Blue for Checkpoints
                line_marker.color.r = 0.0
                line_marker.color.g = 0.6
                line_marker.color.b = 1.0
                line_marker.color.a = 0.8

            # Define line endpoints
            p1 = Point(x=float(line[0][0]), y=float(line[0][1]), z=0.0)
            p2 = Point(x=float(line[1][0]), y=float(line[1][1]), z=0.0)
            line_marker.points = [p1, p2]

            marker_array.markers.append(line_marker)

            # --- 2. Create the Text Marker ---
            text_marker = Marker()
            text_marker.header.frame_id = self.frame_id
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "lap_timer_text"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            # Calculate midpoint for text placement
            mid_x = (line[0][0] + line[1][0]) / 2.0
            mid_y = (line[0][1] + line[1][1]) / 2.0

            text_marker.pose.position.x = float(mid_x)
            text_marker.pose.position.y = float(mid_y)
            text_marker.pose.position.z = 0.1

            # Set text scale (height of the uppercase letters)
            text_marker.scale.z = 0.2

            # Make text pure white
            text_marker.color.r = 0.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0

            if i == 0:
                text_marker.text = f"START/CP-{len(self.lines)}"
            else:
                text_marker.text = f"CP-{i}"

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    lap_timer = LapTimerNode()
    rclpy.spin(lap_timer)
    lap_timer.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
