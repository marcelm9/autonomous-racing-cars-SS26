import os

import rclpy
from rclpy.node import Node
from ament_index_python import get_package_share_directory
from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Strategist(Node):
    def __init__(self):
        super().__init__("strategist")

        self._load_parameters()
        self._setup_ros_communications()
        self._load_strategies()
        self._initialize_state_and_timers()

    def _load_parameters(self):
        """Declares and retrieves ROS2 parameters."""
        self.declare_parameter("map_name")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("base_frame", "ego_racecar/base_link")
        self.declare_parameter("update_rate_hz", 100.0)

        self.map_name = self.get_parameter("map_name").value
        self.frame_id = self.get_parameter("frame_id").value
        self.base_frame = self.get_parameter("base_frame").value
        self.update_rate_hz = self.get_parameter("update_rate_hz").value

        self.get_logger().info(
            f"Looking up TF from {self.frame_id} to {self.base_frame}"
        )

    def _setup_ros_communications(self):
        """Initializes TF listeners and publishers."""
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, "/strategist/markers", 10)
        self.car_indicator_pub = self.create_publisher(
            Marker, "/strategist/car_indicator", 10
        )
        self.strategy_pub = self.create_publisher(String, "/strategist/strategy", 10)

    def _load_strategies(self):
        """Finds, parses, and validates the strategy checkpoints file."""
        map_strategy_path = os.path.join(
            get_package_share_directory("launch_orchestrator"),
            "maps",
            f"{self.map_name}.strategy",
        )

        if not os.path.exists(map_strategy_path):
            raise FileNotFoundError(
                f"Raceline strategy file does not exist: {map_strategy_path}"
            )

        self.checkpoints = []
        self.get_logger().info(f"Loading strategies from: {map_strategy_path}")

        with open(map_strategy_path, "r") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue  # Skip empty lines

                parts = [part.strip() for part in line.split(",")]

                if len(parts) != 5:
                    self.get_logger().warn(f"Skipping malformed line: {line}")
                    continue

                self.checkpoints.append(
                    {
                        "strategy": parts[0],
                        "p1": (float(parts[1]), float(parts[2])),
                        "p2": (float(parts[3]), float(parts[4])),
                    }
                )

        if not self.checkpoints:
            raise ValueError("No checkpoints found in strategy file.")

        self.get_logger().info(f"Loaded {len(self.checkpoints)} strategy checkpoints.")
        self._assign_strategy_colors()

    def _assign_strategy_colors(self):
        """Assigns a distinct visual color to each unique strategy."""
        palette = [
            (1.0, 0.0, 0.0),  # Red
            (0.0, 0.5, 1.0),  # Blue
            (1.0, 1.0, 0.0),  # Yellow
            (1.0, 0.0, 1.0),  # Magenta
            (0.0, 1.0, 1.0),  # Cyan
            (1.0, 0.5, 0.0),  # Orange
            (0.5, 0.0, 1.0),  # Purple
            (0.0, 1.0, 0.0),  # Green
            (1.0, 0.5, 0.5),  # Pink
            (0.5, 0.5, 1.0),  # Light Blue
        ]

        unique_strategies = list(
            dict.fromkeys(cp["strategy"] for cp in self.checkpoints)
        )

        self.strategy_colors = {
            strat: palette[i % len(palette)]
            for i, strat in enumerate(unique_strategies)
        }

    def _initialize_state_and_timers(self):
        """Sets up initial state variables and ROS2 timers."""
        self.target_index = 0
        self.prev_pos = None
        self.current_strategy = self.checkpoints[0]["strategy"]

        self.marker_timer = self.create_timer(1.0, self.publish_markers)
        
        # --- REMOVED THE CONTINUOUS 10HZ TIMER ---
        # Strategy tokens are now published purely via events in position_callback

        self.position_timer = self.create_timer(
            1.0 / self.update_rate_hz, self.position_callback
        )

    def position_callback(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.frame_id, self.base_frame, rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(
                f"Could not transform {self.frame_id} to {self.base_frame}: {ex}",
                throttle_duration_sec=2.0,
            )
            return

        curr_pos = (t.transform.translation.x, t.transform.translation.y)

        if self.prev_pos is not None:
            target_cp = self.checkpoints[self.target_index]

            if self.intersects(
                self.prev_pos, curr_pos, target_cp["p1"], target_cp["p2"]
            ):
                self.current_strategy = target_cp["strategy"]
                self.get_logger().info(f"🟥 STRATEGY UPDATED: {self.current_strategy}")
                
                # --- EVENT TRIGGER: Publish the new token immediately upon crossing ---
                self.publish_strategy()
                
                self.advance_target()

        self.prev_pos = curr_pos
        self.publish_car_indicator()

    def advance_target(self):
        self.target_index = (self.target_index + 1) % len(self.checkpoints)

    def intersects(self, p1, p2, p3, p4):
        def ccw(A, B, C):
            return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

        return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)

    def publish_strategy(self):
        """Publishes the active strategy string (Called on event triggers)."""
        msg = String(data=self.current_strategy)
        self.strategy_pub.publish(msg)

    def publish_car_indicator(self):
        """Publishes a sphere directly over the car reflecting the active strategy color."""
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "car_strategy_indicator"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = 0.3
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.15

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        r, g, b = self.strategy_colors[self.current_strategy]
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        self.car_indicator_pub.publish(marker)

    def publish_markers(self):
        """Generates and publishes RViz lines and text for the strategy checkpoints."""
        marker_array = MarkerArray()

        for i, cp in enumerate(self.checkpoints):
            r, g, b = self.strategy_colors[cp["strategy"]]

            line_marker = Marker()
            line_marker.header.frame_id = self.frame_id
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = "strategy_lines"
            line_marker.id = i
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD

            line_marker.scale.x = 0.15
            line_marker.color.r = r
            line_marker.color.g = g
            line_marker.color.b = b
            line_marker.color.a = 0.6

            line_marker.points = [
                Point(x=cp["p1"][0], y=cp["p1"][1], z=0.0),
                Point(x=cp["p2"][0], y=cp["p2"][1], z=0.0),
            ]
            marker_array.markers.append(line_marker)

            text_marker = Marker()
            text_marker.header.frame_id = self.frame_id
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "strategy_text"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = (cp["p1"][0] + cp["p2"][0]) / 2.0
            text_marker.pose.position.y = (cp["p1"][1] + cp["p2"][1]) / 2.0
            text_marker.pose.position.z = 0.1

            text_marker.scale.z = 0.2
            text_marker.color.r = 0.0
            text_marker.color.g = 0.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0
            text_marker.text = cp["strategy"]

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    strategy_publisher = Strategist()
    try:
        rclpy.spin(strategy_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        strategy_publisher.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
