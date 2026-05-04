#!/usr/bin/env python3
# simple_raceline.py
"""A very simple algorithm for computing a simple raceline for a given map.

Created for ARC 2025 course.
Refactored to separate ROS node logic from the core algorithm.
"""

######################
# Imports & Globals
######################

import sys
import time
from typing import (
    List,
    Optional,
    Tuple,
)

import numpy

# ROS2 client library
import rclpy

# ROS messages
from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
)
from nav_msgs.msg import (
    OccupancyGrid,
    Path,
)
from rcl_interfaces.msg import (
    IntegerRange,
    ParameterDescriptor,
    ParameterType,
)
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
)

from simple_raceline.algorithms import (
    compute_centered_raceline,
    compute_simple_raceline,
)

######################
# RacelineNode
######################


class RacelineNode(Node):
    """A ROS2 node to compute a simple raceline."""

    def __init__(self, name: str = "simple_raceline", **kwargs) -> None:
        """Initialize the node."""
        super(RacelineNode, self).__init__(name, **kwargs)

        # Attributes
        self._origin: Tuple[float, float]
        self._safety: int
        self._sparse: int
        self._closed_loop: bool

        # Parameters
        self.declare_parameter(
            "safety",
            5,
            ParameterDescriptor(
                name="safety",
                type=ParameterType.PARAMETER_INTEGER,
                description="[-] amount of pixels eroded from the driveable area",
                integer_range=[IntegerRange(from_value=1, to_value=50, step=1)],
            ),
        )
        self._safety = self.get_parameter("safety").value

        self.declare_parameter(
            "sparse",
            5,
            ParameterDescriptor(
                name="sparse",
                type=ParameterType.PARAMETER_INTEGER,
                description="[-] 1 / sampling ratio for the final raceline",
                integer_range=[IntegerRange(from_value=1, to_value=50, step=1)],
            ),
        )
        self._sparse = self.get_parameter("sparse").value

        self.declare_parameter(
            "closed_loop",
            False,
            ParameterDescriptor(
                name="closed_loop",
                type=ParameterType.PARAMETER_BOOL,
                description="when True, the path is created as a closed loop",
            ),
        )
        self._closed_loop = self.get_parameter("closed_loop").value

        self.declare_parameter(
            "algorithm",
            "simple",
            ParameterDescriptor(
                name="algorithm",
                type=ParameterType.PARAMETER_STRING,
                description="algorithm to use [options: 'simple', 'centered']",
            ),
        )
        self._algorithm = self.get_parameter("algorithm").value
        self._algorithms = {
            "simple": compute_simple_raceline,
            "centered": compute_centered_raceline,
        }
        self.get_logger().info(f"Using algorithm: {self._algorithm}")

        # Subscribers
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self.callback_map,
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        # Publishers
        self.pub_path = self.create_publisher(
            Path,
            "/path",
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        self.pub_costmap = self.create_publisher(
            OccupancyGrid,
            "/map/costmap",
            QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

    def callback_map(self, msg: OccupancyGrid) -> None:
        """Obtain the map, run the algorithm, and publish the results."""
        start_time = time.time()

        self._origin = (
            msg.info.origin.position.x,
            msg.info.origin.position.y,
        )

        self.get_logger().debug(
            f"Obtained OccupancyGrid message: height: {msg.info.height}, "
            f"width: {msg.info.width}, origin: ({self._origin[0]}, {self._origin[1]}, "
            f"{msg.info.origin.position.z})"
        )

        # 1. Preprocess the map data into standard NumPy format
        height = msg.info.height
        width = msg.info.width
        resolution = msg.info.resolution

        map_grid = numpy.asarray(msg.data, dtype=bool).reshape((height, width))

        # 2. Call the Decoupled Algorithm
        sparse_path, costmap = self._algorithms[self._algorithm](
            grid=map_grid,
            origin_x=self._origin[0],
            origin_y=self._origin[1],
            resolution=resolution,
            safety=self._safety,
            sparse=self._sparse,
            closed_loop=self._closed_loop,
        )

        # 3. Publish the resulting path
        p = Path()
        p.header.frame_id = "map"
        for r, c in sparse_path:
            p.poses.append(
                PoseStamped(
                    pose=Pose(
                        position=Point(
                            x=c * resolution + self._origin[0],
                            y=r * resolution + self._origin[1],
                        ),
                    ),
                ),
            )

        self.pub_path.publish(p)

        self.get_logger().info(
            f"Raceline published in {time.time() - start_time:.3f} seconds."
        )

        # 4. Publish the internal costmap
        m = msg
        _data = numpy.ravel(costmap)
        _data[_data == numpy.inf] = 0
        _data /= max(_data)  # Normalize to...
        _data *= 98  # ... 0-98
        m.data = _data.astype(numpy.uint8).tolist()
        self.pub_costmap.publish(m)


######################
# Main
######################


def main(args: Optional[List[str]] = None) -> None:
    """Start the node."""
    rclpy.init(args=args)

    n = RacelineNode()
    rclpy.spin(n)

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
