#!/usr/bin/env python3
# simple_raceline.py
"""A very simple algorithm for computing a simple raceline for a given map.

Created for ARC 2025 course.
"""
######################
# Imports & Globals
######################

import sys
import time

import numpy

from skimage.segmentation import flood_fill
from skimage.morphology import binary_erosion

from typing import (
    List,
    Optional,
    Tuple,
)

# ROS2 client library
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    DurabilityPolicy,
)

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


######################
# RacelineNode
######################

class RacelineNode(Node):
    """A ROS2 node to compute a simple raceline.

    Note that we consider initial + map orientation (0, 0, 0, 1).
    """

    def __init__(self, name: str = "simple_raceline", **kwargs) -> None:
        """Initialize the node.

        Arguments
        ---------
        name: str
            Name of the ROS2 node.
        """
        super(RacelineNode, self).__init__(name, **kwargs)


        # Attributes
        self._map: OccupancyGrid
        self._origin: Tuple[float, float]
        self._safety: int
        self._sparse: int

        # Parameters
        self.declare_parameter(
            "safety", 5, ParameterDescriptor(
                name = "safety",
                type = ParameterType.PARAMETER_INTEGER,
                description = (
                    "[-] amount of pixels eroded from the driveable area"
                ),
                integer_range = [
                    IntegerRange(
                        from_value = 1,
                        to_value = 50,
                        step = 1,
                    ),
                ],
            )
        )
        self._safety = self.get_parameter("safety").value

        self.declare_parameter(
            "sparse", 5, ParameterDescriptor(
                name = "sparse",
                type = ParameterType.PARAMETER_INTEGER,
                description = (
                    "[-] 1 / sampling ratio for the final raceline"
                ),
                integer_range = [
                    IntegerRange(
                        from_value = 1,
                        to_value = 50,
                        step = 1,
                    ),
                ],
            )
        )
        self._sparse = self.get_parameter("sparse").value

        self.declare_parameter(
            "closed_loop", False, ParameterDescriptor(
                name = "closed_loop",
                type = ParameterType.PARAMETER_BOOL,
                description = (
                    "when True, the path is created as a closed loop"
                ),
            )
        )
        self._closed_loop = self.get_parameter("closed_loop").value


        # Subscribers
        self.create_subscription(
            OccupancyGrid, "/map", self.callback_map, QoSProfile(
                depth = 1,
                durability = DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )


        # Publishers
        self.pub_path = self.create_publisher(
            Path, "/path", QoSProfile(
                depth = 1,
                durability = DurabilityPolicy.TRANSIENT_LOCAL,
            )
        )

        self.pub_costmap = self.create_publisher(
            OccupancyGrid, "/map/costmap", QoSProfile(
                depth = 1,
                durability = DurabilityPolicy.TRANSIENT_LOCAL,
            )
        )


    def callback_map(self, msg: OccupancyGrid) -> None:
        """Obtain the map and compute the raceline.

        Arguments
        ---------
        msg: OccupancyGrid
            Message containing the map of the environment.
        """
        start_time = time.time()

        self.get_logger().debug(
            "Obtained OccupancyGrid message: "
            f"height: {msg.info.height}, "
            f"width: {msg.info.width}, "
            "origin: ("
            f"{msg.info.origin.position.x}, "
            f"{msg.info.origin.position.y}, "
            f"{msg.info.origin.position.z}"
            ")"
        )

        # 1. Preprocess the map data
        height = msg.info.height
        width = msg.info.width
        self._map = numpy.asarray(msg.data, dtype = bool).reshape(
            (height, width)
        )
        # Note that for map, True = wall.

        self._origin = (
            msg.info.origin.position.x, msg.info.origin.position.y,
        )


        # Intermezzo: Convert starting point coordinates
        # (this is used when loading an image directly)
        """
               +c  (numpy column, 2nd axis)
            +------>
            |
         +r |  (numpy row, 1st axis)
            |
            v

                        ^
                        |
                        |
                        O----->  (true map origin)

            ^
            |
         +y |  (map y, 2nd axis)
            |
            +------>
               +x  (map x, 1st axis)
        (origin)
        """

        # (this is used when obtaining the image from an occupancy_grid msg)
        """
                        ^
                        |
                        |
                        O----->  (true map origin)

            ^
            |
         +y |  (map y, 2nd axis) = +r (numpy row, 1st axis)
            |
            +------>
               +x  (map x, 1st axis) = +c (numpy column, 2nd axis)
        (origin)
        """

        # 2. Obtain the driveable area
        res = msg.info.resolution
        xc = int(-self._origin[0] / res)
        yr = int(-self._origin[1] / res)

        driveable_area = ~numpy.bitwise_and(
            # `flood_fill` returns changed image...
            flood_fill(self._map, (yr, xc), 1),
            # ... so we take only the cells that are changed.
            self._map
        )
        # Alternatively, we can work with >2-value data structure.
        # Note that for driveable_area, True = free space
        # ^^ This change is done in order to use `binary_erosion` later.
        # ... also it makes flood filling slightly easier.


        # 3. Obtain the finish line
        """
        ---------------------------
                    |              ---
                 +y |                 ----
                    |                     ---
                    O----->
                    |  +x
                    |
                    |
        ---------------------------
                                   ---
                                      ----
                                          ---
        """
        # Here, we use `argmin` to find the first occurence of a wall (False),
        # while "looking" along the y-axis from the starting point.
        finish_left_yr = yr - numpy.argmin(driveable_area[:yr, xc][::-1]) + 1
        finish_right_yr = yr + numpy.argmin(driveable_area[yr:, xc]) - 1

        # We do it again for a (x-1) shifted point to obtain "pre-finish line",
        # i.e., the initial set of nodes
        if not self._closed_loop:
            prefinish_left_yr = yr - numpy.argmin(driveable_area[:yr, xc - 1][::-1]) + 1  # noqa: E501
            prefinish_right_yr = yr + numpy.argmin(driveable_area[yr:, xc - 1]) - 1  # noqa: E501
        else:
            prefinish_left_yr = prefinish_right_yr = yr

        # 4. Erode the driveable_area by n_safety pixels
        save_area = ~binary_erosion(
            image = driveable_area,
            footprint = [
                (numpy.ones((2 * self._safety + 1, 1)), 1),
                (numpy.ones((1, 2 * self._safety + 1)), 1),
            ],
        )
        # Note that for save_area, True = wall


        # 5. Calculate distance for each point in the save_area
        costmap = numpy.ones_like(save_area, dtype = float) * numpy.inf

        # Initialize the costmap
        costmap[finish_left_yr:finish_right_yr + 1, xc] = 0
        costmap[prefinish_left_yr:prefinish_right_yr + 1, xc - 1] = 1

        # Initialize start nodes
        nodes = [
            (_y, xc - 1)
            for _y in range(prefinish_left_yr, prefinish_right_yr + 1)
        ]

        while nodes:  # effectively works as len(nodes) > 0
            _r, _c = nodes.pop(0)
            _cost = costmap[_r, _c]

            for _ri, _ci in [(-1, 0), (0, -1), (0, 1), (1, 0)]:
                _rc = (_r + _ri, _c + _ci)

                # Skip if outside of the map
                # Note that this can be also resolved with try ... catch
                # as it should be faster. (e.g., as shown later)
                # Or, if you are brave enough, or you are aware of possible
                # risks, just skip this condition entirely.
                if min(_rc) < 0 or _rc[0] >= height or _rc[1] >= width:
                    continue

                # Skip if the cell contains a wall.
                if save_area[_rc]:
                    continue

                if costmap[_rc] > _cost + 1 and costmap[_rc] > 0:
                    nodes.append(_rc)
                    costmap[_rc] = _cost + 1


        # 6. Create a pixel path from the starting point
        path = [(yr, xc)]

        # We employ a 8-hood here.
        _area = numpy.asarray([
            (-1, -1), (-1, 0), (-1, 1),
             (0, -1),           (0, 1),  # noqa: E131, E241
             (1, -1), (1, -1),  (1, 1)   # noqa: E131, E241
        ])

        # We separately compute the first two points to reduce the amount
        # of conditions when constructing the path.
        for _ in range(2):
            _hood = _area + path[-1]

            _values = costmap[_hood[:, 0], _hood[:, 1]]

            # Throw away all low numbers. (originally only 0s and 1s were
            # thrown away but others are required as well)
            _values[_values < 3] = numpy.nan

            path.append(
                tuple(
                    _hood[
                        numpy.nanargmin(_values)
                    ]
                )
            )


        # And now we continue further with creating the path.
        current_cost = costmap[path[-1]]

        # Until we reach the "start" of the path...
        while current_cost > 1:
            _hood = _area + path[-1]

            try:
                # Another way to handle index checks. Just try it, if it fails
                # construct it "slowly".
                _values = costmap[_hood[:, 0], _hood[:, 1]]
            except IndexError:
                _hood_list = []

                for _r, _c in _hood.tolist():
                    if _r < 0 or _c < 0 or _r >= height or _c >= width:
                        continue
                    else:
                        _hood_list.append((_r, _c))

                _hood = numpy.asarray(_hood_list)
                _values = costmap[_hood[:, 0], _hood[:, 1]]

            # We decided to use tuples, as array[tuple] returns directly
            # the value inside. Try that with array[list].
            path.append(
                tuple(_hood[numpy.argmin(_values)])
            )
            current_cost = costmap[path[-1]]


        # 7. Increase the point sparsity
        # Note that this method is not perfect; the space between the last
        # and the first point might be different.
        sparse_path = path[::self._sparse]


        # 8. Publish the resulting path
        p = Path()
        p.header.frame_id = "map"
        for r, c in sparse_path:
            p.poses.append(
                PoseStamped(
                    # Note that header is usually not used here. That holds
                    # for frame_id. In theory, timestamp could be leveraged to,
                    # e.g., storing time progress along the path.
                    pose = Pose(
                        position = Point(
                            x = c * res + self._origin[0],
                            y = (r) * res + self._origin[1],
                        ),
                        # Similar situation is with orientation. For "simple"
                        # algorithms, such as Pure Pursuit, the orientation
                        # of the path points is really not needed.
                    ),
                ),
            )

        self.pub_path.publish(p)

        self.get_logger().info(
            f"Raceline published in {time.time() - start_time:.3f} seconds."
        )


        # Publish the internal costmap
        m = msg
        _data = numpy.ravel(costmap)
        _data[_data == numpy.inf] = 0
        _data /= max(_data)  # Normalize to...
        _data *= 98          # ... 0-98 (check the costmap value meaning)
        m.data = _data.astype(numpy.uint8).tolist()
        self.pub_costmap.publish(m)


######################
# Main
######################

def main(args: Optional[List[str]] = None) -> None:
    """Start the node."""
    rclpy.init(args = args)

    n = RacelineNode()
    rclpy.spin(n)

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
