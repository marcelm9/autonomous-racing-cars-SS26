import math
from random import randint
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class Visualizer:

    def __init__(self, node, publisher):
        self._node = node
        self._publisher = publisher

        self._markers = MarkerArray()
        self._marker_id = 0

    def publish(self, batch_size=100):
        # DELETEALL first
        clear = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        clear.markers.append(m)
        self._node.gap_publisher.publish(clear)

        # Publish in chunks
        all_markers = self._markers.markers
        for i in range(0, len(all_markers), batch_size):
            chunk = MarkerArray()
            chunk.markers = all_markers[i:i + batch_size]
            self._node.gap_publisher.publish(chunk)

        self._markers = MarkerArray()
        self._marker_id = 0

    # def publish(self):
    #     self._node.get_logger().info(f"Publishing {len(self._markers.markers)} markers")
    #     clear = MarkerArray()
    #     m = Marker()
    #     m.action = Marker.DELETEALL
    #     clear.markers.append(m)
    #     self._node.gap_publisher.publish(clear)

    #     self._node.gap_publisher.publish(self._markers)
    #     self._markers = MarkerArray()
    #     self._marker_id = 0

    def vector(self, angle, distance, width=0.02, color=(1.0, 0.0, 0.0)):
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)
        self._marker_id += 1
        self._markers.markers.append(self.__create_line_marker(0.0, 0.0, x, y, float(color[0]), float(color[1]), float(color[2]), float(width), self._marker_id))

    def polygon(self, points, width=0.02, color=(0.0, 0.0, 1.0)):
        self._marker_id += 1
        marker = Marker()
        marker.header.frame_id = "ego_racecar/laser"
        marker.header.stamp = self._node.get_clock().now().to_msg()
        marker.ns = "lines"
        marker.id = self._marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = width
        marker.color.r = float(color[0])
        marker.color.g = float(color[1])
        marker.color.b = float(color[2])
        marker.color.a = 1.0

        for p in points:
            point = Point()
            point.x = float(p[0])
            point.y = float(p[1])
            point.z = float(p[2]) if len(p) > 2 else 0.0
            marker.points.append(point)

        self._markers.markers.append(marker)

    def __create_line_marker(
        self,
        x1: float, y1: float,
        x2: float, y2: float,
        r: float, g: float, b: float,
        width: float,
        marker_id: int,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id    = "ego_racecar/laser"
        marker.header.stamp       = self._node.get_clock().now().to_msg()
        marker.ns                 = "lines"
        marker.id                 = marker_id
        marker.type               = Marker.LINE_LIST
        marker.action             = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x            = float(width)

        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = 1.0

        p1 = Point(); p1.x = float(x1); p1.y = float(y1); p1.z = 0.0
        p2 = Point(); p2.x = float(x2); p2.y = float(y2); p2.z = 0.0
        marker.points = [p1, p2]

        return marker