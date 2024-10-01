#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import xml.etree.ElementTree as ET
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped
from std_msgs.msg import ColorRGBA
import os
from ament_index_python.packages import get_package_share_directory
import tf2_ros
import math


class OSMCartographyNode(Node):
    def __init__(self):
        super().__init__('osm_cartography_node')

        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, 'osm_markers', 10)

        # Declare parameters
        self.declare_parameter('osm_file', 'map.osm')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Load OSM data
        self.load_osm_file()
        self.publish_osm_data()

        self.set_robot_position()

        # Timer to periodically publish markers
        self.timer = self.create_timer(1.0, self.publish_robot_transform)

    def load_osm_file(self):
        try:
            # Get the OSM file path from parameter
            osm_file = self.get_parameter('osm_file').get_parameter_value().string_value

            # Check if it's an absolute path, if not, look in the package's share directory
            if not os.path.isabs(osm_file):
                package_share_dir = get_package_share_directory('osm_cartography')
                osm_file = os.path.join(package_share_dir, 'maps', osm_file)

            self.get_logger().info(f'Loading OSM file: {osm_file}')

            # Parse the XML file
            tree = ET.parse(osm_file)
            self.root = tree.getroot()

            # Find bounds in the OSM file
            bounds = self.root.find('.//bounds')
            self.calculate_bounds()

        except Exception as e:
            self.get_logger().error(f'Failed to load OSM file: {str(e)}')
            self.root = None

    def calculate_bounds(self):
        nodes = self.root.findall('.//node')

        # Calculate bounds
        if nodes:
            lats = [float(node.find('tag[@k="local_x"]').attrib['v']) for node in nodes]
            lons = [float(node.find('tag[@k="local_y"]').attrib['v']) for node in nodes]
            self.min_lat = min(lats)
            self.max_lat = max(lats)
            self.min_lon = min(lons)
            self.max_lon = max(lons)
        else:
            # Default values if no nodes found
            self.min_lat = self.max_lat = self.min_lon = self.max_lon = 0.0

    def publish_osm_data(self):
        if self.root is None:
            return

        marker_array = MarkerArray()
        marker_id = 0

        # Process ways
        for way in self.root.findall('.//way'):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 1.0  # Line width

            # Determine way type and set color
            way_type = self.determine_way_type(way)
            marker.color = self.get_color_for_way_type(way_type)
            marker.color.a = 1.0

            points = []
            node_refs = [nd.get('ref') for nd in way.findall('nd')]

            for ref in node_refs:
                node = self.root.find(f".//node[@id='{ref}']")
                if node is not None:
                    x_tag = node.find('tag[@k="local_x"]')
                    y_tag = node.find('tag[@k="local_y"]')
                    if x_tag is not None and y_tag is not None:
                        x = float(x_tag.attrib['v'])
                        y = float(y_tag.attrib['v'])
                        points.append(Point(x=x, y=y, z=0.0))
                        self.get_logger().info(f'Added point: {x}, {y}')

            marker.points = points

            if points:  # Only add marker if it has points
                marker_array.markers.append(marker)
                marker_id += 1

        self.marker_pub.publish(marker_array)
        self.get_logger().info(f'Published {len(marker_array.markers)} markers')
    def determine_way_type(self, way):
        return "thin_way"
        # for tag in way.findall('tag'):
        #     if tag.get('k') == 'highway':
        #         return 'highway'
        #     elif tag.get('k') == 'building':
        #         return 'building'
        # return 'unknown'

    def get_color_for_way_type(self, way_type):
        color = ColorRGBA()
        if way_type == 'thin_way':
            color.r, color.g, color.b = 1.0, 1.0, 0.0  # Yellow for roads
        else:
            color.r, color.g, color.b = 0.7, 0.7, 0.7  # Gray for unknown
        color.a = 1.0
        return color

    def set_robot_position(self):
        self.robot_x = 40.9428
        self.robot_y = 472.869 #for test

    def publish_robot_transform(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.translation.z = 0.0

        # Convert Euler angle to quaternion (simplified, assuming only yaw)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        # t.transform.rotation.z = math.sin(self.robot_yaw / 2)
        # t.transform.rotation.w = math.cos(self.robot_yaw / 2)

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info('Published robot transform')
def main(args=None):
    rclpy.init(args=args)
    node = OSMCartographyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()