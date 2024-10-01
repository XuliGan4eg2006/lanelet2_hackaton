#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import xml.etree.ElementTree as ET
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import os
from ament_index_python.packages import get_package_share_directory


class OSMCartographyNode(Node):
    def __init__(self):
        super().__init__('osm_cartography_node')

        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(MarkerArray, 'osm_markers', 10)

        # Declare parameters
        self.declare_parameter('osm_file', 'map.osm')

        # Load OSM data
        self.load_osm_file()

        # Timer to periodically publish markers
        self.timer = self.create_timer(1.0, self.publish_osm_data)

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
        for node in nodes:
            print(node.find('.//tag[@k="local_x"]').attrib['v'])

        # if nodes:
        #     lats = [float(node.find('.//tag[@k="local_x"]').attrib['v']) for node in nodes]
        #     lons = [float(node.find('.//tag[@k="local_y"]').attrib['v']) for node in nodes]
        #     self.min_lat = min(lats)
        #     self.max_lat = max(lats)
        #     self.min_lon = min(lons)
        #     self.max_lon = max(lons)
        # else:
        #     # Default values if no nodes found
        #     self.min_lat = self.max_lat = self.min_lon = self.max_lon = 0.0

    def publish_osm_data(self):
        if self.root is None:
            return

        marker_array = MarkerArray()
        marker_id = 0

        # Process ways (roads, buildings, etc.)
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

            points = []
            node_refs = [nd.get('ref') for nd in way.findall('nd')]

            for ref in node_refs:
                node = self.root.find(f".//node[@id='{ref}']")
                if node is not None:
                    lat = float(node.get('lat'))
                    lon = float(node.get('lon'))
                    # Convert to local coordinates (simplified)
                    x = (lon - self.min_lon) * 111000  # Approximate meters
                    y = (lat - self.min_lat) * 111000
                    points.append(Point(x=x, y=y, z=0.0))

            marker.points = points
            if points:  # Only add marker if it has points
                marker_array.markers.append(marker)
                marker_id += 1

        self.marker_pub.publish(marker_array)

    def determine_way_type(self, way):
        for tag in way.findall('tag'):
            if tag.get('k') == 'highway':
                return 'highway'
            elif tag.get('k') == 'building':
                return 'building'
        return 'unknown'

    def get_color_for_way_type(self, way_type):
        color = ColorRGBA()
        if way_type == 'highway':
            color.r, color.g, color.b = 1.0, 1.0, 0.0  # Yellow for roads
        elif way_type == 'building':
            color.r, color.g, color.b = 0.8, 0.0, 0.0  # Red for buildings
        else:
            color.r, color.g, color.b = 0.7, 0.7, 0.7  # Gray for unknown
        color.a = 1.0
        return color


def main(args=None):
    rclpy.init(args=args)
    node = OSMCartographyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()