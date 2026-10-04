#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

class ScanSelfFilter(Node):
    def __init__(self):
        super().__init__("scan_self_filter")

        # Panda-Bot geometry
        self.robot_radius = 0.25
        self.lidar_x = 0.175
        self.lidar_y = 0.0
        self.surface_tolerance = 0.03

        self.scan_config = None
        self.surface_distances = []

        self.scan_sub = self.create_subscription(
            LaserScan, 
            "scan", 
            self.scan_callback, 
            qos_profile_sensor_data
        )

        self.scan_pub = self.create_publisher(
            LaserScan, 
            "scan_filtered", 
            qos_profile_sensor_data
        )

        self.get_logger().info(f"LiDAR self-filter started | \
                               radius={self.robot_radius:.3f} m | \
                               lidar=({self.lidar_x:.3f}, {self.lidar_y:.3f}) m | \
                               tolerance={self.surface_tolerance:.3f} m")

    def get_surface_distance(self, angle):
        """
        Return the first positive intersection between a 
        laser ray and the robot cylinder.
        """
        dx = math.cos(angle)
        dy = math.sin(angle)

        b = 2.0 * (self.lidar_x * dx + self.lidar_y * dy)
        c = self.lidar_x ** 2 + self.lidar_y ** 2 - self.robot_radius ** 2
        discriminant = b ** 2 - 4.0 * c

        if discriminant < 0.0:
            return None

        sqrt_discriminant = math.sqrt(discriminant)
        t1 = (-b - sqrt_discriminant) / 2.0
        t2 = (-b + sqrt_discriminant) / 2.0

        intersections = [t for t in (t1, t2) if t > 0.0]
        return min(intersections) if intersections else None

    def update_surface_distances(self, scan):
        """Precompute the expected robot-surface distance for each laser beam."""
        config = (scan.angle_min, scan.angle_increment, len(scan.ranges))

        if config == self.scan_config:
            return

        self.surface_distances = [
            self.get_surface_distance(scan.angle_min + i * scan.angle_increment)
            for i in range(len(scan.ranges))
        ]
        self.scan_config = config

        self.get_logger().info(f"Computed geometry for {len(self.surface_distances)} laser beams.")

    def is_self_detection(self, distance, surface_distance):
        """Return True only when a measurement matches the robot surface."""
        return surface_distance is not None and abs(distance - surface_distance) <= self.surface_tolerance

    def scan_callback(self, scan):
        self.update_surface_distances(scan)

        filtered_scan = LaserScan()
        filtered_scan.header = scan.header
        filtered_scan.angle_min = scan.angle_min
        filtered_scan.angle_max = scan.angle_max
        filtered_scan.angle_increment = scan.angle_increment
        filtered_scan.time_increment = scan.time_increment
        filtered_scan.scan_time = scan.scan_time
        filtered_scan.range_min = scan.range_min
        filtered_scan.range_max = scan.range_max

        filtered_ranges = list(scan.ranges)

        for i, distance in enumerate(scan.ranges):
            if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
                continue
            
            if self.is_self_detection(distance, self.surface_distances[i]):
                filtered_ranges[i] = float("inf")

        filtered_scan.ranges = filtered_ranges
        filtered_scan.intensities = list(scan.intensities)
        self.scan_pub.publish(filtered_scan)

def main(args=None):
    rclpy.init(args=args)
    scan_self_filter = ScanSelfFilter()
    rclpy.spin(scan_self_filter)
    scan_self_filter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
