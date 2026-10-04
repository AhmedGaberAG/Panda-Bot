#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class ScanAnalyzer(Node):
    def __init__(self):
        super().__init__("scan_analyzer")
        self.subscription = self.create_subscription( LaserScan, "scan", self.scan_callback, 10)

    def scan_callback(self, msg):
        print("\n--- LaserScan ---")
        for i, distance in enumerate(msg.ranges):
            if math.isfinite(distance) and 0.05 < distance < 0.40:
                angle = msg.angle_min + i * msg.angle_increment
                print(
                    f"angle: {math.degrees(angle):7.2f}°"
                    f"   distance: {distance:.3f} m"
                )
        print("-----------------\n")
        self.destroy_node()
        rclpy.shutdown()

def main():
    rclpy.init()
    node = ScanAnalyzer()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
