#!/usr/bin/env python3
"""
Holds the machine's last known position as ROS parameters (x, y, z).

"""
import rclpy
from rclpy.node import Node

class PositionStore(Node):
    def __init__(self):
        super().__init__('machine_position_store_node')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = PositionStore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
