#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from lomas_interfaces.msg import MachineCommand

import json

#  Class converting commands for communcation with the LOMAS testbed
class CommandConverter(Node):
    def __init__(self):
        super().__init__('command_converter_node')
        self.get_logger().info('Command converter node started...')

        # Setup ROS publisher and subscriber
        self.publisher = self.create_publisher(MachineCommand, '/machine/cmd', 10)
        self.subscription = self.create_subscription(String, '/converter/cmd', self.callback, 10)

    # Callback method for converting commands
    def callback(self, msg):
        try:
            # Convert JSON string to dictionary
            dictionary = json.loads(msg.data)
            
            # Create MachineCommand message
            cmd = MachineCommand()
            cmd.x = dictionary.get("x", 0)
            cmd.y = dictionary.get("y", 0)
            cmd.z = dictionary.get("z", 0)
            cmd.stop = dictionary.get("stop", False)
            
            # Publish MachineCommand message
            self.publisher.publish(cmd)

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to decode JSON: {e}')

# Main function
def main(args = None):
    rclpy.init(args = args)
    node = CommandConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Destroy the node (explicitly)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
