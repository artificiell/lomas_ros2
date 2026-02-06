#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from lomas_interfaces.msg import MachineCommand


#  Class for LOMAS testbed demo
class LomasDemo(Node):

    # Declare constants
    X = [25, 25,  0,  0, 25, 25,  0]
    Y = [ 0,  8,  8, 16, 16, 24, 24]
    
    def __init__(self):
        super().__init__('lomas_demo_node')

        # Setup ROS publisher
        self.command_publisher = self.create_publisher(MachineCommand, 'lomas/machine/cmd', 10)

        # Setup ROS subscribers
        self.create_subscription(
            Bool,
            'machine/start',
            self.start_callback,
            10
        )
        self.create_subscription(
            Bool,
            'machine/stop',
            self.stop_callback,
            10
        )
        self.idx = None
        self.x_current, self.y_current = 0, 0
        self.x_target, self.y_target = None, None

        # Create timer callabck
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        
    # Timed callabck function
    def timer_callback(self):
        if self.x_target is not None and self.y_target is not None:
            if self.y_current < self.y_target:
                self.y_current += 1
            elif self.x_target == 25 and self.x_current < self.x_target:
                self.x_current += 1 
            elif self.x_target == 0 and self.x_current > self.x_target:
                self.x_current -= 1

            
            cmd = MachineCommand()
            cmd.x = self.x_current
            cmd.y = self.y_current
            cmd.stop = False
            self.command_publisher.publish(cmd)
            
                
            if self.x_current == self.x_target and self.y_current == self.y_target:
                self.idx += 1
                if self.idx >= len(self.X):
                    self.idx = None
                    self.x_current, self.y_current = 0, 0
                    self.x_target, self.y_target = None, None
                    cmd.stop = True
                    self.command_publisher.publish(cmd)
                else:
                    self.x_target, self.y_target = self.X[self.idx], self.Y[self.idx]

                    
    # Callback for start button pressed
    def start_callback(self, msg):
        if msg.data and self.x_current == 0 and self.y_current == 0: # Data equals to True (= pressed)
            self.idx = 0
            self.x_targe


    # Callback for stop button pressed
    def stop_callback(self, msg):
        if msg.data: # Data equals to True (= pressed)
            cmd = MachineCommand()
            cmd.stop = True
            self.command_publisher.publish(cmd)
            self.idx = None
            self.x_current, self.y_current = 0, 0
            self.x_target, self.y_target = None, None

            
# Main function
def main(args = None):
    rclpy.init(args = args)
    node = LomasDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node (explicitly)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
