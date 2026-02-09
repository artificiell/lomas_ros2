#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from lomas_interfaces.msg import MachineCommand


#  Class for LOMAS testbed demo
class LomasDemo(Node):

    # Declare constants
    X = [24, 24,  0,  0, 24, 24,  0]
    Y = [ 0,  8,  8, 16, 16, 24, 24]
    
    def __init__(self):
        super().__init__('lomas_demo_node')

        # Parameters (machine limits)
        self.declare_parameter('machine_max_x', 24)
        self.declare_parameter('machine_max_y', 24)
        self.max_x = max(1, int(self.get_parameter('machine_max_x').value)) # avoid division by zero
        self.max_y = max(1, int(self.get_parameter('machine_max_y').value)) # avoid division by zero
        
        # Setup ROS publisher
        self.machine_cmd_publisher = self.create_publisher(MachineCommand, 'lomas/machine/cmd', 10)
        self.web_state_publisher = self.create_publisher(String, 'lomas/control/state', 10)
        
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
        self.create_subscription(
            String,  # JSON over std_msgs/String
            '/lomas/control/cmd',
            self.web_cmd_callback,
            10
        )

        # Local variables
        self.idx = None
        self.x_current, self.y_current = 0, 0
        self.x_target, self.y_target = None, None

        # Create timer callabck
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        
    @staticmethod
    def clamp(v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))


    # General publisher of both machine command and machie state (for web control)
    def publish_cmd_and_state(self, x: int, y: int, stop: bool = False) -> None:

        # Publish machine commands
        cmd = MachineCommand()
        cmd.x, cmd.y = x, y
        cmd.stop = stop
        self.machine_cmd_publisher.publish(cmd)
        
        # Map machine units -> normalized [0,1]
        x_norm = self.clamp(x / float(self.max_x), 0.0, 1.0)
        y_norm = self.clamp(y / float(self.max_y), 0.0, 1.0)
        
        # Publish normalized state to the web control
        state = String()
        state.data = json.dumps({"y": round(x_norm, 4), "x": round(y_norm, 4)})
        self.web_state_publisher.publish(state)
    
        
    # Timed callabck function
    def timer_callback(self) -> None:
        if self.x_target is not None and self.y_target is not None:
            if self.y_current < self.y_target:
                self.y_current += 1
            elif self.x_target == 25 and self.x_current < self.x_target:
                self.x_current += 1 
            elif self.x_target == 0 and self.x_current > self.x_target:
                self.x_current -= 1


            # Publish machine command + web state
            self.publish_cmd_and_state(self.x_current, self.y_current, stop = False)
            
            # Handle waypoint completion
            if self.x_current == self.x_target and self.y_current == self.y_target:
                self.idx += 1
                if self.idx >= len(self.X):
                    self.idx = None
                    self.x_current, self.y_current = 0, 0
                    self.x_target, self.y_target = None, None
                    self.publish_cmd_and_state(self.x_current, self.y_current, stop = True)
                else:
                    self.x_target, self.y_target = self.X[self.idx], self.Y[self.idx]

                    
    # Callback for start button pressed
    def start_callback(self, msg) -> None:
        if msg.data and self.x_current == 0 and self.y_current == 0: # Data equals to True (= pressed)
            self.idx = 0
            self.x_target, self.y_target = self.X[self.idx], self.Y[self.idx]


    # Callback for stop button pressed
    def stop_callback(self, msg) -> None:
        if msg.data: # Data equals to True (= pressed)
            self.idx = None
            self.x_current, self.y_current = 0, 0
            self.x_target, self.y_target = None, None
            self.publish_cmd_and_state(self.x_current, self.y_current, stop = True)

            
    # Callback for handle commands from web control
    def web_cmd_callback(self, msg) -> None:
        """
        Receive normalized web command JSON: {"x":0..1,"y":0..1}
        Convert to machine units using machine_max_* params and forward as MachineCommand.
        Also set target so the demo "moves" there via timer_callback.
        """
        try:
            data = json.loads(msg.data)
            x_norm = self.clamp(float(data.get("y")), 0.0, 1.0) 
            y_norm = self.clamp(float(data.get("x")), 0.0, 1.0)

            # Convert normalized -> machine units (integers)
            x_cmd = int(round(x_norm * self.max_x))
            y_cmd = int(round(y_norm * self.max_y))

            # Clamp to machine bounds
            x_cmd = max(0, min(self.max_x, x_cmd))
            y_cmd = max(0, min(self.max_y, y_cmd))

            # Cancel waypoint sequence when web is used
            self.idx = None

            # Set targets for timer motion, and forward immediate MachineCommand too
            self.x_current, self.y_current = x_cmd, y_cmd
            self.x_target, self.y_target = None, None
            self.publish_cmd_and_state(self.x_current, self.y_current, stop = False)
            
        except Exception as e:
            self.get_logger().warn(f"Invalid JSON from web control: {msg.data!r}")
                    
            
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
