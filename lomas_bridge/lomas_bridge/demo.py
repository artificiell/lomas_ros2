#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from lomas_interfaces.msg import MachineCommand, MachineStatus


#  Class for LOMAS testbed demo
class LomasDemo(Node):

    # Declare constants
    #X = [24, 24,  0,  0, 24, 24,  0]
    #Y = [ 0,  8,  8, 16, 16, 24, 24]
    X = [1.0, 1.0,  0.0, 0.0, 1.0, 1.0,  0.0, 0.0, 1.0]
    Y = [0.0, .25,  .25, 0.5, 0.5, 0.75, .75, 1.0, 1.0]

    STEP = 1.0 # Machine units advanced per timer tick, once the previous step is confirmed

    def __init__(self):
        super().__init__('lomas_demo_node')

        # Parameters (machine limits)
        self.declare_parameter('machine_max_x', 24.0)
        self.declare_parameter('machine_max_y', 24.0)
        self.declare_parameter('position_tolerance', 0.1) # How close to a target counts as "reached"
        self.max_x = max(1.0, float(self.get_parameter('machine_max_x').value)) # avoid division by zero
        self.max_y = max(1.0, float(self.get_parameter('machine_max_y').value)) # avoid division by zero
        self.tolerance = float(self.get_parameter('position_tolerance').value)

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
            'lomas/control/cmd',
            self.web_cmd_callback,
            10
        )
        self.create_subscription(
            MachineStatus,
            'lomas/machine/status',
            self.machine_status_callback,
            10
        )

        # Local variables
        self.idx = None
        self.x_current, self.y_current = 0.0, 0.0
        self.x_target, self.y_target = None, None

        # Latest known machine feedback, and whether we're waiting on it to
        # confirm the last commanded step before advancing any further
        self.machine_status = None
        self.awaiting_confirmation = False
        self.last_commanded_x, self.last_commanded_y = None, None

        # Create timer callabck
        timer_period = 0.6  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        
    @staticmethod
    def clamp(v: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, v))


    # Callback for machine status feedback (real reported position + motion state)
    def machine_status_callback(self, msg) -> None:
        self.machine_status = msg


    # Method for checking whether the machine has actually confirmed arrival at
    # the last commanded x/y, using real feedback rather than our own bookkeeping
    def _step_confirmed(self) -> bool:
        if self.machine_status is None or self.last_commanded_x is None:
            return False
        if self.machine_status.machine_moving:
            return False
        dx = abs(self.machine_status.x - self.last_commanded_x)
        dy = abs(self.machine_status.y - self.last_commanded_y)
        return dx <= self.tolerance and dy <= self.tolerance


    # General publisher of both machine command and machie state (for web control)
    def publish_cmd_and_state(self, x: float, y: float, stop: bool = False) -> None:

        # Publish machine commands (x/y/z are float32 on MachineCommand; z left at 0.0 —
        # this testbed only moves in the x/y plane)
        cmd = MachineCommand()
        cmd.x, cmd.y = float(x), float(y)
        cmd.z = 0.0
        cmd.stop = stop
        self.machine_cmd_publisher.publish(cmd)

        # Track what we last asked the machine to do, so it can be confirmed
        # against real feedback in machine_status_callback / _step_confirmed
        self.last_commanded_x, self.last_commanded_y = float(x), float(y)
        self.awaiting_confirmation = True

        # Map machine units -> normalized [0,1]
        x_norm = self.clamp(x / self.max_x, 0.0, 1.0)
        y_norm = self.clamp(y / self.max_y, 0.0, 1.0)
        
        # Publish normalized state to the web control
        state = String()
        state.data = json.dumps({"y": round(x_norm, 4), "x": round(y_norm, 4)})
        self.web_state_publisher.publish(state)
    
        
    # Timed callabck function
    def timer_callback(self) -> None:
        if self.x_target is None or self.y_target is None:
            return

        # Don't advance to the next step until the machine has actually
        # confirmed (via /machine/status) that it reached the last one —
        # previously this just assumed a step succeeded after one tick
        if self.awaiting_confirmation:
            if self._step_confirmed():
                self.awaiting_confirmation = False
            else:
                return

        # Move one step toward target (generic), clamped so it can't overshoot
        if self.y_current < self.y_target:
            self.y_current = min(self.y_current + self.STEP, self.y_target)
        elif self.y_current > self.y_target:
            self.y_current = max(self.y_current - self.STEP, self.y_target)
        elif self.x_current < self.x_target:
            self.x_current = min(self.x_current + self.STEP, self.x_target)
        elif self.x_current > self.x_target:
            self.x_current = max(self.x_current - self.STEP, self.x_target)

        # Publish machine command + web state
        self.publish_cmd_and_state(self.x_current, self.y_current, stop = False)

        # Handle waypoint completion
        if self.x_current == self.x_target and self.y_current == self.y_target:
            self.idx += 1
            if self.idx >= len(self.X):
                self.idx = None
                self.x_current, self.y_current = 0.0, 0.0
                self.x_target, self.y_target = None, None
                self.publish_cmd_and_state(self.x_current, self.y_current, stop = True)
            else:
                self.x_target, self.y_target = self.X[self.idx] * self.max_x, self.Y[self.idx] * self.max_y

                    
    # Callback for start button pressed
    def start_callback(self, msg) -> None:
        if msg.data and self.x_current == 0 and self.y_current == 0: # Data equals to True (= pressed)
            self.idx = 0
            self.x_target, self.y_target = self.X[self.idx] * self.max_x, self.Y[self.idx] * self.max_y


    # Callback for stop button pressed
    def stop_callback(self, msg) -> None:
        if msg.data: # Data equals to True (= pressed)
            self.idx = None
            self.x_current, self.y_current = 0.0, 0.0
            self.x_target, self.y_target = None, None
            self.publish_cmd_and_state(self.x_current, self.y_current, stop = True)

            
    # Callback for handle commands from web control
    def web_cmd_callback(self, msg) -> None:
        """
        Receive normalized web command JSON: {"x":0..1,"y":0..1}
        Convert to machine units using machine_max_* params and forward as MachineCommand.
        Also set target so the demo "moves" there via timer_callback.

        MachineCommand.x/y are float32, so the full sub-unit precision of the
        normalized web input is preserved here rather than rounded to the
        nearest whole machine unit.
        """
        try:
            data = json.loads(msg.data)
            x_norm = self.clamp(float(data.get("y")), 0.0, 1.0) 
            y_norm = self.clamp(float(data.get("x")), 0.0, 1.0)

            # Convert normalized -> machine units (float, sub-unit precision preserved)
            x_cmd = self.clamp(x_norm * self.max_x, 0.0, self.max_x)
            y_cmd = self.clamp(y_norm * self.max_y, 0.0, self.max_y)

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
