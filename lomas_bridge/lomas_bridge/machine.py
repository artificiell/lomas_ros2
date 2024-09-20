#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, String
from lomas_interfaces.msg import MachineCommand, MachineStatus

import atexit
import serial
from serial import SerialException
import time


#  Class bridging the communcation with the LOMAS testbed
class MachineBridge(Node):
    def __init__(self):
        super().__init__('machine_bridge_node')
        self.get_logger().info('-------------------------')
        self.get_logger().info("Starting up machine node")
        self.get_logger().info('-------------------------')

        # Declare ROS parameters
        self.declare_parameter('sim_mode', False)
        self.sim_mode = self.get_parameter('sim_mode').value
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('path', '/media/gcode/')
        self.declare_parameter('interval', 120) # Cultivation interval            
        
        # Status message and publisher
        self.publisher = self.create_publisher(MachineStatus, '/machine/status', 10)
        self.status = MachineStatus()
        self.status.error_nr = 99
        self.status.interval = self.get_parameter('interval').value

        self.get_logger().info('Machine param values:')
        self.get_logger().info(f" * Sim Mode: {self.sim_mode}")
        self.get_logger().info(f" * Port:     {self.get_parameter('port').value}")
        self.get_logger().info(f" * Path:     {self.get_parameter('path').value}")
        self.get_logger().info(f" * Interval: {self.get_parameter('interval').value}")
        self.get_logger().info('-------------------------------------')
        
        # Establish serial connection
        self.stream = None
        self.connect(self.get_parameter('port').value)

        # Publish status
        self.publisher.publish(self.status)
        
        # Setup ROS subscribers
        self.create_subscription(MachineCommand, '/machien/abort', self.abort_callback, 10)
        self.create_subscription(MachineCommand, '/machien/cmd', self.cmd_callback, 10)
        self.create_subscription(MachineCommand, '/machien/stop', self.stop_callback, 10)
        
        # Register clean-up method
        atexit.register(self.disconnect)

    # Method for opening the serial connection
    def connect(self, port):
        self.get_logger().info(f'Opening Serial Port')
        if self.sime_mode:
            self.get_logger().warn(f'Warning : Serial port will be simulated')
            self.status.error_nr = 0
        else:
            try:
                self.stream = serial.Serial(port, 115200)

                # Wake up
                self.stream.write("\r\n\r\n") # Hit enter a few times to wake the Printrbot
                time.sleep(2) # Wait for machine to initialize
                self.stream.flushInput() # Flush startup text in serial input
                self.status.error_nr = 0
                self.get_logger().info(f'Serial port connected to machine')
            except serial.SerialException as e:
                self.status.error_nr = 98
                self.get_logger().error(f'Error when opening Serial Port')
                self.stream = None
                
    # Method for closing the serial connection
    def disconnect(self):
        if not self.sim_mode and self.stream is not None:
            self.stream.close()
    
    # Method for sending serial commands
    def send(self, cmd):
        grbl_out = 'oMGok\n'
        if not self.sim_mode and self.stream is not None:
            self.stream.write(cmd)            # Send g-code block
            grbl_out = self.stream.readline() # Wait for response with carriage return
        self.get_logger().info(f' : {grbl_out.strip()}')
        if grbl_out == 'oMGok\n':
            return True
        return False
    
    # Callback method for handle abort commands
    def abort_callback(self, msg):
        if msg.abort:
            self.get_logger().info(f'Abort ')
        
    # Callback method for handle movement commands
    def cmd_callback(self, msg):
        if msg.command == 99:
            self.get_logger().info(f'Starting to home robot')
            self.status.is_synced = False
            self.send_gcode_cmd('G28 X Y Z' + '\n')
            self.status.is_synced = True
        elif msg.command == 90:
            self.get_logger().info(f'Man. pos X')
            self.send_gcode_cmd('G91\n'+'G0 X10 F1000\n')
        elif msg.command == 91:
            self.get_logger().info(f'Man. neg X')
            self.send_gcode_cmd('G91\n'+'G0 X-10 F1000\n')
        elif msg.command == 92:
            self.get_logger().info(f'Man. pos Y')
            self.send_gcode_cmd('G91\n'+'G0 Y10 F1000\n')
        elif msg.command == 93:
            self.get_logger().info(f'Man. neg Y')
            self.send_gcode_cmd('G91\n'+'G0 Y-10 F1000\n')
        elif msg.command == 94:
            self.get_logger().info(f'Man. pos X pos Y')
            self.send_gcode_cmd('G91\n'+'G0 X10 Y10 F1000\n')
        elif msg.command == 95:
            self.get_logger().info(f'Man. neg X pos Y')
            self.send_gcode_cmd('G91\n'+'G0 X-10 Y10 F1000\n')
        elif msg.command == 96:
            self.get_logger().info(f'Man. pos X neg Y')
            self.send_gcode_cmd('G91\n'+'G0 X10 Y-10 F1000\n')
        elif msg.command == 97:
            self.get_logger().info(f'Man. neg X neg Y')
            self.send_gcode_cmd('G91\n'+'G0 X-10 Y-10 F1000\n')

        # Publish status
        self.publisher.publish(self.status)

    # Callback method for handle stop commands
    def abort_callback(self, msg):
        if msg.stop:
            self.get_logger().info(f'Stop ')

    # Send g-code command
    def self.send_gcode_cmd(self, cmd):

        # Publish status
        self.status.sequens_started = True
        self.status.machine_moving = True
        self.status.sequense_nr = 99
        self.publisher.publish(self.status)

        # Send serial command
        if self.send(cmd):
            self.status.error_nr = 0
            self.status.sequense_started = False
            self.status.machine_moving = False
            self.status.sequense_nr = 0

    
# Main function
def main(args = None):
    rclpy.init(args = args)
    node = MachineBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node (explicitly)
    diff_drive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
