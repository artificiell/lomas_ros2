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
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)  # Baud rate: 9600 or 115200
        #self.declare_parameter('path', '/media/gcode/')
        self.declare_parameter('interval', 120) # Cultivation interval            
        self.declare_parameter('speed', 100) # Motor speed
        
        # Status message and publisher
        self.publisher = self.create_publisher(MachineStatus, '/machine/status', 10)
        self.status = MachineStatus()
        self.status.error_nr = 99
        self.status.interval = self.get_parameter('interval').value

        self.get_logger().info('Machine param values:')
        self.get_logger().info(f" * Sim Mode: {self.sim_mode}")
        self.get_logger().info(f" * Port:     {self.get_parameter('port').value}")
        self.get_logger().info(f" * Baud:     {self.get_parameter('baud').value}")
        #self.get_logger().info(f" * Path:     {self.get_parameter('path').value}")
        self.get_logger().info(f" * Interval: {self.get_parameter('interval').value}")
        self.get_logger().info('-------------------------------------')
        
        # Establish serial connection
        self.stream = None
        self.connect(self.get_parameter('port').value, self.get_parameter('baud').value, self.get_parameter('speed').value)

        # Publish status
        self.publisher.publish(self.status)
        
        # Setup ROS subscribers
        self.create_subscription(MachineCommand, '/machine/cmd', self.cmd_callback, 10)
        
        # Register clean-up method
        atexit.register(self.disconnect)

    # Method for opening the serial connection
    def connect(self, port, baud, speed):
        self.get_logger().info(f'Opening Serial Port')
        if self.sim_mode:
            self.get_logger().warn(f'Warning : Serial port will be simulated')
            self.status.error_nr = 0
        else:
            try:
                self.stream = serial.Serial(port, baud, timeout = 1) # 1-second timeout

                # Wake up
                self.stream.write(b'\r\n\r\n') # Hit enter a few times to wake the Printrbot
                time.sleep(2) # Wait for machine to initialize
                self.stream.flushInput() # Flush startup text in serial input
                self.status.error_nr = 0
                self.get_logger().info(f'Serial port connected to machine')

                # Set absolute positioning system and fixed speed
                self.send(b'G90\n')
                cmd = f'G01 F{speed}\n'
                self.send(cmd.encode('utf-8'))
                
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
        grbl_out = b'oMGok\n'
        if not self.sim_mode and self.stream is not None:
            try:
                self.stream.write(cmd)            # Send g-code block
                grbl_out = self.stream.readline() # Wait for response with carriage return

                if not grbl_out:
                    self.get_logger().error('Serial read timed out — no response from machine')
                    self.status.error_nr = 1
                    self.publisher.publish(self.status)
                    return False

                #if self._detect_reset(grbl_out):
                if grbl_out == b'\x00' or b'Grbl' in grbl_out or b'ALARM' in grbl_out: # Detect GRBL reset signatures.
                    self.get_logger().warn('Machine reset detected — reinitializing')
                    self.reinitialize()
                    # Retry the original command once after reinitialization
                    self.stream.write(cmd)
                    grbl_out = self.stream.readline()

            except (serial.SerialException, serial.SerialTimeoutException) as e:
                self.get_logger().error(f'Serial error: {e}')
                self.status.error_nr = 1
                self.publisher.publish(self.status)
                return False

        self.get_logger().info(f' : {grbl_out.strip()}')
        return grbl_out.strip() == b'ok'

    # Method for reinitialize after a machine reset
    def reinitialize(self):
        try:
            time.sleep(2)  # Wait for GRBL to finish booting
            self.stream.flushInput()  # Clear any remaining startup messages

            speed = self.get_parameter('speed').value
            self.send(b'G90\n')                          # Absolute positioning
            cmd = f'G01 F{speed}\n'
            self.send(cmd.encode('utf-8'))               # Restore feed rate
            self.get_logger().info('Machine reinitialized successfully')
            self.status.error_nr = 0
            self.publisher.publish(self.status)

        except serial.SerialException as e:
            self.get_logger().error(f'Reinitialization failed: {e}')
            self.status.error_nr = 98
            self.publisher.publish(self.status)

    '''
    def send(self, cmd):
        grbl_out = 'oMGok\n'
        if not self.sim_mode and self.stream is not None:
            self.stream.write(cmd)            # Send g-code block
            grbl_out = self.stream.readline() # Wait for response with carriage return
            if not grbl_out:  # Empty bytes = timeout occurred
                self.get_logger().error('Serial read timed out — no response from machine')
                self.status.error_nr = 1
                self.publisher.publish(self.status)
                return False
        self.get_logger().info(f' : {grbl_out.strip()}')
        if grbl_out == 'oMGok\n':
            return True
        return False
    '''


    
    # Callback method for handle movement commands
    def cmd_callback(self, msg):
        if msg.stop:
            self.get_logger().info(f'Moving to home position')
            self.send(b'G28\n')
        else:
            if msg.x >= 0 and msg.x <= 25 and msg.y >= 0 and msg.y <= 24:
                self.get_logger().info(f'Moving to poistion: x={msg.x}, y={msg.y}, z={msg.z}')            
                cmd = f'G01 X{msg.x} Y{msg.y} Z{msg.z}\n'
                self.send(cmd.encode('utf-8'))
    
# Main function
def main(args = None):
    rclpy.init(args = args)
    node = MachineBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node (explicitly)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
