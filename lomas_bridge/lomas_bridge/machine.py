#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lomas_interfaces.msg import MachineCommand, MachineStatus

import atexit
import serial
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
        self.declare_parameter('interval', 120) # Cultivation interval
        self.declare_parameter('speed', 100) # Motor speed
        self.declare_parameter('homing_timeout', 30.0) # Max seconds a homing move may run before it's flagged stalled
        self.declare_parameter('move_timeout', 5.0) # Max seconds a normal move may run before it's flagged stalled
        self.declare_parameter('status_poll_period', 0.25) # Seconds between position polls (assumed default)

        # Status message and publisher
        self.publisher = self.create_publisher(MachineStatus, '/machine/status', 10)
        self.status = MachineStatus()
        self.status.error_nr = 99
        self.status.interval = self.get_parameter('interval').value
        self.status.is_synced = False
        self.status.machine_moving = False
        self.status.x = 0.0
        self.status.y = 0.0
        self.status.z = 0.0

        # Internal tracking for the position-poll timer; not part of the published message
        self._move_deadline = None

        self.get_logger().info('Machine param values:')
        self.get_logger().info(f" * Sim Mode: {self.sim_mode}")
        self.get_logger().info(f" * Port:     {self.get_parameter('port').value}")
        self.get_logger().info(f" * Baud:     {self.get_parameter('baud').value}")
        self.get_logger().info(f" * Interval: {self.get_parameter('interval').value}")
        self.get_logger().info('-------------------------------------')

        # Establish serial connection
        self.stream = None
        self.connect(self.get_parameter('port').value, self.get_parameter('baud').value, self.get_parameter('speed').value)

        # Publish status
        self.publisher.publish(self.status)

        # Setup ROS subscribers
        self.create_subscription(MachineCommand, '/machine/cmd', self.cmd_callback, 10)

        # Setup position-poll timer (single-threaded executor: never overlaps cmd_callback,
        # so both are safe to share self.stream without a lock — do not move this to a
        # multi-threaded executor without adding one)
        self.create_timer(self.get_parameter('status_poll_period').value, self.poll_status)

        # Register clean-up method
        atexit.register(self.disconnect)

    # Method for opening the serial connection
    def connect(self, port, baud, speed):
        self.get_logger().info(f'Opening Serial Port')
        if self.sim_mode:
            self.get_logger().warn(f'Warning : Serial port will be simulated')
            self.status.error_nr = 0
            self.status.is_synced = True
        else:
            try:
                self.stream = serial.Serial(port, baud, timeout = 1) # 1-second timeout

                # Wake up
                self.stream.write(b'\r\n\r\n') # Hit enter a few times to wake the Printrbot
                time.sleep(2) # Wait for machine to initialize
                self.stream.reset_input_buffer() # Flush startup text in serial input

                # Unlock any alarm left over from a previous limit trip or shutdown
                self._unlock_alarm()

                self.status.error_nr = 0
                self.status.is_synced = True
                self.get_logger().info(f'Serial port connected to machine')

                # Set absolute positioning system and fixed speed
                self.send(b'G90\n')
                cmd = f'G01 F{speed}\n'
                self.send(cmd.encode('utf-8'))

            except serial.SerialException as e:
                self.status.error_nr = 98
                self.status.is_synced = False
                self.get_logger().error(f'Error when opening Serial Port')
                self.stream = None

    # Method for closing the serial connection
    def disconnect(self):
        if not self.sim_mode and self.stream is not None:
            self.stream.close()

    # Method for reinitialize after a machine reset
    def reinitialize(self):
        try:
            time.sleep(2)  # Wait for GRBL to finish booting
            self.stream.reset_input_buffer()  # Clear any remaining startup messages

            self._unlock_alarm() # Actually clear an ALARM lock, not just re-set modal state

            speed = self.get_parameter('speed').value
            self.send(b'G90\n')                          # Absolute positioning
            cmd = f'G01 F{speed}\n'
            self.send(cmd.encode('utf-8'))               # Restore feed rate
            self.get_logger().info('Machine reinitialized successfully')
            self.status.error_nr = 0
            self.status.is_synced = True
            self.status.machine_moving = False
            self.publisher.publish(self.status)

        except serial.SerialException as e:
            self.get_logger().error(f'Reinitialization failed: {e}')
            self.status.error_nr = 98
            self.status.is_synced = False
            self.publisher.publish(self.status)

    # Method for sending serial commands. Only confirms the command was accepted
    # into GRBL's motion buffer ('ok') — NOT that the resulting move has finished.
    # Actual motion completion is tracked separately by poll_status().
    def send(self, cmd):
        grbl_out = b'oMGok\n'
        if not self.sim_mode and self.stream is not None:
            try:
                self._drain_stale_input()
                self.stream.write(cmd)            # Send g-code block
                grbl_out = self.stream.readline() # Wait for response with carriage return

                if not grbl_out:
                    self.get_logger().error('Serial read timed out — no response from machine')
                    self.status.error_nr = 1
                    self.status.is_synced = False
                    self.publisher.publish(self.status)
                    return False

                if grbl_out == b'\x00' or b'Grbl' in grbl_out or b'ALARM' in grbl_out: # Detect GRBL reset signatures.
                    self.get_logger().warn('Machine reset detected — reinitializing')
                    self.reinitialize()
                    # Retry the original command once after reinitialization
                    self._drain_stale_input()
                    self.stream.write(cmd)
                    grbl_out = self.stream.readline()

            except (serial.SerialException, serial.SerialTimeoutException) as e:
                self.get_logger().error(f'Serial error: {e}')
                self.status.error_nr = 1
                self.status.is_synced = False
                self.publisher.publish(self.status)
                return False

        self.get_logger().info(f' : {grbl_out.strip()}')
        ok = grbl_out.strip() == b'ok'
        self.status.is_synced = ok
        return ok

            
    # ---------------------------
    # Helper methods
    # ---------------------------
    
    # Method for unlocking a GRBL alarm state (G90/G01 do not clear this, only $X or $H do)
    def _unlock_alarm(self):
        if self.sim_mode or self.stream is None:
            return
        self.stream.write(b'$X\n')
        time.sleep(0.2)
        while self.stream.in_waiting: # Drain whatever GRBL reports back
            line = self.stream.readline()
            self.get_logger().info(f' $X : {line.strip()}')

    # Method for discarding stale input left over from a previous command,
    # so a late response can never be misread as the reply to a new one
    def _drain_stale_input(self):
        if self.sim_mode or self.stream is None:
            return
        stale = self.stream.read(self.stream.in_waiting) if self.stream.in_waiting else b''
        if stale:
            self.get_logger().warn(f'Discarding stale serial input before send: {stale!r}')

    # Method for parsing an 'x,y,z' triple out of a GRBL '?' status report line,
    # e.g. b'<Idle|MPos:12.500,8.300,0.000|FS:0,0>'
    def _parse_mpos(self, line):
        try:
            text = line.decode('utf-8', errors='ignore')
            start = text.index('MPos:') + len('MPos:')
            end = text.index('|', start) if '|' in text[start:] else text.index('>', start)
            x_str, y_str, z_str = text[start:end].split(',')
            return float(x_str), float(y_str), float(z_str)
        except (ValueError, IndexError):
            return None

        
    # ---------------------------
    # ROS callback methods
    # --------------------------
            
    # Callback method for handle movement commands. Non-blocking: sends the move
    # and returns immediately once GRBL has accepted it. poll_status() is what
    # detects when the move actually finishes.
    def cmd_callback(self, msg):
        if msg.stop:
            self.get_logger().info(f'Moving to home position')
            ok = self.send(b'G28\n')
            timeout = self.get_parameter('homing_timeout').value
        elif msg.x >= 0.0 and msg.x <= 24.0 and msg.y >= 0.0 and msg.y <= 24.0 and msg.z >= 0.0:
            self.get_logger().info(f'Moving to poistion: x={msg.x}, y={msg.y}, z={msg.z}')
            cmd = f'G01 X{msg.x} Y{msg.y} Z{msg.z}\n'
            ok = self.send(cmd.encode('utf-8'))
            timeout = self.get_parameter('move_timeout').value
        else:
            self.get_logger().error(f'Rejected out-of-range command: x={msg.x}, y={msg.y}, z={msg.z}')
            ok = False
            self.status.error_nr = 2 # Out-of-range command
            timeout = None

        if ok:
            self.status.machine_moving = True
            self._move_deadline = time.time() + timeout
            if self.status.error_nr not in (98,):
                self.status.error_nr = 0

        self.publisher.publish(self.status)

    # Timed callback: polls GRBL's real-time status report so position is
    # available continuously, including while a move is still in progress,
    # and detects both normal move completion (Idle) and stalled moves.
    def poll_status(self):
        if self.sim_mode or self.stream is None:
            return

        self._drain_stale_input()
        self.stream.write(b'?')
        time.sleep(0.05)

        pos = None
        idle = False
        alarm = False
        while self.stream.in_waiting:
            line = self.stream.readline()
            found = self._parse_mpos(line)
            if found is not None:
                pos = found
            if b'Idle' in line:
                idle = True
            if b'ALARM' in line or b'Alarm' in line:
                alarm = True

        if pos is not None:
            self.status.x, self.status.y, self.status.z = pos

        if alarm:
            self.get_logger().error('ALARM detected during move — reinitializing')
            self.status.machine_moving = False
            self._move_deadline = None
            self.reinitialize()
            return

        if self.status.machine_moving and idle:
            self.get_logger().info('Move complete')
            self.status.machine_moving = False
            self._move_deadline = None

        elif self.status.machine_moving and self._move_deadline is not None \
                and time.time() > self._move_deadline:
            self.get_logger().error('Move stalled — no Idle state reached before timeout')
            self.status.machine_moving = False
            self.status.error_nr = 1
            self._move_deadline = None

        self.publisher.publish(self.status)

        
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
