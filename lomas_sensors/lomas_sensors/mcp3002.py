#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32 

from gpiozero import MCP3002


# Class for handle soil moisture sensor
class SoilMoisture(Node):

    def __init__(self):
        super().__init__('soil_moisture_node')

        # Declare port parameter
        self.declare_parameter('calibration', False)
        self.declare_parameter('channel', 0)
        self.declare_parameter('period', 1.0)  #  ...time period (seconds)

        # Init soil moisture sensor through MCP3002 A/D converter
        channel = self.get_parameter('channel').value
        self.sensor = MCP3002(channel = channel)
        self.get_logger().info(f"Init soil moisture sensor on MCP3002 channel: {channel}")
        if self.get_parameter('calibration').value:
            self.get_logger().info(f"Running calibration, ensure that the sensor is exposed to absoulute soil moisture...")
            self.min_values = []
        else:
            self.min_values = None
            
        # Setup ROS publisher
        self.publisher = self.create_publisher(Float32, '/soil/moisture', 10)
        timer_period = self.get_parameter('period').value
        self.get_logger().info(f"Publishing soil moisture values every {timer_period} second...")
        self.timer = self.create_timer(timer_period, self.callback)

    # Read and publish sensor value
    def callback(self):
        try:
            value = self.sensor.value
            if self.min_values is not None:
                scalar = self.calibrate(value)
                if scalar < 1.0:
                    value = (value - scalar) / (1.0 - scalar) # ...min-max normalization (max = 1.0)
                value = 0.0 if value < 0.0 else value 
            msg = Float32()
            msg.data = 1.0 - value
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Soil moisture sensor: {e}", throttle_duration_sec = 1)

    # Calibration of sensor value
    def calibrate(self, value):
        if len(self.min_values) < 100: #  ...maintain a small list of min sensor readings (to prevent jitter)
            self.min_values.append(value)
        elif value < max(self.min_values):
            self.min_values.remove(max(self.min_values))
            self.min_values.append(value)
        return sum(self.min_values) / len(self.min_values)

            
# Main function
def main(args = None):
    rclpy.init(args = args)
    node = SoilMoisture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    # Destroy the node (explicitly)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

