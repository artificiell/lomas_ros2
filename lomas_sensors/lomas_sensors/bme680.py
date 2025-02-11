#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32 

import board
import adafruit_bme680


# Class for handle Adafruit circuit multi sensor
class AdafruitCircuit(Node):

    def __init__(self):
        super().__init__('adafruit_circuit_node')

        # Init Adafruit circuit multi sensor
        i2c = board.I2C() # Communicating over default I2C bus
        self.sensor =  adafruit_bme680.Adafruit_BME680_I2C(i2c)
        self.get_logger().info(f"Init Adafruit circuit multi sensor over default I2C bus.")

        # Change this to match the location's pressure (hPa) at sea level
        self.sensor.sea_level_pressure = 1013.25

        # Setup ROS publisher
        self.temperature_publisher = self.create_publisher(Float32, '/air/temperature', 10)
        self.humidity_publisher = self.create_publisher(Float32, '/air/humidity', 10)
        self.pressure_publisher = self.create_publisher(Float32, '/air/pressure', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.callback)

    # Read and publish sensor value
    def callback(self):
        try:
            msg = Float32()
            msg.data = self.sensor.temperature
            self.temperature_publisher.publish(msg)
            msg.data = self.sensor.relative_humidity
            self.humidity_publisher.publish(msg)
            msg.data = self.sensor.pressure
            self.pressure_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Adafruit circuit sensor: {e}", throttle_duration_sec = 1)

                                                                                                                                    
# Main function
def main(args = None):
    rclpy.init(args = args)
    node = AdafruitCircuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    # Destroy the node (explicitly)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

