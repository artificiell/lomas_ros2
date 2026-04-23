#!/usr/bin/env python3
import asyncio
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tapo import ApiClient

#  Class for controlling the light of the LOMAS testbed
class LightPlug(Node):
    def __init__(self):
        super().__init__('light_plug_node')

        # ROS paramters
        self.declare_parameter('username', 'user')
        self.declare_parameter('password', 'password')
        self.declare_parameter('ip_address', '192.168.1.23')
        self.username = self.get_parameter('username').get_parameter_value().string_value
        self.password = self.get_parameter('password').get_parameter_value().string_value
        self.ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        
        # Setup ROS subscriber
        self.create_subscription(Bool, '/lomas/light', self.message_callback, 10)

        # Setup ROS service
        self.create_service(SetBool, '/lomas/light', self.service_callback)

        # Taopi 
        self.device, self.toggle = None, False
        self.get_logger().info('Light plug node initialized')    

    # Async connection fucntion
    async def connect(self):
        try:
            client = ApiClient(self.username, self.password)
            self.device = await client.generic_device(self.ip_address)
            info = await self.device.get_device_info()
            self.toggle = info.to_dict()['device_on']
            self.get_logger().info('Connected to light plug device')
        except Exception as e:
            self.device = None
            self.get_logger().warn(f'Failed to connect to light plug: {str(e)}')

    # Check if device connceted (or not)
    def connected(self):
        return self.device is not None
        
    # Callback function for handel message commands
    def message_callback(self, msg):
        asyncio.ensure_future(self.process(msg.data))

    # Async callback function for handel sercive request
    def service_callback(self, request, response):
        if not self.connected():
            response.success = False
            response.message = 'Light plug device not connected'
        else:
            asyncio.ensure_future(self.process(request.data))
            response.success = True
            response.message = f'Light turned {"on" if request.data else "off"}'
        return response
        
    # Async function for processing messages
    async def process(self, cmd):
        if self.connected():
            if cmd and not self.toggle:
                self.get_logger().info(f'Turning on light plug')
                self.toggle = cmd
                await self.device.on()
            elif not cmd and self.toggle:
                self.get_logger().info(f'Turning off light plug')
                self.toggle = cmd
                await self.device.off()

    # Async function to monitor if the device is nocceted (or not)
    async def monitor(self, interval = 5.0):
        if self.connected():
            try:
                await self.device.get_device_info()
            except Exception:
                self.get_logger().warn('Device not responding, attempting reconnect...')
                self.device = None
        else:
            await self.connect()
        await asyncio.sleep(interval)
                
# Custom spin loop
async def spin(executor, node):
    await node.connect()
    asyncio.ensure_future(node.monitor())    
    while rclpy.ok():
        executor.spin_once()
        await asyncio.sleep(1e-3)

# Main function
def main(args=None):
    rclpy.init(args=args)

    node = LightPlug()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    loop = asyncio.get_event_loop()
    loop.run_until_complete(spin(executor, node))

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
