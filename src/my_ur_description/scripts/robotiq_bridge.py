import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time
import pymodbus

try:
    from pymodbus.client import ModbusSerialClient
except ImportError:
    from pymodbus.client.sync import ModbusSerialClient

class RobotiqHumbleBridge(Node):
    def __init__(self):
        super().__init__('robotiq_bridge')
        
        self.client = ModbusSerialClient(
            method='rtu',
            port='/tmp/ttyUR', 
            baudrate=115200, 
            stopbits=1, 
            bytesize=8, 
            parity='N',
            timeout=0.5 # UR TCI prefers shorter timeouts for recovery
        )
        
        if not self.client.connect():
            self.get_logger().error("CONNECTION FAILED!")
            return

        # Give socat a moment to stabilize the TCP handshake
        time.sleep(1.0)

        self.get_logger().info("Activating Robotiq Gripper...")
        # Clear any old data
        self.send_gripper_raw([0x0000, 0x0000, 0x0000])
        time.sleep(0.5)
        # Activation: rACT=1 (Action Request)
        self.send_gripper_raw([0x0100, 0x0000, 0x0000])
        
        self.sub = self.create_subscription(Float64, '/gripper/command', self.listener_callback, 10)

    def send_gripper_raw(self, values):
        # We use a loop to handle the Pymodbus 2.5.3 'slave' vs 'unit' issue
        for arg in ['slave', 'unit']:
            try:
                kwargs = {arg: 9}
                self.client.write_registers(1000, values, **kwargs)
                return
            except Exception:
                continue

    def listener_callback(self, msg):
        pos = int(msg.data * 255)
        pos = max(0, min(255, pos)) 
        # rACT=1, rGTO=1 (Go To), Speed=0xFF, Force=0xFF
        payload = [0x0900, 0x0000 | pos, 0xFFFF] 
        self.send_gripper_raw(payload)
        self.get_logger().info(f'Sent: {pos}')

def main():
    rclpy.init()
    node = RobotiqHumbleBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Prevent the RCLError by checking if context is still ok
        if rclpy.ok():
            node.client.close()
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()