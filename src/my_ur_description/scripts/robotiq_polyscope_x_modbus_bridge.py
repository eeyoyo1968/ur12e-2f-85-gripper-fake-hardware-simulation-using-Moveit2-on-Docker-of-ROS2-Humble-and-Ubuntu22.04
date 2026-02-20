#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from pymodbus.client import ModbusTcpClient

# Updated imports for modern pymodbus 3.x
try:
    from pymodbus.pdu import ModbusRequest
    # Some 3.x versions use this path
    from pymodbus.register_write_message import WriteSingleRegisterRequest, WriteMultipleRegistersRequest
except ImportError:
    # Other 3.x versions use this path
    from pymodbus.bit_write_message import WriteSingleCoilRequest
    from pymodbus.register_write_message import WriteSingleRegisterRequest, WriteMultipleRegistersRequest
except ModuleNotFoundError:
    # If the above fail, we use the most direct internal path
    from pymodbus.pdu.register_write_message import WriteSingleRegisterRequest, WriteMultipleRegistersRequest

class RobotiqModbusBridge(Node):
    def __init__(self):
        super().__init__('robotiq_modbus_bridge')
        
        self.robot_ip = self.declare_parameter('robot_ip', '192.168.2.2').value
        self.slave_id = 9 
        self.client = ModbusTcpClient(self.robot_ip)
        self.last_val = -1
        
        self.create_subscription(JointState, '/joint_states', self._callback, 10)
        
        self.get_logger().info(f"Modbus Bridge Active on {self.robot_ip}.")
        self._init_gripper()

    def _init_gripper(self):
        if self.client.connect():
            # Use slave=self.slave_id inside the Request object
            req = WriteSingleRegisterRequest(1000, 0x0100, slave=self.slave_id)
            self.client.execute(req)
            self.get_logger().info("✓ Activation command executed (Check for Blue LED).")
        else:
            self.get_logger().error("Could not connect to Robot Modbus Server.")

    def _callback(self, msg):
        try:
            # Match the joint name from your URDF/Driver
            idx = msg.name.index('robotiq_85_left_knuckle_joint')
            val = int((msg.position[idx] / 0.8) * 255)
            val = max(0, min(255, val))

            if abs(val - self.last_val) > 2:
                self.last_val = val
                pos_hi = val << 8
                # Write 3 registers starting at 1000
                req = WriteMultipleRegistersRequest(1000, [0x0900, 0x0000, pos_hi], slave=self.slave_id)
                self.client.execute(req)
                self.get_logger().info(f"Gripper Position -> {val}")
        except (ValueError, Exception):
            pass

def main():
    rclpy.init()
    node = RobotiqModbusBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.client.close()
        rclpy.shutdown()

if __name__ == '__main__':
    main()