#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import struct

class RobotiqRawModbusBridge(Node):
    def __init__(self):
        super().__init__('robotiq_modbus_bridge')
        self.robot_ip = self.declare_parameter('robot_ip', '192.168.2.2').value
        self.port = 502
        self.slave_id = 9
        self.last_val = -1
        
        self.create_subscription(JointState, '/joint_states', self._callback, 10)
        self.get_logger().info(f"Raw Modbus Bridge (No Library) on {self.robot_ip}")
        self._activate()

    def _send_raw(self, unit_id, fc, addr, data):
        """Builds a Raw Modbus TCP Packet and sends it via Socket"""
        # MBAP Header: TransID(2), ProtoID(2), Length(2), UnitID(1)
        # PDU: FunctionCode(1), Addr(2), Data(...)
        
        # For Write Multiple (FC 16 / 0x10)
        if fc == 16:
            payload = struct.pack(">BHHHB", unit_id, fc, addr, len(data), len(data)*2)
            for d in data:
                payload += struct.pack(">H", d)
        # For Write Single (FC 6 / 0x06)
        else:
            payload = struct.pack(">BHHH", unit_id, fc, addr, data)

        header = struct.pack(">HHH", 0, 0, len(payload))
        packet = header + payload

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(1.0)
                s.connect((self.robot_ip, self.port))
                s.sendall(packet)
                return True
        except Exception as e:
            self.get_logger().error(f"Socket Error: {e}")
            return False

    def _activate(self):
        # FC 6, Register 1000, Value 256 (0x0100)
        if self._send_raw(self.slave_id, 6, 1000, 256):
            self.get_logger().info("✓ Activation Sent. LED should turn Blue.")

    def _callback(self, msg):
        try:
            idx = msg.name.index('robotiq_85_left_knuckle_joint')
            val = int((msg.position[idx] / 0.8) * 255)
            val = max(0, min(255, val))

            if abs(val - self.last_val) > 2:
                self.last_val = val
                # FC 16, Start 1000, Values [0x0900, 0, pos<<8]
                self._send_raw(self.slave_id, 16, 1000, [2304, 0, val << 8])
                self.get_logger().info(f"Gripper -> {val}")
        except ValueError:
            pass

def main():
    rclpy.init()
    rclpy.spin(RobotiqRawModbusBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()