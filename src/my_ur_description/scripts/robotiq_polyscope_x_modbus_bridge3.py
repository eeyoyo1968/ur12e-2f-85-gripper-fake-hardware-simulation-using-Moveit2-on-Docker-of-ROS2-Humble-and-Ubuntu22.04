#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import socket
import struct
import time

class RobotiqFinalBridge(Node):
    def __init__(self):
        super().__init__('robotiq_modbus_bridge')
        self.robot_ip = self.declare_parameter('robot_ip', '192.168.2.2').value
        self.port = 502
        self.slave_id = 9
        self.last_val = -1
        
        self.get_logger().info(f"Connecting to {self.robot_ip} via Raw Modbus...")
        self._initialize_sequence()
        
        # Subscribe to joint states
        self.create_subscription(JointState, '/joint_states', self._callback, 10)

    def _send_raw(self, unit_id, fc, addr, data):
        """Constructs and sends a Modbus TCP packet manually."""
        if fc == 16: # Write Multiple Registers
            payload = struct.pack(">BHHHB", unit_id, fc, addr, len(data), len(data)*2)
            for d in data: payload += struct.pack(">H", d)
        else: # Write Single Register (FC 06)
            payload = struct.pack(">BHHH", unit_id, fc, addr, data)

        header = struct.pack(">HHH", 0, 0, len(payload))
        packet = header + payload

        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(0.5)
                s.connect((self.robot_ip, self.port))
                s.sendall(packet)
                return True
        except Exception as e:
            self.get_logger().error(f"Modbus Socket Error: {e}")
            return False

    def _initialize_sequence(self):
        """Step-by-step activation for Robotiq 2F-85"""
        self.get_logger().info("1/3: Clearing Gripper Registers...")
        self._send_raw(self.slave_id, 6, 1000, 0x0000)
        time.sleep(0.5)
        
        self.get_logger().info("2/3: Activating (rACT=1)...")
        self._send_raw(self.slave_id, 6, 1000, 0x0100)
        time.sleep(3.0) # Wait for activation beep/LED change
        
        self.get_logger().info("3/3: Setting GoTo bit (rGTO=1)...")
        # 0x0900 (2304) = Activate + GoTo
        self._send_raw(self.slave_id, 6, 1000, 0x0900)
        self.get_logger().info("✅ Gripper is now ready for ROS2 commands.")

    def _callback(self, msg):
        try:
            # Match the joint name from your echo output
            idx = msg.name.index('robotiq_85_left_knuckle_joint')
            raw_pos = msg.position[idx]
            
            # Convert 0.0-0.8 rad to 0-255 byte
            val = int((raw_pos / 0.8) * 255)
            val = max(0, min(255, val))

            # Only send if position changed to avoid flooding the robot
            if abs(val - self.last_val) > 1:
                self.last_val = val
                # Reg 1000: 0x0900, Reg 1001: 0, Reg 1002: (Pos << 8)
                self._send_raw(self.slave_id, 16, 1000, [0x0900, 0x0000, val << 8])
                self.get_logger().info(f"Targeting Pos: {val}/255")
        except ValueError:
            pass

def main():
    rclpy.init()
    rclpy.spin(RobotiqFinalBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()