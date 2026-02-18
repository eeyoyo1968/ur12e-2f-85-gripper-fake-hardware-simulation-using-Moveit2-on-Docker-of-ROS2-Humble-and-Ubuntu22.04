#!/usr/bin/env python3
"""
ROS2 Gripper Bridge for Robotiq 2F-85 via PolyScope X Socat Server.

PolyScope X Architecture:
  - Tool Comm Forwarder runs Socat server on port 54321
  - Socat forwards raw RS-485 bytes to gripper
  - We send Modbus RTU commands to port 54321

This replaces the URScript-based approach (port 30002) used in PolyScope 5.

Setup:
  1. On teach pendant: Start Tool Comm Forwarder Socat server
  2. Verify message shows "Socat server is running on port 54321"
  3. Run this script: ros2 run my_ur_description robotiq_polyscope_x_gripper_server.py
  
Compatible with: PolyScope X 10.x
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from std_msgs.msg import Float32
import socket
import struct
import threading
import time


class RobotiqPolyscopeXGripperServer(Node):
    """
    Robotiq gripper control via PolyScope X Socat server (port 54321).
    Sends raw Modbus RTU commands over TCP.
    """
    
    def __init__(self):
        super().__init__('robotiq_polyscope_x_gripper_server')

        # ── Parameters ──────────────────────────────────────────────────
        self.robot_ip    = self.declare_parameter('robot_ip', '192.168.2.2').value
        self.socat_port  = self.declare_parameter('socat_port', 54321).value
        self.slave_id    = self.declare_parameter('slave_id', 9).value  # Robotiq default

        # ── Gripper state ───────────────────────────────────────────────
        self.current_position = 0.0   # radians: 0.0 open, 0.8 closed
        self._lock = threading.Lock()
        self._gripper_activated = False

        # ── Action CLIENT to ros2_control (for RViz sync) ───────────────
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/robotiq_gripper_controller/follow_joint_trajectory'
        )

        # ── Subscribe to joint states ───────────────────────────────────
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10
        )

        # ── Command subscriber (Float32) ────────────────────────────────
        self.create_subscription(
            Float32,
            '/robotiq_bridge/command',
            self._command_cb,
            10
        )

        # ── Status publisher ────────────────────────────────────────────
        self._status_pub = self.create_publisher(Float32, '/robotiq_bridge/position', 10)
        self.create_timer(0.1, self._publish_status)

        # ── Activate gripper on startup ─────────────────────────────────
        threading.Thread(target=self._startup_activate, daemon=True).start()

        self.get_logger().info(
            f"RobotiqPolyscopeXGripperServer ready | {self.robot_ip}:{self.socat_port}"
        )
        self.get_logger().info(
            "Connects to PolyScope X Socat server (Tool Comm Forwarder port 54321)"
        )
        self.get_logger().info(
            "Send gripper commands to /robotiq_bridge/command (Float32, 0.0=open 0.8=closed)"
        )

    # ────────────────────────────────────────────────────────────────────
    # MODBUS RTU PROTOCOL (Robotiq 2F-85)
    # ────────────────────────────────────────────────────────────────────
    # Function codes:
    #   0x03 = Read Holding Registers
    #   0x10 = Write Multiple Registers (Preset Multiple Registers)
    #
    # Register addresses (base 0x03E8 = 1000):
    #   Write registers: 0x03E8-0x03ED (1000-1005)
    #     1000 (0x03E8): ACTION REQUEST (rACT, rGTO, etc.)
    #     1001 (0x03E9): Reserved
    #     1002 (0x03EA): Reserved  
    #     1003 (0x03EB): POSITION REQUEST (0x00-0xFF)
    #     1004 (0x03EC): SPEED (0x00-0xFF)
    #     1005 (0x03ED): FORCE (0x00-0xFF)
    #
    #   Read registers: 0x07D0-0x07D5 (2000-2005)
    #     2000 (0x07D0): GRIPPER STATUS (gACT, gGTO, gSTA, etc.)
    #     2003 (0x07D3): POSITION ECHO (actual position 0x00-0xFF)
    # ────────────────────────────────────────────────────────────────────

    def _modbus_crc16(self, data: bytes) -> int:
        """Calculate Modbus RTU CRC-16"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def _build_modbus_write(self, start_register: int, values: list) -> bytes:
        """
        Build Modbus RTU Write Multiple Registers (0x10) command.
        
        Args:
            start_register: Starting register address (e.g., 1000)
            values: List of 16-bit register values to write
        
        Returns:
            Complete Modbus RTU frame with CRC
        """
        num_registers = len(values)
        byte_count = num_registers * 2
        
        # Build frame without CRC
        frame = bytearray()
        frame.append(self.slave_id)                    # Slave ID
        frame.append(0x10)                             # Function code (Write Multiple)
        frame.extend(struct.pack('>H', start_register)) # Start address (big-endian)
        frame.extend(struct.pack('>H', num_registers))  # Number of registers
        frame.append(byte_count)                       # Byte count
        
        # Add register values (big-endian)
        for value in values:
            frame.extend(struct.pack('>H', value))
        
        # Calculate and append CRC (little-endian)
        crc = self._modbus_crc16(bytes(frame))
        frame.extend(struct.pack('<H', crc))
        
        return bytes(frame)

    def _build_modbus_read(self, start_register: int, num_registers: int) -> bytes:
        """
        Build Modbus RTU Read Holding Registers (0x03) command.
        """
        frame = bytearray()
        frame.append(self.slave_id)
        frame.append(0x03)  # Function code (Read Holding)
        frame.extend(struct.pack('>H', start_register))
        frame.extend(struct.pack('>H', num_registers))
        
        crc = self._modbus_crc16(bytes(frame))
        frame.extend(struct.pack('<H', crc))
        
        return bytes(frame)

    def _send_modbus_command(self, command: bytes, expect_response: bool = False) -> bytes:
        """
        Send Modbus RTU command via Socat server (port 54321).
        
        Returns:
            Response bytes if expect_response=True, else None
        """
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            sock.connect((self.robot_ip, self.socat_port))
            sock.sendall(command)
            
            if expect_response:
                response = sock.recv(256)
                sock.close()
                return response
            else:
                sock.close()
                return None
                
        except socket.timeout:
            self.get_logger().error(f"Socat connection timeout to {self.robot_ip}:{self.socat_port}")
            return None
        except Exception as e:
            self.get_logger().error(f"Socat socket error: {e}")
            return None

    def _startup_activate(self):
        """Activate gripper via Modbus RTU on startup"""
        time.sleep(2.0)
        self.get_logger().info("Activating Robotiq gripper via Socat (port 54321)...")

        # Step 1: Reset gripper (clear rACT)
        # Write to register 1000: ACTION = 0x00 (reset)
        reset_cmd = self._build_modbus_write(1000, [0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        self._send_modbus_command(reset_cmd)
        time.sleep(0.5)

        # Step 2: Activate gripper (set rACT = 1)
        # Write to register 1000: ACTION = 0x01 (activate)
        activate_cmd = self._build_modbus_write(1000, [0x01, 0x00, 0x00, 0x00, 0x00, 0x00])
        self._send_modbus_command(activate_cmd)
        time.sleep(1.5)

        # Step 3: Check activation status
        # Read register 2000 (gripper status)
        read_cmd = self._build_modbus_read(2000, 1)
        response = self._send_modbus_command(read_cmd, expect_response=True)
        
        if response and len(response) >= 5:
            # Parse response: [slave_id, func_code, byte_count, data_high, data_low, crc_low, crc_high]
            status = response[3]  # High byte of first register
            if status & 0x01:  # gACT bit set
                self._gripper_activated = True
                self.get_logger().info("✓ Gripper activated successfully via Socat")
            else:
                self.get_logger().warn("Gripper activation uncertain - attempting to proceed")
                self._gripper_activated = True  # Proceed anyway
        else:
            self.get_logger().warn("No response from gripper - check Socat server is running")
            self._gripper_activated = True  # Try to proceed

    def _joint_state_cb(self, msg: JointState):
        """Track gripper position from joint_states"""
        try:
            idx = msg.name.index('robotiq_85_left_knuckle_joint')
            with self._lock:
                self.current_position = msg.position[idx]
        except ValueError:
            pass

    def _command_cb(self, msg: Float32):
        """Receive gripper command"""
        position_rad = float(msg.data)
        position_rad = max(0.0, min(0.8, position_rad))
        self.get_logger().info(f"Bridge command received: {position_rad:.3f} rad")
        t = threading.Thread(target=self._execute_gripper_move, args=(position_rad,), daemon=True)
        t.start()

    def _execute_gripper_move(self, position_rad: float):
        """Execute gripper move via Socat + mirror to ros2_control"""
        if not self._gripper_activated:
            self.get_logger().warn("Gripper not yet activated")

        # Part 1: Physical move via Socat/Modbus
        modbus_ok = self._send_gripper_modbus(position_rad)
        if not modbus_ok:
            self.get_logger().error("Modbus move failed")

        # Part 2: Mirror to ros2_control for RViz
        self._send_ros2control_goal(position_rad)

    def _send_gripper_modbus(self, position_rad: float) -> bool:
        """Send gripper move command via Modbus RTU"""
        # Convert radians to Modbus position (0-255)
        gripper_pos = int((position_rad / 0.8) * 255)
        gripper_pos = max(0, min(255, gripper_pos))

        self.get_logger().info(f"Socat: gripper → {gripper_pos}/255 ({position_rad:.3f} rad)")

        # Write to registers 1000-1005:
        #   1000: ACTION = 0x09 (rACT=1, rGTO=1 = go to position)
        #   1001: Reserved = 0x00
        #   1002: Reserved = 0x00
        #   1003: POSITION = gripper_pos
        #   1004: SPEED = 255 (max speed)
        #   1005: FORCE = 100 (moderate force)
        
        values = [
            0x0900,           # Register 1000: ACTION (rACT | rGTO in high byte)
            0x0000,           # Register 1001: Reserved
            0x0000,           # Register 1002: Reserved
            gripper_pos,      # Register 1003: POSITION
            0xFF00,           # Register 1004: SPEED (255 in high byte)
            0x6400            # Register 1005: FORCE (100 in high byte)
        ]
        
        command = self._build_modbus_write(1000, values)
        response = self._send_modbus_command(command)
        
        return response is not None

    def _send_ros2control_goal(self, position_rad: float):
        """Mirror goal to ros2_control for RViz sync"""
        if not self._action_client.wait_for_server(timeout_sec=1.0):
            return

        point = JointTrajectoryPoint()
        point.positions = [position_rad]
        point.time_from_start = Duration(sec=2, nanosec=0)

        traj = JointTrajectory()
        traj.joint_names = ['robotiq_85_left_knuckle_joint']
        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        self._action_client.send_goal_async(goal)

    def _publish_status(self):
        """Publish current gripper position"""
        msg = Float32()
        with self._lock:
            msg.data = float(self.current_position)
        self._status_pub.publish(msg)


def main():
    rclpy.init()
    try:
        node = RobotiqPolyscopeXGripperServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
