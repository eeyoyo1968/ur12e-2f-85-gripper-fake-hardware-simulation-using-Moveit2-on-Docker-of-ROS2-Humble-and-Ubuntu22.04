#!/usr/bin/env python3
"""
DEBUG VERSION: Robotiq PolyScope X Gripper Server
Adds extensive logging to diagnose connection issues.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import socket
import struct
import threading
import time
import sys


class RobotiqPolyscopeXGripperServerDebug(Node):
    def __init__(self):
        super().__init__('robotiq_polyscope_x_gripper_server_debug')

        self.robot_ip = self.declare_parameter('robot_ip', '192.168.2.2').value
        self.socat_port = self.declare_parameter('socat_port', 54321).value
        
        print(f"\n{'='*60}")
        print(f"DEBUG: RobotiqPolyscopeXGripperServer starting...")
        print(f"DEBUG: Target: {self.robot_ip}:{self.socat_port}")
        print(f"{'='*60}\n")
        
        # Test connection FIRST before doing anything else
        print("DEBUG: Step 1 - Testing basic network connectivity...")
        if not self._test_network():
            print("\n❌ FATAL: Cannot reach robot. Exiting.")
            sys.exit(1)
        
        print("\nDEBUG: Step 2 - Testing port 54321 connectivity...")
        if not self._test_port():
            print("\n❌ FATAL: Port 54321 not accessible. Check Socat server!")
            print("   → On teach pendant: Applications → Tool Comm Forwarder → Start Socat Server")
            sys.exit(1)
        
        print("\nDEBUG: Step 3 - Setting up ROS2 interfaces...")
        self._status_pub = self.create_publisher(Float32, '/robotiq_bridge/position', 10)
        self.create_subscription(Float32, '/robotiq_bridge/command', self._command_cb, 10)
        
        print("DEBUG: Step 4 - Attempting gripper activation...")
        threading.Thread(target=self._startup_activate, daemon=True).start()
        
        print("\n✓ Initialization complete. Waiting for activation...\n")

    def _test_network(self):
        """Test basic network connectivity with ping"""
        import subprocess
        try:
            result = subprocess.run(
                ['ping', '-c', '1', '-W', '2', self.robot_ip],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=3
            )
            if result.returncode == 0:
                print(f"  ✓ Robot reachable at {self.robot_ip}")
                return True
            else:
                print(f"  ✗ Cannot ping {self.robot_ip}")
                return False
        except Exception as e:
            print(f"  ✗ Ping failed: {e}")
            return False

    def _test_port(self):
        """Test if port 54321 is open"""
        print(f"  Attempting connection to {self.robot_ip}:{self.socat_port}...")
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(3.0)
            print(f"  Connecting...")
            sock.connect((self.robot_ip, self.socat_port))
            print(f"  ✓ Port {self.socat_port} is OPEN")
            sock.close()
            return True
        except socket.timeout:
            print(f"  ✗ Connection timed out")
            return False
        except ConnectionRefusedError:
            print(f"  ✗ Connection refused - Socat server not running?")
            return False
        except Exception as e:
            print(f"  ✗ Connection failed: {e}")
            return False

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
        """Build Modbus RTU Write Multiple Registers command"""
        num_registers = len(values)
        byte_count = num_registers * 2
        
        frame = bytearray()
        frame.append(9)  # Slave ID (Robotiq default)
        frame.append(0x10)  # Function code
        frame.extend(struct.pack('>H', start_register))
        frame.extend(struct.pack('>H', num_registers))
        frame.append(byte_count)
        
        for value in values:
            frame.extend(struct.pack('>H', value))
        
        crc = self._modbus_crc16(bytes(frame))
        frame.extend(struct.pack('<H', crc))
        
        return bytes(frame)

    def _send_modbus_command(self, command: bytes, expect_response: bool = False, timeout: float = 2.0):
        """Send Modbus command with detailed logging"""
        print(f"\n  → Sending Modbus command ({len(command)} bytes): {command.hex()}")
        
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(timeout)
            
            print(f"  → Connecting to {self.robot_ip}:{self.socat_port}...")
            sock.connect((self.robot_ip, self.socat_port))
            print(f"  ✓ Connected")
            
            print(f"  → Sending data...")
            sock.sendall(command)
            print(f"  ✓ Sent")
            
            if expect_response:
                print(f"  → Waiting for response (timeout: {timeout}s)...")
                response = sock.recv(256)
                print(f"  ✓ Received {len(response)} bytes: {response.hex()}")
                sock.close()
                return response
            else:
                sock.close()
                print(f"  ✓ Command sent successfully (no response expected)")
                return None
                
        except socket.timeout:
            print(f"  ✗ Socket timeout after {timeout}s")
            return None
        except ConnectionRefusedError:
            print(f"  ✗ Connection refused - Socat stopped?")
            return None
        except Exception as e:
            print(f"  ✗ Socket error: {e}")
            return None

    def _startup_activate(self):
        """Activate gripper with detailed logging"""
        time.sleep(2.0)
        
        print("\n" + "="*60)
        print("GRIPPER ACTIVATION SEQUENCE")
        print("="*60)
        
        # Step 1: Reset
        print("\n[1/3] Sending RESET command...")
        reset_cmd = self._build_modbus_write(1000, [0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        self._send_modbus_command(reset_cmd, expect_response=False, timeout=3.0)
        time.sleep(0.5)
        
        # Step 2: Activate
        print("\n[2/3] Sending ACTIVATE command...")
        activate_cmd = self._build_modbus_write(1000, [0x01, 0x00, 0x00, 0x00, 0x00, 0x00])
        self._send_modbus_command(activate_cmd, expect_response=False, timeout=3.0)
        time.sleep(1.5)
        
        # Step 3: Check status
        print("\n[3/3] Reading gripper status...")
        read_cmd = self._build_modbus_read(2000, 1)
        response = self._send_modbus_command(read_cmd, expect_response=True, timeout=3.0)
        
        if response and len(response) >= 5:
            status = response[3]
            print(f"\n  Status byte: 0x{status:02X}")
            if status & 0x01:
                print("  ✓ Gripper activated (gACT bit set)")
            else:
                print("  ⚠ Gripper activation unclear (gACT bit not set)")
        else:
            print("  ⚠ No valid response - gripper may still work")
        
        print("\n" + "="*60)
        print("ACTIVATION SEQUENCE COMPLETE")
        print("="*60 + "\n")
        
        print("✓ Ready to receive commands on /robotiq_bridge/command")
        print("  Test with: ros2 topic pub --once /robotiq_bridge/command std_msgs/msg/Float32 \"{data: 0.4}\"\n")

    def _build_modbus_read(self, start_register: int, num_registers: int) -> bytes:
        """Build Modbus RTU Read Holding Registers command"""
        frame = bytearray()
        frame.append(9)  # Slave ID
        frame.append(0x03)  # Function code (Read Holding)
        frame.extend(struct.pack('>H', start_register))
        frame.extend(struct.pack('>H', num_registers))
        
        crc = self._modbus_crc16(bytes(frame))
        frame.extend(struct.pack('<H', crc))
        
        return bytes(frame)

    def _command_cb(self, msg: Float32):
        """Handle gripper commands"""
        position_rad = max(0.0, min(0.8, float(msg.data)))
        print(f"\n{'─'*60}")
        print(f"GRIPPER COMMAND: {position_rad:.3f} rad")
        print(f"{'─'*60}")
        
        threading.Thread(target=self._execute_move, args=(position_rad,), daemon=True).start()

    def _execute_move(self, position_rad: float):
        """Execute gripper move"""
        gripper_pos = int((position_rad / 0.8) * 255)
        gripper_pos = max(0, min(255, gripper_pos))
        
        print(f"  Target position: {gripper_pos}/255 ({position_rad:.3f} rad)")
        
        values = [
            0x0900,      # ACTION (rACT | rGTO)
            0x0000,      # Reserved
            0x0000,      # Reserved
            gripper_pos, # POSITION
            0xFF00,      # SPEED (max)
            0x6400       # FORCE (100)
        ]
        
        command = self._build_modbus_write(1000, values)
        self._send_modbus_command(command, expect_response=False, timeout=3.0)
        
        print(f"{'─'*60}\n")


def main():
    print("\n" + "="*60)
    print("ROBOTIQ POLYSCOPE X GRIPPER SERVER - DEBUG MODE")
    print("="*60 + "\n")
    
    rclpy.init()
    
    try:
        node = RobotiqPolyscopeXGripperServerDebug()
        print("Node created, spinning...\n")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n⚠ Interrupted by user (Ctrl+C)")
    except Exception as e:
        print(f"\n\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nShutting down...")
        rclpy.shutdown()
        print("✓ Shutdown complete\n")


if __name__ == '__main__':
    main()
