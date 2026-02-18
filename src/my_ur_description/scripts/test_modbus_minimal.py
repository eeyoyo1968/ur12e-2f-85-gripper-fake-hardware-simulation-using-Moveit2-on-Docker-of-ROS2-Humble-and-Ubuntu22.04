#!/usr/bin/env python3
"""
Minimal Modbus test for PolyScope X Socat server.
No ROS2 required - pure socket test.
"""

import socket
import struct
import time

ROBOT_IP = '192.168.2.2'
SOCAT_PORT = 54321
SLAVE_ID = 9

def modbus_crc16(data):
    """Calculate Modbus CRC-16"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def build_modbus_write(start_register, values):
    """Build Modbus Write Multiple Registers command"""
    num_registers = len(values)
    byte_count = num_registers * 2
    
    frame = bytearray()
    frame.append(SLAVE_ID)
    frame.append(0x10)  # Write Multiple Registers
    frame.extend(struct.pack('>H', start_register))
    frame.extend(struct.pack('>H', num_registers))
    frame.append(byte_count)
    
    for value in values:
        frame.extend(struct.pack('>H', value))
    
    crc = modbus_crc16(bytes(frame))
    frame.extend(struct.pack('<H', crc))
    
    return bytes(frame)

def send_command(command):
    """Send Modbus command to Socat"""
    print(f"\nSending: {command.hex()}")
    
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3.0)
        
        print(f"Connecting to {ROBOT_IP}:{SOCAT_PORT}...")
        sock.connect((ROBOT_IP, SOCAT_PORT))
        print("✓ Connected")
        
        sock.sendall(command)
        print("✓ Sent")
        
        sock.close()
        return True
        
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

def main():
    print("="*60)
    print("MINIMAL MODBUS TEST - PolyScope X Socat")
    print("="*60)
    print(f"\nTarget: {ROBOT_IP}:{SOCAT_PORT}")
    print(f"Slave ID: {SLAVE_ID}\n")
    
    # Test 1: Reset gripper
    print("\n[1/3] RESET gripper...")
    reset_cmd = build_modbus_write(1000, [0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    if not send_command(reset_cmd):
        print("\n❌ Reset failed. Check Socat server!")
        return
    time.sleep(0.5)
    
    # Test 2: Activate gripper
    print("\n[2/3] ACTIVATE gripper...")
    activate_cmd = build_modbus_write(1000, [0x01, 0x00, 0x00, 0x00, 0x00, 0x00])
    if not send_command(activate_cmd):
        print("\n❌ Activate failed!")
        return
    print("\n⏱ Waiting 2 seconds for activation...")
    time.sleep(2.0)
    
    # Test 3: Move to position 128 (half closed)
    print("\n[3/3] MOVE to position 128/255 (half closed)...")
    move_cmd = build_modbus_write(1000, [
        0x0900,  # ACTION (rACT | rGTO)
        0x0000,  # Reserved
        0x0000,  # Reserved
        128,     # POSITION (half)
        0xFF00,  # SPEED (max)
        0x6400   # FORCE (100)
    ])
    if not send_command(move_cmd):
        print("\n❌ Move failed!")
        return
    
    print("\n✓ All commands sent successfully!")
    print("\n👀 Watch the physical gripper:")
    print("   - Should have activated (small movement)")
    print("   - Should be closing to half position now")
    print("\nIf gripper moved → Modbus communication works! ✅")
    print("If gripper didn't move → Check RS-485 wiring or gripper power")

if __name__ == '__main__':
    main()
