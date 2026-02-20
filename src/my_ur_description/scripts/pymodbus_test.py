from pymodbus.client import ModbusTcpClient
import time

ROBOT_IP = "192.168.2.2"
ID = 9  # Slave ID

client = ModbusTcpClient(ROBOT_IP)

def test_gripper():
    if not client.connect():
        print("❌ Connection Failed. Check Modbus TCP Server in PolyScope Settings.")
        return

    print("✅ Connected! Activating...")
    # Modern pymodbus 3.x uses 'slave' as a keyword
    client.write_register(1000, 0x0100, slave=ID)
    
    print("Waiting 5s for activation (Watch for Blue LED)...")
    time.sleep(5)

    print("Closing Gripper...")
    # Write 3 registers: 1000 (Action), 1001 (Reserved), 1002 (Position)
    # 0x0900 = GoTo, 0xFF00 = Position 255 (Closed)
    client.write_registers(1000, [0x0900, 0x0000, 0xFF00], slave=ID)
    
    time.sleep(3)
    print("Opening Gripper...")
    client.write_registers(1000, [0x0900, 0x0000, 0x0000], slave=ID)

    client.close()

if __name__ == "__main__":
    test_gripper()