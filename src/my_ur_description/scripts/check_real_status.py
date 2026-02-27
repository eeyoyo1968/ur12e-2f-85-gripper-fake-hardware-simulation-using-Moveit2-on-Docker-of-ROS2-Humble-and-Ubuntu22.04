import minimalmodbus
import time

instrument = minimalmodbus.Instrument('/tmp/ttyUR', 9)
instrument.serial.baudrate = 115200
instrument.serial.timeout = 1.0

def get_detailed_status():
    try:
        # We read 3 registers starting from 2000
        # 2000: Gripper Status, Object Status, Fault Status
        # 2001: Reserved & Final Position Request Echo
        # 2002: Actual Position & Current
        status_regs = instrument.read_registers(2000, 3)
        
        gSTA = (status_regs[0] >> 8) & 0xFF  # Gripper Status Byte
        fault = status_regs[0] & 0xFF        # Fault Byte
        pos_echo = (status_regs[1] >> 8) & 0xFF
        actual_pos = (status_regs[2] >> 8) & 0xFF
        current = status_regs[2] & 0xFF
        
        print(f"--- Real Status (Reg 2000) ---")
        print(f"Gripper Status Byte: {hex(gSTA)} (Should be 0x31 if Active & Ready)")
        print(f"Fault Code:          {hex(fault)}")
        print(f"Actual Position:     {actual_pos}")
        print(f"Motor Current:       {current * 10}mA")
        
        return gSTA
    except Exception as e:
        print(f"Failed to read status: {e}")
        return None

if __name__ == "__main__":
    get_detailed_status()