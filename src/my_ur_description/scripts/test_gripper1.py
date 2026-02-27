import minimalmodbus
import time

instrument = minimalmodbus.Instrument('/tmp/ttyUR', 9)
instrument.serial.baudrate = 115200
instrument.serial.timeout = 1.0

def robust_activate():
    try:
        # Step 1: CLEAR FAULTS / RESET
        print("Clearing faults and resetting gripper...")
        # Write 0x0000 to Register 1000 to clear everything
        instrument.write_register(1000, 0x0000)
        time.sleep(0.5)
        
        # Step 2: ACTIVATE
        print("Sending Activation command (LED should flash then stay BLUE)...")
        # 0x0100 = rACT (Action bit 0)
        instrument.write_register(1000, 0x0100)
        
        # Step 3: WAIT FOR COMPLETION
        # Activation can take up to 2-3 seconds as the fingers move to find end-stops
        time.sleep(3.0) 
        
        # Step 4: VERIFY STATUS
        status = instrument.read_register(1000)
        # Register 1000 (gSTA) should now show 0x1100 or 0x3100 (ready/active)
        if status != 0:
            print(f"SUCCESS! Gripper stays Active. Status: {hex(status)}")
            
            print("Step 5: Testing Movement (CLOSE)...")
            # Action: 0x09 (Active + GoTo), Pos: 0xFF (Closed)
            instrument.write_registers(1000, [0x0900, 0x0000, 0xFF00])
            time.sleep(2)
            
            print("Step 6: Testing Movement (OPEN)...")
            instrument.write_registers(1000, [0x0900, 0x0000, 0x0000])
        else:
            print("FAILED: Gripper returned to status 0. Check for physical obstructions.")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    robust_activate()