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
        
        # Step 2: ACTIVATE
        print("Sending Activation command... calibration started.")
        instrument.write_register(1000, 0x0100)
        
        # Step 3: POLLING STATUS (Wait up to 5 seconds)
        success = False
        for _ in range(25): # 25 * 0.2s = 5 seconds
            time.sleep(0.2)
            status = instrument.read_register(1000)
            
            # Check bit 3 of the high byte (gSTA - Gripper Status)
            # 0x3100 means Activation Logically Finished AND Active
            if (status & 0x3100) == 0x3100:
                print(f"Activation Complete! Status: {hex(status)}")
                success = True
                break
            elif status == 0:
                # If it drops back to 0, it faulted
                continue 

        if success:
            print("LED should be BLUE. Testing CLOSE...")
            instrument.write_registers(1000, [0x0900, 0x0000, 0xFF00])
        else:
            # Read the fault register (Register 1002, low byte is gFLT)
            fault = instrument.read_register(1002) & 0x00FF
            print(f"FAILED. Fault Code: {hex(fault)}")
            print("Note: 0x05 = Action delayed, 0x07 = Activation bit not set, 0x0B = Overload")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    robust_activate()