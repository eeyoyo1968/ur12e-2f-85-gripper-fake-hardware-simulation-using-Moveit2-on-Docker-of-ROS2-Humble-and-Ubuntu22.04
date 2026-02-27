import minimalmodbus
import time

instrument = minimalmodbus.Instrument('/tmp/ttyUR', 9)
instrument.serial.baudrate = 115200
instrument.serial.timeout = 1.0
instrument.clear_buffers_before_each_transaction = True

def soft_start_test():
    try:
        # Step 1: Clear and wait
        print("Forcing Reset and waiting for electronics to settle...")
        instrument.write_register(1000, 0x0000)
        time.sleep(2.0)
        
        # Step 2: Set LOW speed and force (Register 1003)
        # 0x0A0A sets speed to 10/255 and force to 10/255 (Very gentle)
        print("Setting low-power limits for calibration...")
        instrument.write_register(1003, 0x0A0A)
        time.sleep(0.5)
        
        # Step 3: Activation
        print("Sending Activation (rACT=1)...")
        instrument.write_register(1000, 0x0100)
        
        # Step 4: Long wait for physical sweep
        print("Waiting 6 seconds for calibration sweep...")
        time.sleep(6.0)
        
        # Step 5: Check Status - We need to see if it finally stayed Blue
        status = instrument.read_register(1000)
        print(f"Final Status: {hex(status)}")
        
        if (status & 0x3100) == 0x3100:
            print("SUCCESS: LED is BLUE. Moving now...")
            # Close
            instrument.write_registers(1000, [0x0900, 0x0000, 0xFF00])
            time.sleep(2.0)
            # Open
            instrument.write_registers(1000, [0x0900, 0x0000, 0x0000])
        else:
            # Read Fault Register (1002)
            fault = instrument.read_register(1002) & 0xFF
            print(f"FAILED: Gripper faulted. Code: {hex(fault)}")
            print("0x0B = Overload, 0x05 = Action delayed (low power), 0x09 = Comm loss")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    soft_start_test()