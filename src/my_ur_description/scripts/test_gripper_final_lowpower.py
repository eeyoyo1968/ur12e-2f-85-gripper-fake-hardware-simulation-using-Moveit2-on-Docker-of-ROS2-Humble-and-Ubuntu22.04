import minimalmodbus
import time

instrument = minimalmodbus.Instrument('/tmp/ttyUR', 9)
instrument.serial.baudrate = 115200
instrument.serial.timeout = 1.0

def run_low_power_test():
    try:
        print("Step 1: Hard Reset...")
        instrument.write_register(1000, 0x0000)
        time.sleep(1.0)
        
        # Step 2: Set very low speed and force limits before activation
        # Register 1003: High byte = Speed, Low byte = Force
        # 0x0101 is the lowest possible setting (approx 1%)
        print("Step 2: Pre-setting minimum power limits...")
        instrument.write_register(1003, 0x0101)
        time.sleep(0.2)
        
        print("Step 3: Sending Activation...")
        instrument.write_register(1000, 0x0100)
        
        print("Step 4: Waiting for Blue LED (Patient Polling)...")
        for i in range(40): # 8 seconds
            time.sleep(0.2)
            try:
                status = instrument.read_register(1000)
                if status != 0:
                    print(f"  Current Status: {hex(status)}")
                if (status & 0x3100) == 0x3100:
                    print("SUCCESS! LED is BLUE.")
                    return True
            except:
                continue
                
        print("Activation timed out. Read Fault Register (1002)...")
        fault = instrument.read_register(1002) & 0xFF
        print(f"Fault Code: {hex(fault)}")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    run_low_power_test()