import minimalmodbus
import time

instrument = minimalmodbus.Instrument('/tmp/ttyUR', 9)
instrument.serial.baudrate = 115200
instrument.serial.timeout = 1.0

def super_safe_test():
    try:
        # 1. Clear everything
        print("Clearing and Resetting...")
        instrument.write_register(1000, 0x0000)
        time.sleep(1.5)
        
        # 2. Activate ONLY (Wait for the physical click/motion to end)
        print("Activating... Wait for the mechanical 'click'...")
        instrument.write_register(1000, 0x0100)
        time.sleep(5.0) # Give it plenty of time to finish calibration
        
        # 3. Check status before moving
        status = instrument.read_register(1000)
        print(f"Status before move: {hex(status)}")
        if (status & 0x3100) != 0x3100:
            print("Not ready to move. Still not Blue.")
            return

        # 4. VERY GENTLE MOVE
        # Instead of a block write, let's just send a Position Request
        # 0x09 is rACT=1 and rGTO=1. 
        # We write to 1000 to set the 'GoTo' bit, then 1002 to set position.
        print("Setting Action bits...")
        instrument.write_register(1000, 0x0900)
        time.sleep(0.1)
        
        print("Moving very slowly to position 100...")
        # Write Position (100), Speed (10), Force (10) as separate steps
        instrument.write_register(1003, 0x0A0A) # Speed 10, Force 10
        time.sleep(0.1)
        instrument.write_register(1002, 0x6400) # Position 100 (in high byte)
        
        time.sleep(3.0)
        print("Did it stay Blue?")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    super_safe_test()