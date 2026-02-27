import minimalmodbus
import time
import sys

# Standard setup
instrument = minimalmodbus.Instrument('/tmp/ttyUR', 9)
instrument.serial.baudrate = 115200
instrument.serial.timeout = 1.0

def set_gripper(pos, speed=255, force=255):
    """
    Sends a block of 4 registers at once (Action, Reserved, Position, Speed/Force).
    This is the most stable way to control the 2F-85.
    """
    # 0x0900 = rACT=1, rGTO=1
    speed_force = (speed << 8) + force
    payload = [0x0900, 0x0000, pos << 8, speed_force]
    try:
        instrument.write_registers(1000, payload)
    except Exception as e:
        print(f"Communication error during move: {e}")

def run_stable_test():
    try:
        # 1. CLEAR SERIAL BUFFER
        print("Cleaning serial buffers...")
        instrument.clear_buffers_before_each_transaction = True
        
        # 2. INITIALIZATION (test_gripper2 logic)
        print("Step 1: Resetting...")
        instrument.write_register(1000, 0x0000)
        time.sleep(1.0)
        
        print("Step 2: Activating (Double-Call Strategy)...")
        instrument.write_register(1000, 0x0100)
        time.sleep(0.1)
        instrument.write_register(1000, 0x0100) # Replicating your working script
        
        # 3. PATIENT POLLING
        print("Step 3: Waiting for BLUE LED...")
        success = False
        for i in range(40):
            time.sleep(0.2)
            try:
                status = instrument.read_register(1000)
                if (status & 0x3100) == 0x3100:
                    print(f"SUCCESS: Gripper is Blue. Status: {hex(status)}")
                    success = True
                    break
            except:
                continue
        
        if not success:
            print("Activation Failed again. Please check if another script is running.")
            return

        # 4. CONTROL LOOP
        print("\n--- Testing Force and Distance ---")
        # Test 1: Close halfway with low force
        print("Moving to position 128 (Halfway) with Low Force...")
        set_gripper(pos=128, speed=100, force=20)
        time.sleep(2.0)
        
        # Test 2: Full Open
        print("Opening Fully...")
        set_gripper(pos=0, speed=255, force=255)
        time.sleep(2.0)
        
        print("Test Complete. Keeping connection alive for 5s to prevent watchdog fault...")
        time.sleep(5.0)

    except Exception as e:
        print(f"Major Error: {e}")

if __name__ == "__main__":
    run_stable_test()