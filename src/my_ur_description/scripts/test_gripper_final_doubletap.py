import minimalmodbus
import time

instrument = minimalmodbus.Instrument('/tmp/ttyUR', 9)
instrument.serial.baudrate = 115200
instrument.serial.timeout = 1.0

def run_test():
    try:
        # STEP 1: INITIALIZATION (REPLICATING test_gripper2.py EXACTLY)
        print("Clearing faults...")
        instrument.write_register(1000, 0x0000)
        time.sleep(0.5)
        
        print("Sending DOUBLE Activation...")
        instrument.write_register(1000, 0x0100)
        # NO DELAY between these two, just like your working script
        instrument.write_register(1000, 0x0100) 
        
        print("Waiting for Blue LED...")
        activated = False
        for _ in range(25):
            time.sleep(0.2)
            status = instrument.read_register(1000)
            if (status & 0x3100) == 0x3100:
                print(f"Status is {hex(status)} - LED is BLUE.")
                activated = True
                break
        
        if not activated:
            print("Failed to stay blue. Check Tool Power (24V).")
            return

        # STEP 2: STABILIZED CONTROL
        # We write to 1000, 1001, 1002, and 1003 in one single block.
        # This is how the URCap ensures the gripper doesn't "panic."
        print("Moving to Position 128 (Halfway)...")
        # [Action, Reserved, Position, Speed/Force]
        # 0x0900 = rACT=1, rGTO=1
        # 0x8000 = Position 128 (0x80) shifted to High Byte
        # 0x6464 = Speed 100 (0x64), Force 100 (0x64)
        payload = [0x0900, 0x0000, 0x8000, 0x6464]
        instrument.write_registers(1000, payload)
        
        time.sleep(3.0)
        print("If it is still BLUE, the block-write worked.")

    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    run_test()