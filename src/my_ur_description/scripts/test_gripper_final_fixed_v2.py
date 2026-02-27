import minimalmodbus
import time

instrument = minimalmodbus.Instrument('/tmp/ttyUR', 9)
instrument.serial.baudrate = 115200
instrument.serial.timeout = 1.0

def move_gripper_confirmed(pos, speed, force):
    print(f"Moving to Pos: {pos}...")
    try:
        # Step A: Ensure rACT and rGTO are set, and send target
        # Register 1000: 0x0900 (rACT=1, rGTO=1)
        payload = [0x0900, 0x0000, pos << 8, (speed << 8) | force]
        instrument.write_registers(1000, payload)
        
        # Step B: Wait and verify movement
        for _ in range(15):
            status = instrument.read_register(1000)
            # Check bits 3-4 of the status byte (gOBJ)
            # 0x00 = Moving, 0x01/02 = Stopped (Object or Position reached)
            motion_status = (status >> 14) & 0x03 
            
            if motion_status == 0x03: # Position Reached
                print("  Target Reached.")
                break
            elif motion_status == 0x01 or motion_status == 0x02:
                print("  Stopped (Object/Contact detected).")
                break
            
            time.sleep(0.2)
    except Exception as e:
        print(f"  Error: {e}")

def run_test():
    print("--- Initializing ---")
    instrument.write_register(1000, 0x0000) # Reset
    time.sleep(1.0)
    instrument.write_register(1000, 0x0100) # Activate
    time.sleep(4.0) # Wait for sweep

    # TEST: Explicitly send a 'Go To' bit 
    print("--- Enabling GoTo bit ---")
    instrument.write_register(1000, 0x0900)
    time.sleep(0.5)

    print("--- Starting Sequence ---")
    # Position, Speed, Force
    moves = [(255, 100, 50), (0, 100, 50), (128, 50, 50), (0, 255, 50)]
    
    for p, s, f in moves:
        move_gripper_confirmed(p, s, f)
        time.sleep(1.0)

if __name__ == "__main__":
    run_test()