import minimalmodbus
import time

# Connection Setup (Matched to your successful tests)
instrument = minimalmodbus.Instrument('/tmp/ttyUR', 9)
instrument.serial.baudrate = 115200
instrument.serial.timeout = 0.5
instrument.clear_buffers_before_each_transaction = True

def move_and_wait(pos, speed, force, label):
    """Atomic write + Heartbeat to maintain the Blue LED."""
    print(f"\n[ACTION] {label}")
    print(f"Target -> Pos: {pos}, Spd: {speed}, For: {force}")
    
    # 0x0900 = rACT=1, rGTO=1
    # Register 1002 high byte = Position
    # Register 1003 high byte = Speed, low byte = Force
    payload = [0x0900, 0x0000, pos << 8, (speed << 8) | force]
    
    try:
        instrument.write_registers(1000, payload)
        
        # Keep the connection 'hot' for 3 seconds so the watchdog doesn't trip
        for _ in range(15):
            instrument.read_register(1000) # Heartbeat status read
            time.sleep(0.2)
    except Exception as e:
        print(f"Comm error during action '{label}': {e}")

def run_once():
    try:
        # 1. INITIALIZATION (The 'Double-Tap' method)
        print("--- Step 1: Initialization ---")
        instrument.write_register(1000, 0x0000) # Reset
        time.sleep(1.0)
        instrument.write_register(1000, 0x0100) # Activate
        instrument.write_register(1000, 0x0100) # Double-tap
        
        print("Waiting for calibration sweep (Watch for Blue LED)...")
        time.sleep(5.0) 

        # 2. THE TEST LIST (Position, Speed, Force, Label)
        # We test different distances and forces here
        test_steps = [
            (0,   255, 100, "Full Open"),
            (128, 40,  20,  "Halfway - Slow & Gentle"),
            (255, 100, 150, "Full Close - High Force"),
            (100, 200, 50,  "Partial Open - Fast"),
            (0,   100, 100, "Return to Ready")
        ]

        # 3. EXECUTE SEQUENCE
        print("\n--- Step 2: Executing Control Sequence ---")
        for pos, spd, f, label in test_steps:
            move_and_wait(pos, spd, f, label)

        print("\n--- Test Finished Successfully ---")
        print("LED will turn Red in 1 second because the program is exiting.")

    except Exception as e:
        print(f"Critical Failure: {e}")

if __name__ == "__main__":
    run_once()