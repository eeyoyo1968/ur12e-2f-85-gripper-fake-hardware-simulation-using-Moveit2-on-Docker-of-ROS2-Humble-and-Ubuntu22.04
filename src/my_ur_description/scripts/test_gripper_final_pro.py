import minimalmodbus
import time

# Use the exact settings that worked for you before
instrument = minimalmodbus.Instrument('/tmp/ttyUR', 9)
instrument.serial.baudrate = 115200
instrument.serial.timeout = 1.0

def move_gripper(pos, speed, force):
    """Sends atomic write and pings status to keep the watchdog happy."""
    print(f"Executing -> Pos: {pos}, Spd: {speed}, For: {force}")
    # payload: [Action, Reserved, Position, Speed/Force]
    payload = [0x0900, 0x0000, pos << 8, (speed << 8) | force]
    try:
        instrument.write_registers(1000, payload)
        # Hold for 2 seconds while pinging status (Heartbeat)
        for _ in range(10):
            instrument.read_register(1000)
            time.sleep(0.2)
    except Exception as e:
        print(f"Comm Error during move: {e}")

def run_once():
    try:
        print("--- Step 1: Initialization ---")
        instrument.write_register(1000, 0x0000) # Reset
        time.sleep(1.0)
        instrument.write_register(1000, 0x0100) # Activate
        instrument.write_register(1000, 0x0100) # Double-tap
        print("Waiting for Blue LED...")
        time.sleep(4.0)

        # List: (Position, Speed, Force)
        test_sequence = [
            (0,   100, 50),   # Open
            (128, 40,  20),   # Halfway, slow/soft
            (255, 60,  150),  # Closed, firm force
            (0,   200, 100)   # Open, fast
        ]

        print("--- Step 2: Running Sequence ---")
        for p, s, f in test_sequence:
            move_gripper(p, s, f)

        print("--- Test Complete ---")

    except Exception as e:
        print(f"Critical Error: {e}")

if __name__ == "__main__":
    run_once()