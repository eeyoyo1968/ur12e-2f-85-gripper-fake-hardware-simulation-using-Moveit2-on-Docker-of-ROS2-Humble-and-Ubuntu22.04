import minimalmodbus
import time

# Settings from your successful test_gripper2.py
instrument = minimalmodbus.Instrument('/tmp/ttyUR', 9)
instrument.serial.baudrate = 115200
instrument.serial.timeout = 1.0

def send_command(pos, speed=255, force=255):
    """Packs registers exactly like the URCap to control force and distance."""
    # Register 1000: Action Request (0x0900 = rACT=1, rGTO=1)
    # Register 1002: Position (High byte)
    # Register 1003: Speed (High byte) and Force (Low byte)
    speed_force = (speed << 8) + force
    payload = [0x0900, 0x0000, pos << 8, speed_force]
    instrument.write_registers(1000, payload)

def interactive_test():
    try:
        print("Initializing Hardware (test_gripper2 logic)...")
        instrument.write_register(1000, 0x0000) # Reset
        time.sleep(0.5)
        instrument.write_register(1000, 0x0100) # Activate
        instrument.write_register(1000, 0x0100) # Double-call
        
        print("Waiting for Blue LED...")
        activated = False
        for _ in range(25):
            time.sleep(0.2)
            status = instrument.read_register(1000)
            if (status & 0x3100) == 0x3100:
                print("LED is BLUE. Hardware Ready.")
                activated = True
                break
        
        if not activated:
            print("Activation failed.")
            return

        print("\n--- Continuous Control Mode ---")
        print("Enter position 0-255 (or 'q' to quit)")
        print("0 = Open, 255 = Closed")
        
        while True:
            user_input = input("Target Position: ")
            if user_input.lower() == 'q':
                break
            
            try:
                pos = int(user_input)
                if 0 <= pos <= 255:
                    # Send command with medium speed/force to be safe
                    send_command(pos, speed=120, force=100)
                else:
                    print("Please enter a value between 0 and 255.")
            except ValueError:
                print("Invalid input.")
            
            # Brief heartbeat check to keep the bus active
            status = instrument.read_register(1000)
            if (status & 0x3100) != 0x3100:
                print(f"FAULT DETECTED! Status: {hex(status)}. LED likely turned RED.")
                break

    except Exception as e:
        print(f"Communication Error: {e}")

if __name__ == "__main__":
    interactive_test()