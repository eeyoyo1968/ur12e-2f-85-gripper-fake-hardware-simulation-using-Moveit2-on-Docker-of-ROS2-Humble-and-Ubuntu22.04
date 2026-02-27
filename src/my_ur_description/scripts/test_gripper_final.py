import minimalmodbus
import time

# Settings from your working test_gripper2.py
instrument = minimalmodbus.Instrument('/tmp/ttyUR', 9)
instrument.serial.baudrate = 115200
instrument.serial.timeout = 1.0

def set_gripper(pos, speed=255, force=255):
    """Controls position (0-255), speed (0-255), and force (0-255)."""
    # 0x0900 = rACT (Active) and rGTO (Go To) bits high
    speed_force = (speed << 8) + force
    payload = [0x0900, 0x0000, pos << 8, speed_force]
    instrument.write_registers(1000, payload)

def run_test():
    try:
        print("Starting Activation (test_gripper2 logic)...")
        instrument.write_register(1000, 0x0000) # Reset
        time.sleep(1.0)
        
        # Double-call activation as required by your setup
        instrument.write_register(1000, 0x0100) 
        instrument.write_register(1000, 0x0100)
        
        print("Waiting for Blue LED...")
        for _ in range(25):
            time.sleep(0.2)
            status = instrument.read_register(1000)
            if (status & 0x3100) == 0x3100: #
                print("LED is BLUE. Hardware Ready.")
                break
        
        # Keep the program running in a loop to prevent timeout
        print("Testing Control (Press Ctrl+C to stop)...")
        while True:
            print("Closing halfway with low force...")
            set_gripper(pos=128, speed=100, force=50)
            time.sleep(3.0)
            
            print("Opening fully...")
            set_gripper(pos=0, speed=255, force=255)
            time.sleep(3.0)
            
    except KeyboardInterrupt:
        print("Stopping test.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    run_test()