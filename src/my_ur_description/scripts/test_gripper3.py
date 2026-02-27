import minimalmodbus
import time

# Use the settings known to work for your port
instrument = minimalmodbus.Instrument('/tmp/ttyUR', 9)
instrument.serial.baudrate = 115200
instrument.serial.timeout = 1.0
instrument.clear_buffers_before_each_transaction = True

def force_test():
    try:
        # Step 1: Check if we can even read the ID
        # Register 1010 often contains gripper info/firmware
        try:
            ver = instrument.read_register(1000)
            print(f"Initial Register 1000 read: {hex(ver)}")
        except:
            print("Cannot read register 1000. Serial link is dead.")
            return

        # Step 2: Clear and Activate
        print("Clearing faults...")
        instrument.write_register(1000, 0x0000)
        time.sleep(1.0)
        
        print("Sending Activation (rACT=1)...")
        instrument.write_register(1000, 0x0100)
        
        # Step 3: Monitor with a wider catch
        for i in range(30):
            time.sleep(0.3)
            status = instrument.read_register(1000)
            
            # Print any change in status
            if status != 0:
                print(f"Status changed! New Status: {hex(status)}")
            
            # 0x3100 is the goal: gSTA=3 (Active), rACT=1
            if (status & 0x3100) == 0x3100:
                print("SUCCESS: Gripper is Blue and Ready.")
                return True
        
        print("Final Status stayed 0x0. The command 0x0100 was ignored.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    force_test()