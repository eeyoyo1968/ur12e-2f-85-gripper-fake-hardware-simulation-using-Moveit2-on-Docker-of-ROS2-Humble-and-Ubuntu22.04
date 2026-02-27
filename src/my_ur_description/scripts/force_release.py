import minimalmodbus
import time

try:
    gripper = minimalmodbus.Instrument('/tmp/ttyUR', 9)
    gripper.serial.baudrate = 115200
    gripper.serial.timeout = 0.5

    print("Step 1: Deactivating to drop motor tension...")
    gripper.write_register(1000, 0x0000) 
    time.sleep(1)

    print("Step 2: Re-activating...")
    gripper.write_register(1000, 0x0100)
    time.sleep(2) # Wait for the 'clunk' sound

    print("Step 3: Forcing Open command...")
    # 0x0900 = Action, 3 = Position Open, 0x6432 = Speed/Force
    gripper.write_registers(1000, [0x0900, 3, 0x6432])
    print("Release successful.")

except Exception as e:
    print(f"Failed to release: {e}")