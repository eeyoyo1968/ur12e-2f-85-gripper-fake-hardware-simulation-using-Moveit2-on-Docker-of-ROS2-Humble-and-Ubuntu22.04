import minimalmodbus
import time

instrument = minimalmodbus.Instrument('/tmp/ttyUR', 9)
instrument.serial.baudrate = 115200
instrument.serial.timeout = 1.0

def move_now(pos, speed, force):
    # We send 0x09 in the first byte (rACT=1, rGTO=1) 
    # and the position in the third byte.
    # We use write_registers (plural) to ensure they arrive in one Modbus frame.
    payload = [0x0900, 0x0000, pos << 8, (speed << 8) | force]
    
    print(f"Sending Command: Pos {pos}...")
    instrument.write_registers(1000, payload)
    
    # Monitor the status for 2 seconds
    for i in range(10):
        status = instrument.read_register(1000)
        # We want to see bits indicating Activation (0x01) and GTO (0x08)
        # If status is 0x3100 or 0xb100, it's working.
        if i == 0:
            print(f"  Live Status: {hex(status)}")
        time.sleep(0.2)

def run():
    print("--- Hard Reset ---")
    instrument.write_register(1000, 0x0000)
    time.sleep(1.0)
    
    print("--- Activation ---")
    instrument.write_register(1000, 0x0100)
    time.sleep(5.0) # Wait for the click/sweep to finish
    
    # Test Moves
    move_now(255, 100, 50) # Close
    time.sleep(1.0)
    move_now(0, 100, 50)   # Open

if __name__ == "__main__":
    run()