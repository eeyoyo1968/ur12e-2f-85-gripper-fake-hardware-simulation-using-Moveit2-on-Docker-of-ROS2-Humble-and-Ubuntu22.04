import minimalmodbus
import serial
import time

def test_id(slave_id):
    print(f"--- Testing ID {slave_id} ---")
    instrument = minimalmodbus.Instrument('/tmp/ttyUR', slave_id)
    instrument.serial.baudrate = 115200
    instrument.serial.bytesize = 8
    instrument.serial.parity = serial.PARITY_NONE
    instrument.serial.stopbits = 1
    instrument.serial.timeout = 1.0 
    
    # Robotiq grippers sometimes need a tiny 'silent period' 
    # to recognize the start of a Modbus RTU frame
    time.sleep(0.1) 
    
    try:
        # Read Register 1000 (Status)
        # If it returns a value, the connection is solid!
        val = instrument.read_register(1000)
        print(f"SUCCESS! ID {slave_id} responded: {val}")
        return True
    except Exception as e:
        print(f"Failed ID {slave_id}: {e}")
        return False

if __name__ == "__main__":
    if not test_id(9):
        test_id(1)