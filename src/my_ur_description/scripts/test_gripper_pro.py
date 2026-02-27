import minimalmodbus
import time

instrument = minimalmodbus.Instrument('/tmp/ttyUR', 9)
instrument.serial.baudrate = 115200
instrument.serial.timeout = 0.5
instrument.clear_buffers_before_each_transaction = True

def heartbeat():
    """Reads status to reset the 1-second watchdog timer."""
    try:
        return instrument.read_register(1000)
    except:
        return None

def move_safe(pos, speed=64, force=64):
    """Atomic write: sets Action, Position, Speed, and Force at once."""
    # 0x0900 = rACT=1 (Active), rGTO=1 (GoTo)
    payload = [0x0900, 0x0000, pos << 8, (speed << 8) | force]
    instrument.write_registers(1000, payload)

print("Starting Pro-Mode Test...")

# 1. Reset & Activation (Double-Tap)
instrument.write_register(1000, 0x0000)
time.sleep(1.0)
instrument.write_register(1000, 0x0100)
instrument.write_register(1000, 0x0100)

# 2. Polling with Heartbeat (Crucial)
print("Waiting for Blue LED...")
for _ in range(30):
    status = heartbeat()
    if status and (status & 0x3100) == 0x3100:
        print("Ready!")
        break
    time.sleep(0.2)

# 3. Controlled Movement (Low Speed/Force to prevent power spike)
try:
    print("Closing gently...")
    move_safe(pos=255, speed=20, force=20) # Low power start
    
    # Keep heartbeat alive while moving
    for _ in range(10): 
        heartbeat()
        time.sleep(0.5)
        
    print("Opening gently...")
    move_safe(pos=0, speed=20, force=20)
    
    for _ in range(10):
        heartbeat()
        time.sleep(0.5)

except KeyboardInterrupt:
    pass