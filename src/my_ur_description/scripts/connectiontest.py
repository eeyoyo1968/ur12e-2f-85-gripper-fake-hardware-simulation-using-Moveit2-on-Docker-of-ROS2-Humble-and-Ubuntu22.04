import socket

ROBOT_IP = "192.168.2.2"  # Change to your Robot IP
PORT = 30002

test_script = """def test_comm():
    textmsg("NETWORK TEST: If you see this, Port 30002 is OPEN")
end
test_comm()
"""

try:
    print(f"Connecting to {ROBOT_IP}:{PORT}...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(2.0)
    s.connect((ROBOT_IP, PORT))
    s.sendall(test_script.encode('utf-8'))
    s.close()
    print("Command sent. Check the PolyScope Log/Messages tab.")
except Exception as e:
    print(f"FAILED to connect: {e}")