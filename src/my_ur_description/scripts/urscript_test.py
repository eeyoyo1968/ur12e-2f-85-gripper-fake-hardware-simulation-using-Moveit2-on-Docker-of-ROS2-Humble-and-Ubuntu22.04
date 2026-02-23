python3 << 'PYTHON'
import socket
import time

script = """def test_gripper():
  rq_move_and_wait(1, 0)
  sleep(2)
  rq_move_and_wait(1, 1)
  sleep(2)
end
test_gripper()
"""

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(3.0)
sock.connect(('192.168.2.2', 30002))
sock.sendall(script.encode('utf-8'))
sock.close()
print("Gripper test sent!")
PYTHON




# In the container
python3 << 'PYTHON'
import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('192.168.2.2', 30002))
sock.sendall(b'popup("Test from Python")\n')
sock.close()
print("Sent!")
PYTHON


# In the container
python3 << 'PYTHON'
import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('192.168.2.2', 30020))
cmd = "rq_close_and_wait()\n"
sock.sendall(cmd.encode("utf-8"))
sock.close()
print("Sent!")
PYTHON


python3 << 'PYTHON'
import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect(('192.168.2.2', 30020))
close_cmd = "rq_close_and_wait()\n"
sock.sendall(close_cmd.encode("utf-8"))
open_cmd = "rq_open_and_wait()\n"
sock.sendall(open_cmd.encode("utf-8"))
sock.close()
print("Sent!")
PYTHON




python3 << 'PYTHON'
from pymodbus.client import ModbusTcpClient

robot_ip = "192.168.2.2"
client = ModbusTcpClient(robot_ip, port=502)

print("Connected:", client.connect())
result = client.read_holding_registers(0, 10)
print(result)

PYTHON