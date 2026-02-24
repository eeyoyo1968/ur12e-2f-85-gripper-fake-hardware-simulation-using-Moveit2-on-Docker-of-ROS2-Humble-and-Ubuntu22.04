# Echo client program
import socket

HOST = "192.168.2.2" # The UR IP address
PORT = 30002 # UR secondary client
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

f = open ("Gripper1.script", "rb")   #Robotiq Gripper
#f = open ("setzero.script", "rb")  #Robotiq FT sensor

l = f.read(1024)
while (l):
    s.send(l)
    l = f.read(1024)
s.close()



