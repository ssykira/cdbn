import socket 
import json
import time

time.sleep(0.1)
HOST='192.168.1.103'
PORT=30003
BUFFER=4096
send_content = "movep([-1.05, -1.87, -1.74, -1.52, 0.92, 0.63],a=0.1,v=0.1)\n"
send_content1 = "movep([-1.569, -1.668, -1.932, -0.327, 1.571, 1.568],a=0.1,v=0.1)\n"
soc=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
soc.connect((HOST,PORT))
soc.send(send_content)
#soc.send(send_content1)
buf=soc.recv(BUFFER)
print('!!!!!!!!!!!!!!!!!')
print(buf)
print('first send has been done')
#send_content = "movep([-1.569, -1.668, -1.932, -0.327, 1.571, 1.568],a=0.1,v=0.1)\n"
#soc.close()
time.sleep(10)
soc.send(send_content1)
#soc=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
#soc.connect((HOST,PORT))
#soc.send(send_content1)
buf=soc.recv(BUFFER)
print('!!!!!!!!!!!!!!!!!')
print(buf)
print('first send has been done')
#send_content = "movep([-1.569, -1.668, -1.932, -0.327, 1.571, 1.568],a=0.1,v=0.1)\n"
soc.close()
