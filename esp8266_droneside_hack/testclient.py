import socket
import sys
import time
TCP_IP = '192.168.4.1'
TCP_PORT = 80
BUFFER_SIZE = 1024


def connect():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((TCP_IP, TCP_PORT))
    num = 0
    lx = 0
    s.send("START\n")
    while(True):
        msg = "LX=" + str(lx) + " LY=513 RX=511 RY=514 \n"
        s.send(msg)
        lx += 1
        print "sent", num
        num += 1
        time.sleep(0.1)
        sys.stdout.flush()
    s.close()



connect()
