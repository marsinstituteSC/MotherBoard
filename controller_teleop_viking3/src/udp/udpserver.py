#!/usr/bin/python

import socket
import sys

HOST = '192.168.1.3'
PORT = 5000


def server():
    # setup socket
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.bind((HOST, PORT))
    except socket.error as e:
        print('Unable to create socket')
        print(e)
        sys.exit()
    # look for data
    data, addr = s.recvfrom(1024)
    if data:
        # decode bytes
        data = data.decode('UTF-8')
        return data
    else:
        # TODO: set watchdog timer
        pass


if __name__ == '__main__':
    while True:
        server()
