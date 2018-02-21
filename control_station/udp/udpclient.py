#!/usr/bin/python

import socket
import sys
import json

HOST = '192.168.1.3'
PORT = 5000

def send(data):
    try:
        # create UDP socket
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # convert and send data
        data = json.dumps(data)
        sent = s.sendto(bytes(data, 'UTF-8'), (HOST, PORT))
        # print('length of data:', sent)
    except socket.error as e:
        print('Failed to create socket!')
        print(e)
        sys.exit()
    finally:
        s.close()
