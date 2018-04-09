#!/usr/bin/python

import socket
import sys

HOST = '192.168.1.3'	# on-board computer's IP address
PORT = 5000


def server():
	try:
		# setup socket
		s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		s.bind((HOST, PORT))
		# look for data
		data, addr = s.recvfrom(1024)
	except socket.error as e:
		print('Exiting or unable to create socket')
		print(e)
		sys.exit()
	if data:
		# decode bytes
		data = data.decode('UTF-8')
		return data
	else:
		pass


if __name__ == '__main__':
	while True:
		server()
