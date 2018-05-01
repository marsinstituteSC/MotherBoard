import can
import time

# Configure socketcan (python-can)
bustype = 'socketcan_ctypes'
channel = 'can0'
can.rc['interface'] = bustype
can.rc['channel'] = channel
bus = can.interface.Bus(channel=channel, bustype=bustype)


def send_msg(ID, data):
	"""
	param ID:   CAN message ID
	param data: list of data
	"""
	global bus
	msg = can.Message(arbitration_id=ID, data=data, extended_id=False)
	msg.data = data
	try:
		bus.send(msg)
	except can.CanError:
		print('CAN error:', can.CanError)


def check_status(mutex, buffer, ID=None):
	"""
	param ID:   	CAN message ID
	param buffer:   Read buffer
	param mutex:  	Mutex lock for read buffer
	"""
	global bus
	msg = bus.recv()
	with mutex:
		if msg and not ID:
			# add to general buffer
			buffer[msg.arbitration_id] = msg.data
		elif msg and msg.arbitration_id == ID:
			# add to node-specific buffer
			buffer[time.time()] = list(msg.data)


def split_bytes(val, lvl):
	"""
	splits bytes for CAN msg
	MSB first
	param val:	intX where X > 8
	param lvl:	number of splits needed (2 for int16, 4 for int32 etc.)
	"""
	if lvl == 2:
		MSB = (val & 0xFF00) >> 8
		LSB = (val & 0x00FF)
		return MSB, LSB
	elif lvl == 4:
		B1 = (val & 0xFF000000) >> 24
		B2 = (val & 0x00FF0000) >> 16
		B3 = (val & 0x0000FF00) >> 8
		B4 = (val & 0x000000FF)
		return B1, B2, B3, B4
