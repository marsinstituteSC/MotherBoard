import can

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
    	msg = can.Message(arbitration_id=ID, data=data, extended_id=False)
	msg.data = data
	try:
		bus.send(msg)
	except can.CanError:
		print('CAN error:', can.CanError)

def check_status(ID):
    	"""
    	param ID:   CAN message ID
    	"""
    	# TODO: fix decoding
    	for msg in bus:
    	    if msg.arbitration_id == ID:
    	        print('Status of:', ID)
    	        for i in len(msg.data):
    	            print(data[i])

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
