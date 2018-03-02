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
