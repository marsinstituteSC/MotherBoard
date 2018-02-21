def drive(direction, speed):
	if direction == 'forward':
		# stop if currently going in reverse
		if speed == 0x00:
			speed = 0x7F
		else:
			speed = 0xFF
	elif direction == 'reverse':
		# stop if currently going forward
		if speed == 0xFF:
			speed = 0x7F
		else: 
			speed = 0x00
	return speed
