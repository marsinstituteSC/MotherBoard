def turn(direction, current_attitude):
	if direction == 'left' and current_attitude > 0x00:
		return current_attitude - 1
	elif direction == 'right' and current_attitude < 0xFF:
		return current_attitude + 1
	else:
		return current_attitude
