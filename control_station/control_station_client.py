from inputs import get_gamepad # https://pypi.python.org/pypi/inputs
import sys, time, threading, socket, json

# Value mapping for Windows systems. Linux systems give different values.
vals = {
    'Axes': {
        0: 0,   # left joy, x-axis, >0 = right, <0 = left
        1: 0,   # left joy, y-axis, >0 = up, <0 = down
        2: 0,   # LT, 0 = not pressed, 255 = fully pressed
        3: 0,   # right joy, x-axis, >0 = right, <0 = left
        4: 0,   # right joy, y-axis, >0 = up, <0 = down
        5: 0,   # RT, 0 = not pressed, 255 = fully pressed
        6: 0,   # arrow keys, x-axis, 1 = right, -1 = left
        7: 0    # arrow keys, y-axis, 1 = down, -1 up
    },
    'Buttons': {
        0: 0,   # A
        1: 0,   # B
        2: 0,   # Y
        3: 0,   # X
        4: 0,   # LB
        5: 0,   # RB
        6: 0,   # START
        7: 0,   # SELECT
        8: 0,   # XBOX - NB! opens Game Bar in windows
        9: 0,   # left joy click
        10: 0   # right joy click
    }
}

# rover's IP address
HOST = '192.168.1.3'
PORT = 5000
# dead zone for joysticks
dead_zone = 3000
# thread run flag
running = True


def udp_client_send(data):
    try:
        # create UDP socket
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # convert and send data
        data = json.dumps(data)
        sent = s.sendto(bytes(data, 'UTF-8'), (HOST, PORT))
    except socket.error as e:
        print('Failed to create socket!')
        print(e)
        sys.exit()
    finally:
        s.close()


def sender():
    global vals
    global mutex
    global running
    while running:
        time.sleep(0.05) # 20 Hz
        # pass values from control station (client) to rover (server)
        udp_client_send(vals)
    sys.exit()

def updater():
    global vals
    global running
    while running:
        try:
            # fetch controller input events
            events = get_gamepad()
            # start thread to send data
            t = threading.Thread(target=sender)
            t.start()
        except Exception as e:
            print('\n{}'.format(e))
            running = False
            sys.exit()
        for event in events:
            # update controller values
            # Axes:
            if event.ev_type == 'Absolute' or event.ev_type == 'Sync':
                if event.code == 'ABS_X':
                    if event.state > dead_zone or event.state < -dead_zone:
                        vals['Axes'][0] = event.state
                    else:
                        vals['Axes'][0] = 0
                elif event.code == 'ABS_Y':
                    if event.state > dead_zone or event.state < -dead_zone:
                        vals['Axes'][1] = event.state
                    else:
                        vals['Axes'][1] = 0
                elif event.code == 'ABS_Z':
                    vals['Axes'][2] = event.state
                elif event.code == 'ABS_RX':
                    if event.state > dead_zone or event.state < -dead_zone:
                        vals['Axes'][3] = event.state
                    else:
                        vals['Axes'][3] = 0
                elif event.code == 'ABS_RY' :
                    if event.state > dead_zone or event.state < -dead_zone:
                        vals['Axes'][4] = event.state
                    else:
                        vals['Axes'][4] = 0
                elif event.code == 'ABS_RZ':
                    vals['Axes'][5] = event.state
                elif event.code == 'ABS_HAT0X':
                    vals['Axes'][6] = event.state
                elif event.code == 'ABS_HAT0Y':
                    vals['Axes'][7] = event.state
            # Buttons:
            elif event.ev_type == 'Key' or event.ev_type == 'Sync':
                if event.code == 'BTN_SOUTH':
                    vals['Buttons'][0] = event.state
                elif event.code == 'BTN_EAST':
                    vals['Buttons'][1] = event.state
                elif event.code == 'BTN_NORTH':
                    vals['Buttons'][2] = event.state
                elif event.code == 'BTN_WEST':
                    vals['Buttons'][3] = event.state
                elif event.code == 'BTN_TL':
                    vals['Buttons'][4] = event.state
                elif event.code == 'BTN_TR':
                    vals['Buttons'][5] = event.state
                elif event.code == 'BTN_SELECT':
                    vals['Buttons'][6] = event.state
                elif event.code == 'BTN_START':
                    vals['Buttons'][7] = event.state
                elif event.code == 'BTN_MODE':
                    vals['Buttons'][8] = event.state
                elif event.code == 'BTN_THUMBL':
                    vals['Buttons'][9] = event.state
                elif event.code == 'BTN_THUMBR':
                    vals['Buttons'][10] = event.state
        print(vals)


if __name__ == '__main__':
    try:
        updater()
    except KeyboardInterrupt:
        running = False
        print('\nStopping program.')
        sys.exit()
