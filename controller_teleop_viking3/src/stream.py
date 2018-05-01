#!/usr/bin/env python

# https://blog.miguelgrinberg.com/post/video-streaming-with-flask

from flask import Flask, Response
import time

app = Flask(__name__)

HOST = '192.168.1.3'	# on-board computer's IP address
PORT = 8080


@app.route('/')
def index():
	return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')


def gen():
	"""Video streaming generator function."""
	global img
	while True:
		try:
			with open('frame.jpeg', 'rb') as f:
				time.sleep(0.05) # 20fps
				yield(b'--frame\r\n'
					b'Content-Type: image/jpeg\r\n\r\n' + f.read() + b'\r\n')
		except Exception as e:
			print(e)


if __name__ == '__main__':
	app.run(host=HOST, port=PORT, threaded=True)
