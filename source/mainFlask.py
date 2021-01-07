##############################################################################
#   Reconhecimento e Processamento de Imagens @ Trabalho de Robotica 2019.1
#   Arquivo Flask
##############################################################################
#   Author: Paulo Felipe - paulof (at) ufop.edu.br
import numpy as np
from flask import Flask, render_template, Response
import cv2
import imutils
import sys
import time


app = Flask(__name__)
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_record')
def video_record():	
	return Response(generate(),mimetype = "multipart/x-mixed-replace; boundary=frame")

def generate():
	camera = cv2.VideoCapture("/dev/video1")
#	while True:
#		success, frame = camera.read()  # read the camera frame
#		if not success:
#			break
#		else:
#			ret, buffer = cv2.imencode('.jpg', frame)
#			frame = buffer.tobytes()
#			yield (b'--frame\r\n'
#					b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result
	while True:
		file = open("cameraImg.jpg", "rb")
		imgBytes = file.read();
        yield (b'--frame\r\n'
					b'Content-Type: image/jpeg\r\n\r\n' + imgBytes + b'\r\n')

if __name__ == '__main__':
	app.run(debug=True, threaded=True)
	
   	 