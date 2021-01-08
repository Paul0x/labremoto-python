##############################################################################
#   Reconhecimento e Processamento de Imagens @ Trabalho de Robotica 2019.1
#   Arquivo Flask
##############################################################################
#   Author: Paulo Felipe - paulof (at) ufop.edu.br
import numpy as np
from flask import Flask, render_template, Response
import cv2
import io
import imutils
import sys
import time
import socket

app = Flask(__name__)
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_record')
def video_record():	
	generate()
	return Response(generate(),mimetype = "multipart/x-mixed-replace; boundary=frame")

def generate():
	serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	SOCKET_HOST = "127.0.0.1"
	SOCKET_PORT = 9000
	
	serverSocket.bind((SOCKET_HOST,SOCKET_PORT))
	serverSocket.listen(0)
	(clientConnected, clientAddress) = serverSocket.accept()
	RecvData = clientConnected.recv(1024)
	img = io.BytesIO("")		
	while RecvData:
		dataFromClient = clientConnected.recv(1024)
		img.write(dataFromClient)
	serverSocket.close()
	yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + img + b'\r\n')


if __name__ == '__main__':
	app.run(debug=True, threaded=True)
	
   	 