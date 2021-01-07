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
	SOCKET_HOST = "127.0.0.1"
	SOCKET_PORT = 9000
	
	serverSocket.bind((SOCKET_HOST,SOCKET_PORT))
	serverSocket.listen(0)
	
	while True:
		(clientConnected, clientAddress) = serverSocket.accept()
		dataFromClient = clientConnected.recv(1024)
		print(dataFromClient)

if __name__ == '__main__':
	app.run(debug=True, threaded=True)
	
   	 