##############################################################################
#   Laboratorio Remoto de Robotica Movel - TCC
#   Arquivo Principal
##############################################################################
#   Author: Paulo Felipe - paulof (at) ufop.edu.br
#
##############################################################################
#  Contem o loop principal para controlar o robo.
#
#
from collections import deque
import os
from math import atan2, acos, cos, sin, sqrt, pi
from imutils.video import VideoStream
from services.robotController import RobotController
from services.sessaoService import SessaoService
from datetime import datetime, timedelta
import numpy as np
import argparse
from utils.imageProcessingUtils import ImageProcessingUtils
from entities.ev3 import Ev3, Point
from services.database import Database
import cv2
import io
import imutils
import sys
import time
import rospy
import json
from datetime import datetime

np.set_printoptions(threshold=sys.maxsize)
class Main():

	# Construtor
	def __init__(self):
			now = datetime.now()
			print("Laboratorio Remoto de Robotica Movel")
			print("Trabalho de Conclusao de Curso")
			print("Paulo Felipe Possa Parreira")
			print now.strftime("%d/%m/%Y %h/%m")
			print("======================================")
			print("Inicializando...")
			self.utils = ImageProcessingUtils()
			self.utils.init()
			self.robotController = RobotController()
			self.trajetoriaEv3 = []
			self.loadArgs()
			videoSource = self.loadCameraImage()
			self.loadParams()
			self.running = False
			self.initDatabase()
			self.sessaoService = SessaoService(self.db)
			self.sessaoService.getSessaoAtiva()
			self.sessaoService.checkSessaoTimeout()
			print("Inicializado com sucesso.")
			self.mainLoop(videoSource)

	# Carrega os argumentos
	def loadParams(self):
		print("Carregando parametros do Ev3")
		#Primeiro range HSV - Origem
		self.origemLower = (97, 170, 170)
		self.origemUpper = (125, 255, 255)

		#Segundo range HSV - EV3
		self.ev3Lower = (145, 100, 150)
		self.ev3Upper = (185, 255, 255)

		#Terceiro range HSV - Walls (Obstaculos)
		self.wallsLower = (70, 50, 50)
		self.wallsUpper = (100, 255, 255)

		self.ev3telemetry = ()
		print("Parametros do Ev3 carregados.")
	
	# Carrega os argumentos do console
	def loadArgs(self):
		print("Carregando argumentos do script...")
		ap = argparse.ArgumentParser()
		ap.add_argument("-v", "--video", help="path to the (optional) video file")
		ap.add_argument("-b", "--buffer", type=int, default=2, help="max buffer size")
		self.args = vars(ap.parse_args())
		print("Argumentos do script carregados.")
	
	# Carrega a imagem da camera
	def loadCameraImage(self):
		print("Carregando imagem da camera")
		videoSource = cv2.VideoCapture(self.args["video"])
		videoSource.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
		videoSource.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
		videoSource.set(cv2.CAP_PROP_FPS, 25)
		#videoSource = cv2.VideoCapture('vid04.mp4')
		time.sleep(2.0)
		print("Imagem da camera carregada.")
		return videoSource

	# Inicializa a classe do banco de dados
	def initDatabase(self):
		print("Inicializando conexao com o banco de dados")
		self.db = Database()
		self.db.initDatabase()
		print("Conexao estalecida.")

	# Realiza o calculo do angulo do ev3 com relacao ao seu centro
	def getOrientation(self, ev3):
		vector = Point()
		u = Point()
		vector.x = ev3.front.x - ev3.center.x
		vector.y = ev3.front.y - ev3.center.y
		u.x = 1
		u.y = 0
		costh = (vector.x * u.x) / sqrt(vector.x**2 + vector.y ** 2) * sqrt(u.x**2)
		angle = acos(costh)
		if(ev3.front.y > ev3.center.y):
			angle = pi + (pi - angle)
		return angle
	
	def createRawImage(self):
		# Definicao do grafico de trajetoria
		gWidth = 1280
		gHeight = 720
		graph = np.ones((gHeight,gWidth,3), np.uint8)*255
		return graph


	def loadPath(self):
		path = self.utils.path
		self.goalX = self.utils.goal[0] * self.utils.mapScale
		self.goalY = self.utils.goal[1] * self.utils.mapScale
		self.path = self.buildRealPath(path, self.utils.mapScale)
		self.robotController.initPid()

		# O primeiro goal e o primeiro item do path
		self.currentGoal = Point()
		self.currentGoal.x = self.path[0][0]
		self.currentGoal.y = self.path[0][1]
		self.path.pop(0)

		self.running = True

	def buildRealPath(self, path, scale):
		pathLen = len(path)
		newPathLen = int(pathLen/10)
		counter = 0
		newPath = []
		for i in range(0, len(path), newPathLen):
			newPath.append((path[i][0]*scale, path[i][1]*scale))
		#print('Caminho Real')
		#print(newPath)
		return newPath


	def runRobot(self, ev3, graph, frame):
		pose = Point()
		pose.x = ev3.center.x
		pose.y = ev3.center.y
		pidResp, ev3telemetry = self.robotController.pidRun(graph, self.currentGoal, pose, ev3, False)
		self.ev3telemetry = ev3telemetry
		if(pidResp == True):
			if(len(self.path) == 0):
				self.running = False
				self.robotController.stopRobot()
				self.utils.trajetoria = False
			else:
				self.currentGoal = Point()
				self.currentGoal.x = self.path[0][0]
				self.currentGoal.y = self.path[0][1]
				self.path.pop(0)

		# Desenha os objetivos no mapa		
		for point in self.path:
			cv2.circle(frame, (int(point[0]), int(point[1])), 2, (0, 0, 0), -1)
		
		# Imprime arquivo JSON
		self.generateWebEv3Data()

	#
	# Funcao para gerar uma imagem jpg - utilizada para o servidor web
	#
	def generateWebFrame(self, frame, graph):
		ret, buffer = cv2.imencode('.jpg', frame)
		frameImg = buffer.tobytes()
		file = open("static/imgVideo.jpg", "wb")
		file.write(frameImg)


		ret, buffer = cv2.imencode('.jpg', graph)
		frameImg = buffer.tobytes()
		file = open("static/imgGraph.jpg", "wb")
		file.write(frameImg)

	#
	#	Geracao do arquivo JSON para consumo do servidor WEB
	#
	def generateWebEv3Data(self):
		data = {}
		if self.ev3telemetry == ():
			return
		print(self.ev3telemetry)
		data['currentPosX'] = self.ev3telemetry[0]
		data['currentPosY'] = self.ev3telemetry[1]
		data['goalPosX'] = self.ev3telemetry[2]
		data['goalPosY'] = self.ev3telemetry[3]
		data['linearVel'] = self.ev3telemetry[4]
		data['angularVel'] = self.ev3telemetry[5]
		with open("static/ev3data.json", "w") as outputfile:
			json.dump(data, outputfile)
		
	# Loop de reconhcimento
	def mainLoop(self, videoSource):
		self.pts = deque(maxlen=self.args["buffer"])


		while True:		
			count = 50
			graph = self.createRawImage()
			# Pega o frame da camera
			frame = self.utils.getFrame(videoSource)

			# Altera o espaco para hsv
			hsv = self.utils.frameToHsv(frame)
			hsv2 = self.utils.frameToHsv(frame)
			hsv3 = self.utils.frameToHsv(frame)

			# Aplica as mascaras para o Ev3 e a origem
			maskEv3Front = self.utils.applyMaskToFrame(hsv, self.ev3Lower, self.ev3Upper)		
			maskOrigem = self.utils.applyMaskToFrame(hsv2, self.origemLower, self.origemUpper)		
			
			# Aplica a mascara para os obstaculos
			maskObstaculo = self.utils.applyMaskToFrame(hsv3, self.wallsLower, self.wallsUpper)

			# Busca os contornos encontrados
			contoursOrigem = self.utils.getObjectsContours(maskOrigem)
			contoursEv3Front = self.utils.getObjectsContours(maskEv3Front)
			contoursWalls = self.utils.getObjectsContours(maskObstaculo)

			# Manda imprimir os obstaculos
			self.utils.printWalls(frame,graph, contoursWalls)

			# Reconhece o objeto, buscando o contorno encontrado com maior raio
			ev3 = Ev3()
			(ev3.front.x, ev3.front.y) = self.utils.recognizeObject(contoursEv3Front, frame, 'Ev3')
			(ev3.center.x, ev3.center.y) = self.utils.recognizeObject(contoursOrigem, frame, 'Origem')
			
			# Caso ambos os objetos sejam reconhecido, faz o processamento para mostrar a trajetoria
			if(ev3.front.x != -1 and ev3.center.x != -1):
				ev3.th = self.getOrientation(ev3)
				self.utils.printRobot(ev3, frame, graph, 10)
			
			#Envia os dados do EV3 para o processador de imagens
			self.utils.ev3 = ev3

			# Checa se existe trajetoria atual 
			if(self.utils.updateTrajetoria == True or (self.utils.trajetoria == True and self.running == False)):
				self.utils.updateTrajetoria = False
				self.loadPath()
			elif(self.utils.trajetoria == True and self.running == True):
				self.runRobot(ev3, graph,frame)

			# Finaliza o programa caso aperte a tecla q
			if self.utils.printImage(frame, graph) == ord("q"):
				break

			# Transforma o frame em .jpg para fazer o stream
			self.generateWebFrame(frame, graph)


if __name__ == '__main__':
	rospy.init_node('ev3_controlador_py', anonymous=True)
	cl = Main()
	
   	 