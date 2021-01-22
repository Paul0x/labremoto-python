##############################################################################
#   Laboratorio Remoto de Robotica Movel - TCC
#   Arquivo Principal
##############################################################################
#   Author: Paulo Felipe - paulof (at) ufop.edu.br
#
##############################################################################
#  Contem o loop principal para controlar o robo.
##############################################################################
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
from entities.experimento import SessaoExperimento, SessaoExperimentoApontarParametros
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
			print now.strftime("%d/%m/%Y %H:%M")
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
			self.sessaoAtiva = self.sessaoService.getSessaoAtiva()
			print("Inicializado com sucesso.\n\n")
			self.mainLoop(videoSource)

	# Carrega os argumentos do ev3
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

		# Carrega a saida da telemetria
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

		# Parametros da camera
		videoSource = cv2.VideoCapture(self.args["video"])
		videoSource.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
		videoSource.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
		videoSource.set(cv2.CAP_PROP_FPS, 25)
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


	# Carrega o caminho realizado pelo robo e inicializa o PID
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

	# Constroi o caminho real que o robo realizara,
	# ou seja, diminui os pontos de objetivo que serao enviados ao controlador PID
	def buildRealPath(self, path, scale):
		pathLen = len(path)
		if (pathLen > 10):
			newPathLen = int(pathLen/10)
		else:
			newPathLen = pathLen
		counter = 0
		newPath = []
		for i in range(0, len(path), newPathLen):
			newPath.append((path[i][0]*scale, path[i][1]*scale))
		return newPath


	# Funcao para fazer o robo funcionar de fato
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
			data['currentPosX'] = 0
			data['currentPosY'] = 0
			data['goalPosX'] = 0
			data['goalPosY'] = 0
			data['linearVel'] = 0
			data['angularVel'] = 0
		else:
			data['currentPosX'] = self.ev3telemetry[0]
			data['currentPosY'] = self.ev3telemetry[1]
			data['goalPosX'] = self.ev3telemetry[2]
			data['goalPosY'] = self.ev3telemetry[3]
			data['linearVel'] = self.ev3telemetry[4]
			data['angularVel'] = self.ev3telemetry[5]
		if (self.running == True):
			data['running'] = 1
		else:
			data['running'] = 0
		with open("static/ev3data.json", "w") as outputfile:
			json.dump(data, outputfile)

	# Processamento da imagem para mostrar na camera
	def processaImagemCamera(self, videoSource):
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
			
			# Caso ambos os objetos sejam reconhecido, faz o processamento para mostrar o robo na tela
			if(ev3.front.x != -1 and ev3.center.x != -1):
				ev3.th = self.getOrientation(ev3)
				self.utils.printRobot(ev3, frame, graph, 10)
			
			#Envia os dados do EV3 para o processador de imagens
			self.utils.ev3 = ev3

			return frame, graph, ev3

	# Funcao principal para rodar os experimentos do site
	def runExperimento(self, frame, graph, ev3) :

		# Verifica se existe sinal para rodar os experimentos
		# 1 = Rodar | 0 = Parar
		self.runStatus = self.db.getRodarExperimentoStatus()

		# Busca a sessao atual
		self.sessaoAtiva = self.sessaoService.getSessaoAtiva()
		
		# Verifica se o robo nao esta rodando e existe pedido para rodar
		if (self.running == False and self.runStatus == 1):
			# Verifica se existe sessao ativa
				if(self.sessaoAtiva is not None):
					# Existe sessao ativa, procura experimento ativo
					self.experimentoAtivo = self.db.getExperimentoAtivo(self.sessaoAtiva.id)
					if(self.experimentoAtivo is not None):
						# Configura experimento e manda rodar, com base no experimento
						self.configurarExperimento()
					else:
						# Para o pedido de rodar, nao existe experimento ativo
						self.db.setRodarExperimentoStatus(0)
				else:
					# Nao existe sessao ativa, parar sinal de rodar
					self.db.setRodarExperimentoStatus(0)
		# Verifica se o robo esta rodando e nao existe pedido para rodar
		elif (self.running == True and self.runStatus == 0):
			# Para de executar o robo
			self.running = False
			self.robotController.stopRobot()
		# Verifica se o robo esta rodando e existe pedido para rodar
		elif (self.running == True and self.runStatus == 1):
			# Procede com a execucao do experimento
			self.runRobot(ev3, graph,frame)

	# Configura os experimentos para rodar	
	def configurarExperimento(self):	

		# Constantes para configuracao
		ESCALA = 4
		ESCALA_FATOR_SEGURANCA = 3

		# Verifica se o experimento e de apontar
		if (int(self.experimentoAtivo.codExperimento) == 1) :
			self.experimentoAtivo.parametros = self.db.getParametrosExperimentoApontar(self.experimentoAtivo.codigo)
			self.utils.getImageMap(
				self.utils.graph,
				self.experimentoAtivo.parametros.objetivoX, 
				self.experimentoAtivo.parametros.objetivoY,
				ESCALA,
				ESCALA_FATOR_SEGURANCA,
				self.experimentoAtivo)
			self.loadPath()
			# Inicializa o robo
			self.running = True
				

		
	# Loop principal
	def mainLoop(self, videoSource):
		self.pts = deque(maxlen=self.args["buffer"])

		while True:		
			(frame, graph, ev3) = self.processaImagemCamera(videoSource)
		
			# Validacao para ver se a imagem esta carregada
			if hasattr(self.utils, "graph"): 
				self.runExperimento(frame, graph, ev3)

			# Finaliza o programa caso aperte a tecla q
			if self.utils.printImage(frame, graph) == ord("q"):
				break

			# Transforma o frame em .jpg para fazer o stream
			self.generateWebFrame(frame, graph)

			# Imprime arquivo JSON
			self.generateWebEv3Data()

if __name__ == '__main__':
	rospy.init_node('ev3_controlador_py', anonymous=True)
	cl = Main()
	
   	 