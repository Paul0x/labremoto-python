from datetime import datetime

class SessaoExperimento():
        def __init__(self):
                self.codigo = 0
                self.codSessao = 0
                self.codExperimento = 0
                self.dtInicio = datetime.now()
                self.ativo = False
                self.parametros = None

class SessaoExperimentoApontarParametros():
        def __init__(self):
                self.codSessaoExperimento = 0
                self.algoritmoBusca = 0
                self.obstaculos = True
                self.kp = 1
                self.kd = 1
                self.ki = 1
                self.objetivoX = 0
                self.objetivoY = 0
                self.tamanhoMapaBusca = 0
                self.tamanhoAreaSeguranca = 0
                self.dtCriacao = datetime.now()