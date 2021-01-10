#!/usr/bin/env python

##############################################################################
#   Laboratorio Remoto de Robotica Movel - TCC
#   Arquivo de gerenciamento de sessoes e experimentos
##############################################################################
#   Author: Paulo Felipe - paulof (at) ufop.edu.br
#
##############################################################################
#
#

from datetime import datetime, date, time
from entities.sessao import Sessao
class SessaoService:

    sessaoAtiva = None

	# Construtor
    def __init__(self, db):
        self.db = db

    # Verifica se a sessao esta no tempo valido
    def checkSessaoTimeout(self):
        if self.sessaoAtiva is not None:
            now = datetime.now()
            if now > self.sessaoAtiva.dtFim:
                self.db.removeSessaoAtiva()

    # Pega a sessao ativa do laboratorio
    def getSessaoAtiva(self):
        print("Carregando sessao ativa atual")
        sessoes = self.db.getSessaoAtiva()
        if len(sessoes) != 1:
            print("Nao foi encontrada sessao ativa no momento")
            return
        self.sessaoAtiva = Sessao(sessoes[0])
        print("Sessao ativa carregada")

