#!/usr/bin/env python
import mysql.connector
from entities.experimento import SessaoExperimento, SessaoExperimentoApontarParametros
##############################################################################
#   Laboratorio Remoto de Robotica Movel - TCC
#   Arquivo de conexao com o banco de dados
##############################################################################
#   Author: Paulo Felipe - paulof (at) ufop.edu.br
#
##############################################################################
#
#
class Database():

    def initDatabase(self):
        self.conn = mysql.connector.connect(
            host="localhost",
            user="root",
            password="praisekek18",
            database="labremoto")
        self.conn.autocommit = True

    def getSessaoAtiva(self):
        queryString = "SELECT * FROM sessao WHERE ativo = 1"

        cursor = self.conn.cursor()
        cursor.execute(queryString)
        results = cursor.fetchall()
        return results

    def removeSessaoAtiva(self):
        queryString = "UPDATE sessao SET ativo = 0 WHERE ativo = 1"
        cursor = self.conn.cursor()
        cursor.execute(queryString)
        self.conn.commit()
        queryString = "UPDATE sessao_experimento SET ativo = 0 WHERE ativo = 1"
        cursor = self.conn.cursor()
        cursor.execute(queryString)
        self.conn.commit()

    def getExperimentosSessaoAtiva(self, codSessaoAtiva):
        queryString = "SELECT codigo, cod_sessao, cod_experimento, dt_inicio, ativo FROM sessao_experimento WHERE cod_sessao = %s"
        cursor = self.conn.cursor()
        cursor.execute(queryString, (codSessaoAtiva, ))
        result = cursor.fetchone()
        if(result == None):
            return None
        sessaoExperimento = SessaoExperimento()
        sessaoExperimento.codigo = result[0]
        sessaoExperimento.codSessao = result[1]
        sessaoExperimento.codExperimento = result[2]
        sessaoExperimento.dtInicio = result[3]
        sessaoExperimento.ativo = result[4]
        return sessaoExperimento

    def getExperimentoAtivo(self, codSessaoAtiva):
        queryString = "SELECT codigo, cod_sessao, cod_experimento, dt_inicio, ativo FROM sessao_experimento WHERE cod_sessao = %s AND ativo = true"
        cursor = self.conn.cursor()
        cursor.execute(queryString, (codSessaoAtiva, ))
        result = cursor.fetchone()
        if(result == None):
            return None
        sessaoExperimento = SessaoExperimento()
        sessaoExperimento.codigo = result[0]
        sessaoExperimento.codSessao = result[1]
        sessaoExperimento.codExperimento = result[2]
        sessaoExperimento.dtInicio = result[3]
        sessaoExperimento.ativo = result[4]
        return sessaoExperimento
    
    def getParametrosExperimentoApontar(self, codExperimento):
        queryString = "SELECT cod_sessao_experimento, algoritmo_busca, obstaculos, kp, kd, ki, "
        queryString += "tamanho_mapa_busca, tamanho_area_seguranca, dt_criacao, objetivo_x, objetivo_y "
        queryString += "FROM experimento_apontar_parametros WHERE cod_sessao_experimento = %s"
        cursor = self.conn.cursor()
        cursor.execute(queryString, (codExperimento, ))
        result = cursor.fetchone()
        if(result == None):
            return None
        parametros = SessaoExperimentoApontarParametros()
        parametros.codSessaoExperimento = result[0]
        parametros.algoritmoBusca = result[1]
        parametros.obstaculos = result[2]
        parametros.kp = result[3]
        parametros.kd = result[4]
        parametros.ki = result[5]
        parametros.tamanhoMapaBusca = result[6]
        parametros.tamanhoAreaSeguranca = result[7]
        parametros.dtCriacao = result[8]
        parametros.objetivoX = result[9]
        parametros.objetivoY = result[10]
        return parametros
    
    def getRodarExperimentoStatus(self):
        queryString = "SELECT valor FROM configuracoes WHERE id = 2"
        cursor = self.conn.cursor()
        cursor.execute(queryString)
        results = cursor.fetchone()
        return int(results[0])

    def setRodarExperimentoStatus(self, status):
        queryString = "UPDATE configuracoes SET valor = '" + str(status) + "' WHERE id = 1"
        cursor = self.conn.cursor()
        cursor.execute(queryString)
        self.conn.commit()

    
