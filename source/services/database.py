#!/usr/bin/env python
import mysql.connector

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

    def getExperimentosSessaoAtiva(self, codSessaoAtiva):
        queryString = "SELECT * FROM sessao_experimento WHERE cod_sessao = %s"
        cursor = self.conn.cursor()
        cursor.execute(queryString, (codSessaoAtiva, ))
        results = cursor.fetchall()
        print(results)
        return results
