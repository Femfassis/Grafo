# -*- coding: cp1252 -*-
from Pontos2D import Pontos2D
from time import time
from random import randint

#* Instanciando Conjunto de Pontos *#

""" Carregando arquivo """
#p = Pontos2D("points-5.txt")
#p = Pontos2D("points-10.txt")
#p = Pontos2D("points-20.txt")
#p = Pontos2D("points-50.txt")
#p = Pontos2D("points-100.txt")
#p = Pontos2D("points-200.txt")
#p = Pontos2D("points-500.txt")
#p = Pontos2D("points-1000.txt")
#p = Pontos2D("points-2000.txt")
#p = Pontos2D("points-5000.txt")
#p = Pontos2D("points-7500.txt")
#p = Pontos2D("points-10000.txt")


#* Obtendo percurso e distancias *#

# Método 1- TSP para um unico vertice origem
"""
t0 = time()
p.TSP_Euclides(1)
t1 = time()
"""

# Método 2 - Menor TSP dentre um conjunto de vertices origem
"""
t0 = time()
for i in range(5):
    p.TSP_Euclides(randint(1,p.numPontos))
t1 = time()
"""

print("Tempo: %f\n"%(t1-t0))

