import Grafos
from math import *
import numpy
import Queue

class Pontos2D():
    def __init__(self,Arquivo):
        """Dado o caminho do arquivo que contem o conjunto de pontos, constroi
        carrega em memoria"""
        with open(Arquivo) as arq:
            self.numPontos = int(arq.readline())
            self.pontos = [[0,0] for i in range(self.numPontos)]
            p = 0
            for linha in arq:
                x,y = linha.split()
                self.pontos[p][0] = float(x)
                self.pontos[p][1] = float(y)
                p += 1

    def distanciaEuclidiana(self,p1,p2):
        """Retorna a distancia euclidiana entre dois pontos de coordenadas (x0,y0) e (x1,y1)"""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return sqrt(dx**2+dy**2)

    def GrafoCompleto(self,Destino="GrafoCompleto.txt"):
        """Dado um caminho de destino, gera um arquivo texto de um Grafo Completo, onde os pontos
        sao vertices e as arestas representam a distancia euclidiana entre o par de vertices."""
        with open(Destino,"w") as grafo:
            grafo.write("%d\n"%(self.numPontos))
            for v in range(self.numPontos):
                for w in range(v+1,self.numPontos):
                    distancia = self.distanciaEuclidiana(self.pontos[v],self.pontos[w])
                    grafo.write("%d %d %f\n"%(v+1,w+1,distancia))
        return Grafos.MatrizAdjacencia(Destino)

    def TSP_Euclides(self,origem):
        grafo = self.GrafoCompleto()
        grafo.mst(origem)
        filhos=[ [] for i in range(grafo.numVert)]
        with open("MST.txt","r") as mst:
            mst.readline()
            for linha in mst:
                a=linha.split()
                filho=int(a[0])-1
                pai=int(a[1])-1
                filhos[pai]+=[filho]
        #IniciaPercorre:
        origem=origem-1
        sequencia=[0 for i in range(grafo.numVert)]
        p= Queue.LifoQueue()
        p.put(origem)
        posicao=0
        while p.empty() == False:
            v=p.get()
            sequencia[posicao]=v+1
            for filho in filhos[v]:
                p.put(filho)
            posicao+=1
        #Ver peso total:
        total=0
        for i in range(grafo.numVert-1):
            total+= grafo.matrizAdj[sequencia[i]-1][sequencia[i+1]-1]
        total+=grafo.matrizAdj[sequencia[i+1]-1][sequencia[0]-1]
        print(sequencia, total)
