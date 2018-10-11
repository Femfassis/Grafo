from Grafos import *
import random
import time

#G = ListaAdjacencia("grafo_1.txt")
#G = ListaAdjacencia("grafo_2.txt")
#G = ListaAdjacencia("grafo_3.txt")
#G = ListaAdjacencia("grafo_4.txt")
#G = ListaAdjacencia("grafo_5.txt")
#G = ListaAdjacencia("rede_colaboracao.txt")
Colaboracao = ListaAdjacencia("rede_colaboracao.txt")

#Excentricidade

vertices = [10,20,30,40,50]
for v in vertices:
    excent = G.excentricidade(v)
    print(excent)


#Dijkstra - Distancia
"""
vertices = [10,20,30,40,50]
dist = G.Dijkstra(1)
for i in vertices:
    print(dist[i-1])
"""

#Dijkstra - Caminho Minimo (De tras para frente)
"""
vertices = [10,20,30,40,50]
caminho = G.Dijkstra(1,1)
for vert in vertices:
    if caminho[vert-1] == -1:
        print("Nao ha caminho para o vertice %d!"%vert)
    else:
        while vert != -1:
            print(vert)
            vert = caminho[vert-1]
    print("\n")    
"""

#Arvore Geradora Minima, pesos e tempo de execucao
"""
t1=time.time()
Colaboracao.mst(1)
t=time.time()-t1
print(t)
"""

#Tempo medio para excentricidade
"""
vertices = [int(random.random()*G.numVert) for i in range(100)]
tempoTotal = 0
for v in vertices:
    t0 = time.time()
    G.excentricidade(v)
    t1 = time.time()
    tempoTotal += t1-t0
print(tempoTotal/100)
"""

#Distancia media do grafo e tempo para calcula-lo
"""
t0 = time.time()
distMedia = G.distanciaMedia()
t1 = time.time()
print("Distancia: %.2f, Tempo: %.3f"%(distMedia,t1-t0))
"""

#Distancia e caminho minimo entre Edsger W. Dijkstra e:
#Alan M. Turing, J. B. Kruskal, Jon M. Kleinberg, Eva Tardos,
#Daniel R. Figueiredo
"""
DijkstraVert = 2723
vertices = [11366,471366,5710,11387,343931]
dist = Colaboracao.Dijkstra(DijkstraVert,0)
caminho = Colaboracao.Dijkstra(DijkstraVert,1)

for vert in vertices:
    print("Distancia: %.3f\n\nCaminho:\n"%dist[vert-1])
    if caminho[vert-1] == -1:
        print("Nao ha caminho para o vertice %d!"%vert)
    else:
        while vert != -1:
            print(vert)
            vert = caminho[vert-1]
"""

#Tres maiores graus - Rede de colaboracao
"""
with open("MST.txt") as mst:
    a=mst.readline()
    graus=[0 for i in range(742385)]
    for i in mst:
           a=i.split()
           graus[int(a[0])]+=1
           graus[int(a[1])]+=1
    a1,b1,c1=0,0,0
    a2,b2,c2=None, None, None
    
    for i in range(len(graus)):
           if graus[i]>=a1:
               c1,c2=b1,b2
               b1,b2=a1,a2
               a1,a2=graus[i],i
           elif graus[i]>=b1:
               c1,c2=b1,b2
               b1,b2=graus[i],i
           elif graus[i]>=c1:
               c1,c2=graus[i],i
    print (a1,a2,b1,b2,c1,c2)
with open("rede_colaboracao_vertices.txt", encoding="utf8") as txt:
    resultado=[]
    for i in txt:
        a=i.split(',')
        if int(a[0])==a2: resultado+=[[a2,a[1],a1]]
        if int(a[0])==b2: resultado+=[[b2,a[1],b1]]
        if int(a[0])==c2: resultado+=[[c2,a[1],c1]]
    print (resultado)
"""

#Vizinhos de Dijkstra e Ratton
"""
with open("MST.txt") as mst:
    a=mst.readline()
    vizdij=[]
    vizdani=[]
    for i in mst:
        a=i.split()
        if a[0]=='2722': vizdij+=[a[1]]
        if a[1]=='2722': vizdij+=[a[0]]
        if a[0]=='343930': vizdani+=[a[1]]
        if a[1]=='343930': vizdani+=[a[0]]
    print(vizdij,vizdani)
"""
