import Grafos
from time import time
from random import random

caminhoGrafo = "live_journal.txt"

'''grafoMatriz = Grafos.MatrizAdjacencia(caminhoGrafo)'''
memMatriz = float(input("Digite a quantidade de memoria utilizada (MB): "))
grafoLista = Grafos.ListaAdjacencia(caminhoGrafo)
memLista = float(input("Digite a quantidade de memoria utilizada (MB): ")) - memMatriz


def Tempo_BFS(Grafo,n = 5):
    numVert = Grafo.numeroVertices()
    delta = numVert/n
    vertTeste = [int(delta*i)-1 for i in range(1,n+1)]
    inicio = time()
    for v in vertTeste:
        Grafo.BFS(v)
    final = time()    
    return (final-inicio)
    
def Tempo_DFS(Grafo,n = 5):
    numVert = Grafo.numeroVertices()
    delta = numVert/n
    vertTeste = [int(delta*i)-1 for i in range(1,n+1)]
    inicio = time()
    for v in vertTeste:
        Grafo.DFS(v)
    final = time()    
    return (final-inicio)


def Pai_BFS(Grafo,v,o):
    """Retorna o pai[v] na arvore induzida pela BFS quando iniciada no
    vertice 'o'."""
    with open("BFS.txt","r") as arvGer:
        next(arvGer) # Pulando a primeira linha
        vertice = 1
        for linha in arvGer:
            if vertice == v:
                return int(linha.split()[0])
            vertice += 1

def Pai_DFS(Grafo,v,o):
    """Retorna o pai[v] na arvore induzida pela DFS quando iniciada no
    vertice 'o'."""
    with open("BFS.txt","r") as arvGer:
        next(arvGer)# Pulando a primeira linha
        vertice = 1
        for linha in arvGer:
            if vertice == v:
                return int(linha.split()[0])
            vertice += 1

def conexasEstatistica(Grafo):
    """Retorna o numero de componentes conexas, o tamanho da maior e da
    menor componente conexa do grafo."""
    Grafo.compConexas()
    with open("Componentes Conexas.txt","r") as componentes:
        numComponentes = int(componentes.readline())
        tamMaior = int(componentes.readline())
        tamMenor = tamMaior
        linhaValida = 1
        for linha in componentes:
            if (linhaValida%2) == 0:
                tamMenor = int(linha)
    return [numComponentes,tamMaior,tamMenor]        
    
def grauEstatistica(Grafo):
    estatistica = Grafo.estatistica()
    return estatistica[2:]
    
def diametro(Grafo):
    maiorDist = 0
    for i in range(Grafo.numeroVertices()):
        Grafo.BFS(i+1)
        with open("BFS.txt","r") as arvGer:
            profundidade = int(arvGer.readline())
            if profundidade > maiorDist:
                maiorDist = profundidade
    return maiorDist

# Estudos de Caso

#tempoBFSmat = Tempo_BFS(grafoMatriz)
tempoBFSmat = 0.0
tempoBFSlist = 0
print(1)

#tempoDFSmat = Tempo_DFS(grafoMatriz)
tempoDFSmat = 0.0
tempoDFSlist = 0
print(2)

# Linha: origem, Coluna: v, [Linha,Coluna]: pai[v]
pais_BFS = [[0 for i in range(5)] for j in range(5)]
print(3)
pais_DFS = [[0 for i in range(5)] for j in range(5)]
print(4)
origem = [1,2,3,4,5]
v = [10,20,30,40,50]
print(5)
for i in range(5):
    t1=time()
    grafoLista.BFS(i)
    tempoBFSlist+= (time()-t1)
    t1=time()
    grafoLista.DFS(i)
    tempoDFSlist+= time()-t1
    for j in range(5):
        pais_BFS[i][j] = Pai_BFS(grafoLista,v[j],origem[i])
        pais_DFS[i][j] = Pai_DFS(grafoLista,v[j],origem[i])
print(6)

infoCompCon = conexasEstatistica(grafoLista)
print(7)
infoGrau = grauEstatistica(grafoLista)
print(8)

diametro = 0
'''diametro(grafoLista)'''
        
with open("Estudos de Caso.txt","w") as estudo:
    estudo.write("Memoria:\n")
    estudo.write("Grafo Matriz: %.2f MB\nGrafo Lista: %.2f MB\n\n"%(memMatriz,memLista))
    estudo.write("Tempo de Execucao:\n")
    estudo.write("Busca em Largura: Matriz = %.3f s | Lista = %.3f s\n"\
                 %(tempoBFSmat,tempoBFSlist))
    estudo.write("Busca em Profundidade: Matriz = %.3f s | Lista = %.3f s\n\n"\
                 %(tempoDFSmat,tempoDFSlist))
    estudo.write(str(pais_BFS))
    estudo.write(str(pais_DFS))
    estudo.write("Componentes Conexas:\n")
    estudo.write("Numero: %d\nTamanho maximo: %d\nTamanho minimo: %d\n\n"%(infoCompCon[0],infoCompCon[1],infoCompCon[2]))
    estudo.write("Grau:\n")
    estudo.write("Grau minimo: %d\n Grau maximo: %d\nGrau medio: %.2f\nMediana: %d\n\n"%(infoGrau[0],infoGrau[1],infoGrau[2],infoGrau[3]))
    estudo.write("Diametro: %d"%(diametro))
