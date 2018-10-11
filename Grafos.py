# -*- coding: cp1252 -*-
from collections import deque
import numpy
import heapq
import math

#Estrutura MinHeap com alteracoes de chaves
class MinHeap:
    def __init__(self,n):
        """Constroi a estrutura HeapMin com n elementos."""
        self.heap=[-1 for i in range(n)]
        self.pointer=[-1 for i in range(n)]
        self.apontado=[-1 for i in range(n)]    
        self.fim=-1

    def vazio(self):
        """Retorna True caso o Heap esteja vazio. Caso contrario, retorna False."""
        return self.fim==-1
    
    def tamanho(self):
        """Retorna o numero de elementos no Heap."""
        return self.fim+1
    
    def insere(self,elemento,chave):
        """Recebe a proxima posicao do Heap a ser preenchida e sua respectiva
        chave, inserindo-o na estrutura com complexidade log(n)."""
        self.fim+=1
        self.pointer[elemento]=self.fim
        self.heap[self.fim]=chave
        self.apontado[self.fim]=elemento
        pai=(self.fim-1)//2
        filho=self.fim
        while pai!= -1 and self.heap[pai]>self.heap[filho]:
            self.pointer[self.apontado[filho]],self.pointer[self.apontado[pai]]=self.pointer[self.apontado[pai]],self.pointer[self.apontado[filho]]
            self.apontado[filho],self.apontado[pai]=self.apontado[pai],self.apontado[filho]
            self.heap[pai],self.heap[filho]=self.heap[filho],self.heap[pai]
            filho=pai
            pai=(filho-1)//2

    def remove(self):
        """Remove o menor elemento do Heap com complexidade log(n)."""
        self.pointer[self.apontado[0]],self.pointer[self.apontado[self.fim]]=self.pointer[self.apontado[self.fim]],self.pointer[self.apontado[0]]
        self.apontado[0],self.apontado[self.fim]=self.apontado[self.fim],self.apontado[0]
        self.heap[self.fim],self.heap[0]=self.heap[0],self.heap[self.fim]
        resultado=[self.apontado[self.fim],self.heap[self.fim]]
        self.heap[self.fim]=-1
        self.pointer[self.apontado[self.fim]]=-1
        self.apontado[self.fim]=-1
        self.fim-=1
        pai=0
        try:
            if self.heap[1] == -1:
                return resultado
            elif self.heap[2] == -1:
                filho = 1
            else:
                if self.heap[1]<self.heap[2]:
                    filho = 1
                else:
                    filho = 2
        except:
            return resultado
                
        while filho<=self.fim and self.heap[pai]>self.heap[filho]:
            self.pointer[self.apontado[filho]],self.pointer[self.apontado[pai]]=self.pointer[self.apontado[pai]],self.pointer[self.apontado[filho]]
            self.apontado[filho],self.apontado[pai]=self.apontado[pai],self.apontado[filho]
            self.heap[pai],self.heap[filho]=self.heap[filho],self.heap[pai]
            pai=filho
            if (2*pai+1) > self.fim:
                return resultado
            elif (2*pai+2) > self.fim:
                filho = 2*pai+1
            else:
                if self.heap[2*pai+1]<self.heap[2*pai+2]:
                    filho = 2*pai+1
                else:
                    filho = 2*pai+2
        return resultado
        
    def altera(self,elemento,chave):
        """Dado o elemento, altera a chave na posicao respectiva do Heap em
        complexidade log(n)."""
        if self.pointer[elemento] == -1:
            print("Erro: Elemento nao esta no Heap!")
            return False
        self.heap[self.pointer[elemento]]=chave
        pai=(self.pointer[elemento]-1)//2
        filho=self.pointer[elemento]
        while pai!= -1 and self.heap[pai]>self.heap[filho]:
            self.pointer[self.apontado[filho]],self.pointer[self.apontado[pai]]=self.pointer[self.apontado[pai]],self.pointer[self.apontado[filho]]
            self.apontado[filho],self.apontado[pai]=self.apontado[pai],self.apontado[filho]
            self.heap[pai],self.heap[filho]=self.heap[filho],self.heap[pai]
            filho=pai
            pai=(filho-1)//2
        pai=self.pointer[elemento]
        try:
            if self.heap[2*pai+1]<self.heap[2*pai+2]: filho=2*pai+1
            else: filho = 2*pai+2
        except:
             filho=2*pai+1
        while filho<=self.fim and self.heap[pai]>self.heap[filho]:
            self.pointer[self.apontado[filho]],self.pointer[self.apontado[pai]]=self.pointer[self.apontado[pai]],self.pointer[self.apontado[filho]]
            self.apontado[filho],self.apontado[pai]=self.apontado[pai],self.apontado[filho]
            self.heap[pai],self.heap[filho]=self.heap[filho],self.heap[pai]
            pai=filho
            try:
                if self.heap[2*pai+1]<self.heap[2*pai+2]: filho=2*pai+1
                else: filho = 2*pai+2
            except:
                 filho=2*pai+1

    def valor(self, elemento):
        """Dado o elemento, retorna a chave no Heap caso exista."""
        try:
            return self.heap[self.pointer[elemento]]
        except:
            print("Erro de Indexacao!")
            return False

class MatrizAdjacencia:

    def __init__(self,caminho):
        """Dado o caminho do arquivo texto do grafo, constroi uma matriz de
        adjacencia."""
        with open(caminho,"r") as grafo:
            # Cria atributo "numero de vertices"
            self.numVert = int(grafo.readline())
            # Inicializando vetor grau de vertices
            self.grau = numpy.zeros(self.numVert,int)
            comeco=grafo.tell()
            linha1=grafo.readline().split()
            if len(linha1)==3: self.peso=1
            else: self.peso=0
            grafo.seek(comeco)
            if self.peso==0:
                # Inicializa uma matriz quadrada de lado igual ao numero de
                # vertices completa de zeros.
                self.matrizAdj = numpy.zeros((self.numVert,self.numVert))
                for linha in grafo:
                    aresta = linha.split()
                    u = min(int(aresta[0]),int(aresta[1]))
                    v = max(int(aresta[0]),int(aresta[1]))
                    # Se os vertices estao no formato correto, add a aresta
                    # na matriz e incrementa o grau de ambos os vertices
                    if u > 0 and v <= self.numVert:
                        self.matrizAdj[u-1][v-1] = 1
                        self.matrizAdj[v-1][u-1] = 1
                        self.grau[u-1]+=1
                        self.grau[v-1]+=1
            if self.peso==1:
                # Inicializa uma matriz quadrada de lado igual ao numero de
                # vertices completa de vazios.
                self.matrizAdj = numpy.empty((self.numVert,self.numVert))
                self.matrizAdj[:]=numpy.nan
                for linha in grafo:
                    aresta = linha.split()
                    u = min(int(aresta[0]),int(aresta[1]))
                    v = max(int(aresta[0]),int(aresta[1]))
                    # Se os vertices estao no formato correto, add a aresta
                    # na matriz e incrementa o grau de ambos os vertices
                    if u > 0 and v <= self.numVert:
                        self.matrizAdj[u-1][v-1] = float(aresta[2])
                        self.matrizAdj[v-1][u-1] = float(aresta[2])
                        self.grau[u-1]+=1
                        self.grau[v-1]+=1

    def AchagrauMin(self):
        """Retorna o vertice de menor grau do grafo. Complexidade O(n)."""
        return min(self.grau)

    def AchagrauMax(self):
        """Retorna o vertice de maior grau do grafo. Complexidade O(n)."""
        return max(self.grau)
    
    def AchanumAresta(self):
        """Retorna o numero de arestas do grafo. Complexidade O(n)."""
        soma=0
        for i in self.grau: i = soma + i
        return soma/2

    def AchagrauMedio(self):
        """Retorna o grau medio dos vertices. Complexidade O(n)."""
        soma=0
        for i in self.grau: i = soma + i
        return soma/self.numVert

    def AchagrauMediana(self):
        """Retorna a mediana dos graus dos vertices."""
        return numpy.median(self.grau)

    def numeroVertices(self):
        """Retorna o numero de vertices do grafo."""
        return self.numVert

    def estatistica(self):
        """Retorna o vetor [numero de vertices,nummero de arestas,grau minimo,grau maximo,
        grau medio,mediana dos graus]."""
        grauMin = self.grau[0]
        grauMax = self.grau[0]
        grauMediana = int(numpy.median(self.grau))
        grauSoma = 0
        for i in self.grau:
            if i < grauMin:
                grauMin = i
            if i > grauMax:
                grauMax = i
            grauSoma += i
        numArest =  grauSoma/2
        grauMedio = float(grauSoma)/self.numVert
        return [self.numVert,numArest,grauMin,grauMax,grauMedio,grauMediana]
    
    def BFS(self,origem,caminho = "BFS.txt"):
        """Recebe o vertice origem e o caminho para o diretorio onde
        sera criado o arquivo referente a arvore geradora induzida."""
        u = origem - 1
        #Inicializa um vetor de zeros, que armazena o nivel de cada vertice na arvore geradora
        nivel = numpy.array([-1 for i in range(self.numVert)], int)
        #Inicializa um vetor de zeros, que armazena o pai de cada vertice na arvore geradora
        pai = numpy.array([-1 for i in range(self.numVert)], int)
        #Inicializa um vetor de zeros, que indica 1 para os vertices descobertos e 0 para o contrario
        descobertos = numpy.zeros(self.numVert, int)
        #Inicializa fila da BFS
        fila = deque([u])
        descobertos[u] = 1
        nivel[u] = 0
        profundidade = 0
        while len(fila)>0:
            #Tira um vetor da pilha e em seguida, descobre todos os vertices adjacentes, colocando os
            #na pilha e atualizando seus niveis e pais
            v = fila.popleft()
            for w in range(self.numVert):
                if self.matrizAdj[v,w] == 1 and descobertos[w] == 0:
                    fila.append(w)
                    descobertos[w] = 1
                    pai[w] = v
                    nivel[w]= nivel[v]+1
                    if nivel[w] > profundidade:
                        profundidade = nivel[w]
        with open("BFS.txt","w") as arvGer:
            arvGer.write("%s\n"%(profundidade))
            for i in range(self.numVert):
                linha = "%s %s\n"%(str(pai[i]+1),str(nivel[i]))
                arvGer.writelines(linha)
                
    def DFS(self,origem,caminho = "DFS.txt"):
        """Recebe o vertice origem e o caminho para o diretorio onde
        sera criado o arquivo referente a arvore geradora induzida."""
        u = origem - 1
        #Inicializa um vetor de zeros, que armazena o nivel de cada vertice na arvore geradora
        nivel = numpy.array([-1 for i in range(self.numVert)], int)
        #Inicializa um vetor de zeros, que armazena o pai de cada vertice na arvore geradora
        pai = numpy.array([-1 for i in range(self.numVert)], int)
        #Inicializa um vetor de zeros, que armazena 1 se o vertice foi explorado e 0 caso contrario
        explorados = numpy.zeros((self.numVert), int)
        pilha = deque([u])
        nivel[u] = 0
        profundidade = 0
        while len(pilha)>0:
            #Tira um vetor da pilha e em seguida, descobre todos os vertices adjacentes, colocando
            #os na pilha e, se nao explorados, atualiza seus niveis e pais
            v = pilha.pop()
            if explorados[v] == 0:
                explorados[v] = 1
                for w in range(self.numVert):
                    if self.matrizAdj[v,w] == 1:
                        pilha.append(w)
                        if explorados[w] == 0:
                            pai[w] = v
                            nivel[w] = nivel[v]+1
                            if nivel[w] > profundidade:
                                profundidade = nivel[w]
        with open("DFS.txt","w") as arvGer:
            arvGer.write("%s\n"%(profundidade))
            for i in range(self.numVert):
                linha = "%s %s\n"%(str(pai[i]+1),str(nivel[i]))
                arvGer.writelines(linha)
        
    def compConexas(self,caminho = "Componentes Conexas.txt"):
        """Cria um arquivo de texto com as informacoes das componentes
        conexas no caminho especificado. Por padrao, o arquivo vai para
        a pasta onde se encontra o programa."""
        descobertos = [0 for i in range(self.numVert)]
        vertices = [i for i in range(self.numVert)] 
        desconhecido = 0
        numComp = 0
        componentes = []
        fila = deque()
        while desconhecido < self.numVert:
            inicioComp = desconhecido
            u = vertices[desconhecido]
            desconhecido += 1
            fila.append(u)
            descobertos[u] = 1
            while len(fila) > 0:
                v = fila.popleft()
                for w in range(self.numVert):
                    if self.matrizAdj[v,w] == 1 and descobertos[w] == 0:
                        fila.append(w)
                        temp = vertices[w]
                        vertices[w] = vertices[desconhecido]
                        vertices[desconhecido] = temp
                        descobertos[w] = 1
                        desconhecido += 1
            tamComp = desconhecido - inicioComp
            #Heap min para Heap max
            heapq.heappush(componentes,[-tamComp,inicioComp]) 
            numComp += 1    
        with open(caminho,"w") as compConexas:
            compConexas.write("%d\n"%(numComp))
            for i in range(numComp):
                maiorComp = heapq.heappop(componentes)
                compConexas.write("%d\n"%(-maiorComp[0]))
                for j in range(-maiorComp[0]):
                    compConexas.write("%d "%(vertices[maiorComp[1]+j]+1))
                compConexas.write("\n")

    def conexasEstatistica(self):
        """Retorna o numero de componentes conexas, o tamanho da maior e da
        menor componente conexa do grafo."""
        self.compConexas()
        with open("Componentes Conexas.txt","r") as componentes:
            numComponentes = int(componentes.readline())
            tamMaior = int(componentes.readline())
            tamMenor = tamMaior
            linhaValida = 1
            for linha in componentes:
                if (linhaValida%2) == 0:
                    tamMenor = int(linha)
        return [numComponentes,tamMaior,tamMenor]  

    def pai_BFS(self,v):
        """Retorna o pai[v] na arvore induzida pela BFS."""
        with open("BFS.txt","r") as arvGer:
            #Pulando a primeira linha
            next(arvGer) 
            vertice = 1
            for linha in arvGer:
                if vertice == v:
                    return int(linha.split()[0])
                vertice += 1

    def pai_DFS(self,v):
        """Retorna o pai[v] na arvore induzida pela DFS."""
        with open("DFS.txt","r") as arvGer:
            #Pulando a primeira linha
            next(arvGer) 
            vertice = 1
            for linha in arvGer:
                if vertice == v:
                    return int(linha.split()[0])
                vertice += 1

    def diametro(self):
        """Retorna a maior distancia do grafo. Infinito caso seja desconexo."""
        self.compConexas()
        with open("Componentes Conexas.txt","r") as componentes:
            if int(componentes.readline()) != 1:
                return numpy.inf
        maiorDist = 0
        for i in range(Grafo.numeroVertices()):
            print(i)
            Grafo.BFS(i+1)
            with open("BFS.txt","r") as arvGer:
                profundidade = int(arvGer.readline())
                if profundidade > maiorDist:
                    maiorDist = profundidade
        return maiorDist
    
    def Dijkstra(self,origem,dist_caminho = 0):
        """Dado um vertice origem, calcula distancias e caminhos minimos entre
        origem e todos os outros vertices. Caso o parametro dist_caminho seja
        0, retorna o vetor de distancias, caso contrário, se for 1 retorna o
        vetor de pais dos vertices."""
        # Criterios para Dijkstra - Grafo com pesos; pesos positivos
        if self.peso == 0:
            print("Erro: Grafo sem pesos!")
            return False
        for vertice in range(self.numVert):
            for vizinho in range(self.numVert):
                if self.matrizAdj[vertice][vizinho] != numpy.nan:
                    if self.matrizAdj[vertice][vizinho] < 0:
                        print("Erro: Pesos negativos!")
                        return False
        # Inicializando distancias
        distancia = [numpy.inf for i in range(self.numVert)]
        pai = [-1 for i in range(self.numVert)]
        dist = MinHeap(self.numVert)
        for i in range(self.numVert):
            dist.insere(i,numpy.inf)
        dist.altera(origem-1,0)
        # Loop principal: Pega vertice u com menor distancia d e atualiza distancias
        while not dist.vazio():
            u,d = dist.remove()
            distancia[u] = d
            # Olha cada vizinho v e o peso(u,v) w
            for v in range(self.numVert):
                try:
                    if self.matrizAdj[u][v] != numpy.nan:
                        if dist.valor(v) > distancia[u] + self.matrizAdj[u][v]:
                            dist.altera(v, distancia[u] + self.matrizAdj[u][v])
                            pai[v] = u+1
                except:
                    pass
        if dist_caminho == 0:
            return distancia
        else:
            return pai

    def excentricidade(self,origem):
        """Dado um vertice v, retorna a maior distancia entre o v
        e qualquer outro vertice do grafo."""    
        if self.peso == 0:
            # BFS
            u = origem - 1
            descobertos = numpy.zeros(self.numVert,int)
            descobertos[u] = 1
            fila = deque([u])
            nivel = numpy.array([-1 for i in range(self.numVert)], int)
            nivel[u] = 0
            maior = 0
            while len(fila) > 0:
                v = fila.popleft()
                for w in range(self.numVert):
                    if self.matrizAdj[v][w] == 1 and descobertos[w] ==0:
                        if descobertos[w] == 0:
                            fila.append(w)
                            descobertos[w] = 1
                            nivel[w] = nivel[v] + 1
                            maior = max(maior,nivel[w])
            for i in range(descobertos):
                if i == 0:
                    return numpy.inf
        else:
            # Dijkstra
            for vertice in range(self.numVert):
                for vizinho in range(self.numVert):
                    if self.matrizAdj[vertice][vizinho] != numpy.nan:
                        if self.matrizAdj[vertice][vizinho] < 0:
                            print("Erro: Pesos negativos!")
                            return False
            # Inicializando distancias
            maior = -numpy.inf
            distancia = [numpy.inf for i in range(self.numVert)]
            dist = MinHeap(self.numVert)
            for i in range(self.numVert):
                dist.insere(i,numpy.inf)
            dist.altera(origem-1,0)
            # Loop principal: Pega vertice u com menor distancia d e atualiza distancias
            while not dist.vazio():
                u,d = dist.remove()
                if d != numpy.inf:
                    maior = max(maior,d)
                else:
                    return numpy.inf
                distancia[u] = d
                # Olha cada vizinho v e o peso(u,v) w
                for v in range(self.numVert):
                    try:
                        if self.matrizAdj[u][v] != numpy.nan:
                            if dist.valor(v) > distancia[u] + self.matrizAdj[u][v]:
                                dist.altera(v, distancia[u] + self.matrizAdj[u][v])
                    except:
                        pass
        return maior

    def distanciaMedia(self):
        """Retorna a media das distancias entre todos pares (nao ordenados)
        de vertices do grafo"""
        if self.peso == 0:
            return 1
        else:
            somaDist = 0.0
            numPares = 0
            for u in range(self.numVert):
                distancias = self.Dijkstra(u+1)
                for v in range(u+1,self.numVert):
                    if distancias[v]!= numpy.inf:
                        somaDist += distancias[v]
                        numPares += 1
            return (somaDist/numPares) 

    def mst(self,s):
        """Dado um vertice de origem, gera dois arquivos de texto contendo a arvore
        geradora minima do grafo começando com origem e o peso da MST gerada."""
        s=s-1
        custo=numpy.array([math.inf for i in range(self.numVert)])
        custo[s]=0
        heap=MinHeap(self.numVert)
        pai=numpy.array([-1 for i in range(self.numVert)])
        for i in range(len(custo)):
            heap.insere(i,custo[i])
        while not heap.vazio():
            u=heap.remove()[0]
            for i in range(self.numVert):
                if self.matrizAdj[u,i] != numpy.nan and heap.pointer[i]!=-1:
                    if custo[i]>self.matrizAdj[u,i]:
                        pai[i]=u
                        custo[i]=self.matrizAdj[u,i]
                        heap.altera(i,self.matrizAdj[u,i])
        k=0
        pesototal=0
        for i in range(len(pai)):
            if pai[i]!=-1:
                k+=1
                pesototal+=self.matrizAdj[i,pai[i]]
        with open("PESOMST.txt","w") as arvGer: arvGer.writelines(str(pesototal))
        with open("MST.txt","w") as arvGer:
            arvGer.writelines("%s\n"%(k+1))
            for i in range(len(pai)):
                if pai[i]!=-1:
                    linha = "%s %s %s\n"%(str(i+1),str(pai[i]+1),str(self.matrizAdj[i,pai[i]]))
                    arvGer.writelines(linha)
                    
class ListaAdjacencia:

    def __init__(self,caminho = ""):
        """Dado o caminho do arquivo texto do grafo, constroi uma lista de
        adjacencia."""
        with open(caminho,"r") as grafo:
            #Cria atributo "numero de vertices"
            self.numVert = int(grafo.readline())
            #Inicializa um vetor de tamanho dos numeros de vertices, com uma
            #lista vazia em cada posicao
            self.listaAdj = [[] for i in range(self.numVert)]
            #Inicializa um vetor de zeros, que se trata de um atributo que
            #contabilizara o grau de cada vertice
            self.grau = [0 for i in range(self.numVert)]
            comeco=grafo.tell()
            linha1=grafo.readline().split()
            if len(linha1)==3: self.peso=1
            else: self.peso=0
            if self.peso==0:
                #Le linhas do arquivo e separa os dois vertices de cada linha do arquivo
                for linha in grafo:
                    aresta = linha.split()
                    u = min(int(aresta[0]),int(aresta[1]))
                    v = max(int(aresta[0]),int(aresta[1]))
                    #Se os vertices estao no formato corretos, adiciona as arestas ao
                    #vetor e incrementa o grau de ambos os vertices
                    if u > 0 and v <= self.numVert:
                        self.listaAdj[u-1] += [v-1]
                        self.listaAdj[v-1] += [u-1]
                        self.grau[u-1] += 1
                        self.grau[v-1] += 1
            if self.peso==1:
                #Le linhas do arquivo e separa os dois vertices de cada linha do arquivo
                for linha in grafo:
                    aresta = linha.split()
                    u = min(int(aresta[0]),int(aresta[1]))
                    v = max(int(aresta[0]),int(aresta[1]))
                    #Se os vertices estao no formato corretos, adiciona as arestas ao
                    #vetor e incrementa o grau de ambos os vertices
                    if u > 0 and v <= self.numVert:
                        self.listaAdj[u-1] += [[v-1,float(aresta[2])]]
                        self.listaAdj[v-1] += [[u-1,float(aresta[2])]]
                        self.grau[u-1] += 1
                        self.grau[v-1] += 1

    def numeroVertices(self):
        """Retorna o numero de vertices do grafo."""
        return self.numVert

    def AchagrauMin(self):
        """Retorna o vertice de menor grau do grafo. Complexidade O(n)."""
        return min(self.grau)

    def AchagrauMax(self):
        """Retorna o vertice de maior grau do grafo. Complexidade O(n)."""
        return max(self.grau)
    
    def AchanumAresta(self):
        """Retorna o numero de arestas do grafo. Complexidade O(n)."""
        soma=0
        for i in self.grau: i = soma + i
        return soma/2

    def AchagrauMedio(self):
        """Retorna o grau medio dos vertices. Complexidade O(n)."""
        soma=0
        for i in self.grau: i = soma + i
        return soma/self.numVert

    def AchagrauMediana(self):
        """Retorna a mediana dos graus dos vertices."""
        return numpy.median(self.grau)   

    def estatistica(self):
        """Retorna o vetor [numero de vertices,nummero de arestas,grau minimo,grau maximo,
        grau medio,mediana dos graus]."""
        grauMin = self.grau[0]
        grauMax = self.grau[0]
        grauMediana = int(numpy.median(self.grau))
        grauSoma = 0
        for i in self.grau:
            if i < grauMin:
                grauMin = i
            if i > grauMax:
                grauax = i
            grauSoma += i
        numArest =  grauSoma/2
        grauMedio = float(grauSoma)/self.numVert
        return [self.numVert,numArest,grauMin,grauMax,grauMedio,grauMediana]
        
    def BFS(self,origem,caminho = "BFS.txt"):
        """Recebe o vertice origem e o caminho para o diretorio onde
        sera criado o arquivo referente a arvore geradora induzida."""
        u = origem - 1
        #Inicializa um vetor de zeros, que armazena o nivel de cada vertice na arvore geradora
        nivel = numpy.array([-1 for i in range(self.numVert)], int)
        #Inicializa um vetor de zeros, que armazena o pai de cada vertice na arvore geradora
        pai = numpy.array([-1 for i in range(self.numVert)], int)
        #Inicializa um vetor de zeros, que indica 1 para os vertices descobertos e 0 para o contrario
        descobertos = numpy.zeros(self.numVert,int)
        descobertos[u] = 1
        fila = deque([u])
        nivel[u] = 0
        profundidade = 0
        while len(fila) > 0:
            #Tira um vetor da pilha e em seguida, descobre todos os vertices adjacentes, colocando
            #os na pilha e atualizando seus niveis e pais
            v = fila.popleft()
            for w in self.listaAdj[v]:
                if descobertos[w] == 0:
                    fila.append(w)
                    descobertos[w] = 1
                    pai[w] = v
                    nivel[w] = nivel[v] + 1
                    if nivel[w] > profundidade:
                        profundidade = nivel[w]
        with open("BFS.txt","w") as arvGer:
            arvGer.write("%s\n"%(profundidade))
            for i in range(self.numVert):
                linha = "%s %s\n"%(str(pai[i]+1),str(nivel[i]))
                arvGer.writelines(linha)

    def DFS(self,origem,caminho = "DFS.txt"):
        """Recebe o vertice origem e o caminho para o diretorio onde
        sera criado o arquivo referente a arvore geradora induzida."""
        u = origem - 1
        #Inicializa um vetor de zeros, que armazena o nivel de cada vertice na arvore geradora    
        nivel = numpy.array([-1 for i in range(self.numVert)], int)
        #Inicializa um vetor de zeros, que armazena o pai de cada vertice na arvore geradora
        pai = numpy.array([-1 for i in range(self.numVert)], int)
        #Inicializa um vetor de zeros, que armazena 1 se o vertice foi explorado e 0 caso contrario
        explorados = numpy.zeros((self.numVert), int)
        pilha = [u]
        nivel[u] = 0
        profundidade = 0
        while len(pilha) > 0:
            v = pilha.pop()
            if explorados[v] == 0:
                explorados[v] = 1
                for w in self.listaAdj[v]:
                    pilha.append(w)
                    if explorados[w] == 0:
                        pai[w] = v
                        nivel[w] = nivel[v] + 1
                        if nivel[w] > profundidade:
                            profundidade = nivel[w]
        with open("DFS.txt","w") as arvGer:
            arvGer.write("%s\n"%(profundidade))
            for i in range(self.numVert):
                linha = "%s %s\n"%(str(pai[i]+1),str(nivel[i]))
                arvGer.writelines(linha)

    def compConexas(self,caminho = "Componentes Conexas.txt"):
        """Cria um arquivo de texto com as informacoes das componentes
        conexas no caminho especificado. Por padrao, o arquivo vai para
        a pasta onde se encontra o programa."""
        if self.peso == 1:
            return False
        else:
            descobertos = [0 for i in range(self.numVert)]
            vertices = [i for i in range(self.numVert)] 
            desconhecido = 0
            numComp = 0
            componentes = []
            fila = deque()
            while desconhecido < self.numVert:
                inicioComp = desconhecido
                u = vertices[desconhecido]
                desconhecido += 1
                fila.append(u)
                descobertos[u] = 1
                while len(fila) > 0:
                    v = fila.popleft()
                    for w in self.listaAdj[v]:
                        if descobertos[w] == 0:
                            fila.append(w)
                            temp = vertices[w]
                            vertices[w] = vertices[desconhecido]
                            vertices[desconhecido] = temp
                            descobertos[w] = 1
                            desconhecido += 1
                tamComp = desconhecido - inicioComp
                # Heap min para Heap max
                heapq.heappush(componentes,[-tamComp,inicioComp]) 
                numComp += 1    
            with open(caminho,"w") as compConexas:
                compConexas.write("%d\n"%(numComp))
                for i in range(numComp):
                    maiorComp = heapq.heappop(componentes)
                    compConexas.write("%d\n"%(-maiorComp[0]))
                    for j in range(-maiorComp[0]):
                        compConexas.write("%d "%(vertices[maiorComp[1]+j]+1))
                    compConexas.write("\n")

    def conexasEstatistica(self):
        """Retorna o numero de componentes conexas, o tamanho da maior e da
        menor componente conexa do grafo."""
        self.compConexas()
        with open("Componentes Conexas.txt","r") as componentes:
            numComponentes = int(componentes.readline())
            tamMaior = int(componentes.readline())
            tamMenor = tamMaior
            linhaValida = 1
            for linha in componentes:
                if (linhaValida%2) == 0:
                    tamMenor = int(linha)
        return [numComponentes,tamMaior,tamMenor]
    
    def pai_BFS(self,v):
        """Retorna o pai[v] na arvore induzida pela BFS."""
        with open("BFS.txt","r") as arvGer:
            # Pulando a primeira linha
            next(arvGer) 
            vertice = 1
            for linha in arvGer:
                if vertice == v:
                    return int(linha.split()[0])
                vertice += 1

    def pai_DFS(self,v):
        """Retorna o pai[v] na arvore induzida pela DFS."""
        with open("DFS.txt","r") as arvGer:
            # Pulando a primeira linha
            next(arvGer) 
            vertice = 1
            for linha in arvGer:
                if vertice == v:
                    return int(linha.split()[0])
                vertice += 1

    def diametro(self):
        """Retorna a maior distancia do grafo. Infinito caso seja desconexo."""
        self.compConexas()
        with open("Componentes Conexas.txt","r") as componentes:
            if int(componentes.readline()) != 1:
                return numpy.inf
        maiorDist = 0
        for i in range(Grafo.numeroVertices()):
            print(i)
            Grafo.BFS(i+1)
            with open("BFS.txt","r") as arvGer:
                profundidade = int(arvGer.readline())
                if profundidade > maiorDist:
                    maiorDist = profundidade
        return maiorDist

    def mst(self,s):
        """Dado um vertice de origem, gera um arquivo texto contendo a arvore
        geradora minima do grafo começando com origem."""
        s=s-1
        custo=numpy.array([math.inf for i in range(self.numVert)])
        custo[s]=0
        heap=MinHeap(self.numVert)
        pai=[-1 for i in range(self.numVert)]
        for i in range(len(custo)):
            heap.insere(i,custo[i])
        while not heap.vazio():
            u=heap.remove()[0]
            for i in self.listaAdj[u]:
                if heap.pointer[i[0]]!=-1:
                    if custo[i[0]]>i[1]:
                        pai[i[0]]=[u,i[1]]
                        custo[i[0]]=i[1]
                        heap.altera(i[0],i[1])
        k=0
        pesototal=0
        for i in range(len(pai)):
            if pai[i]!=-1:
                k+=1
                pesototal+=pai[i][1]
        with open("PESOMST.txt","w") as arvGer: arvGer.writelines(str(pesototal))
        with open("MST.txt","w") as arvGer:
            arvGer.writelines("%s\n"%(k+1))
            for i in range(len(pai)):
                if pai[i]!=-1:
                    linha = "%s %s %s\n"%(str(i+1),str(pai[i][0]+1),str(pai[i][1]))
                    arvGer.writelines(linha)

    def Dijkstra(self,origem,dist_caminho = 0):
        """Dado um vertice origem, calcula distancias e caminhos minimos entre
        origem e todos os outros vertices. Caso o parametro dist_caminho seja
        0, retorna o vetor de distancias, caso contrário, se for 1 retorna o
        vetor de pais dos vertices."""
        # Criterios para Dijkstra - Grafo com pesos; pesos positivos
        if self.peso == 0:
            print("Erro: Grafo sem pesos!")
            return False
        for vertice in range(self.numVert):
            for vizinho,peso in self.listaAdj[vertice]:
                if peso < 0:
                    print("Erro: Pesos negativos!")
                    return False
        # Inicializando distancias
        distancia = [numpy.inf for i in range(self.numVert)]
        pai = [-1 for i in range(self.numVert)]
        dist = MinHeap(self.numVert)
        for i in range(self.numVert):
            dist.insere(i,numpy.inf)
        dist.altera(origem-1,0)
        # Loop principal: Pega vertice u com menor distancia d e atualiza distancias
        while not dist.vazio():
            u,d = dist.remove()
            distancia[u] = d
            # Olha cada vizinho v e o peso(u,v) w
            for v,w in self.listaAdj[u]:
                try:
                    if dist.valor(v) > distancia[u] + w:
                        dist.altera(v, distancia[u] + w)
                        pai[v] = u+1
                except:
                    pass
        if dist_caminho == 0:
            return distancia
        else:
            return pai

    def excentricidade(self,origem):
        """Dado um vertice v, retorna a maior distancia entre o v
        e qualquer outro vertice do grafo."""
        if self.peso == 0:
            # BFS
            u = origem - 1
            descobertos = numpy.zeros(self.numVert,int)
            descobertos[u] = 1
            fila = deque([u])
            nivel = numpy.array([-1 for i in range(self.numVert)], int)
            nivel[u] = 0
            maior = 0
            while len(fila) > 0:
                v = fila.popleft()
                for w in self.listaAdj[v]:
                    if descobertos[w] == 0:
                        fila.append(w)
                        descobertos[w] = 1
                        nivel[w] = nivel[v] + 1
                        maior = max(maior,nivel[w])
            for i in range(descobertos):
                if i == 0:
                    return numpy.inf
                            
        else:
            # Dijkstra
            for vertice in range(self.numVert):
                for vizinho,peso in self.listaAdj[vertice]:
                    if peso < 0:
                        print("Erro: Pesos negativos!")
                        return False
            # Inicializando distancias
            maior = -numpy.inf
            distancia = [numpy.inf for i in range(self.numVert)]
            dist = MinHeap(self.numVert)
            for i in range(self.numVert):
                dist.insere(i,numpy.inf)
            dist.altera(origem-1,0)
            # Loop principal: Pega vertice u com menor distancia d e atualiza distancias
            while not dist.vazio():
                u,d = dist.remove()
                if d != numpy.inf:
                    maior = max(maior,d)
                else:
                    return numpy.inf
                distancia[u] = d
                # Olha cada vizinho v e o peso(u,v) w
                for v,w in self.listaAdj[u]:
                    try:
                        if dist.valor(v) > distancia[u] + w:
                            dist.altera(v, distancia[u] + w)
                    except:
                        pass
        return maior

    def distancia(self,origem,destino):
        """Dado um vertice origem e um vertice destino, retorna a distancia entre
        eles. Caso o grafo seja sem peso, o algoritmo BFS define a distancia.
        Caso contrario, o algoritmo de Dijkstra define a distancia."""
        if self.peso == 0:
            u = origem - 1
            nivel = numpy.array([-1 for i in range(self.numVert)], int)
            descobertos = numpy.zeros(self.numVert,int)
            descobertos[u] = 1
            fila = deque([u])
            nivel[u] = 0
            while len(fila) > 0:
                v = fila.popleft()
                for w in self.listaAdj[v]:
                    if descobertos[w] == 0:
                        fila.append(w)
                        descobertos[w] = 1
                        nivel[w] = nivel[v] + 1
                    if w == destino-1:
                        return nivel[w]
        else:
            for vertice in range(self.numVert):
                for vizinho,peso in self.listaAdj[vertice]:
                    if peso < 0:
                        print("Erro: Pesos negativos!")
                        return False
            # Inicializando distancias
            distancia = [numpy.inf for i in range(self.numVert)]
            dist = MinHeap(self.numVert)
            for i in range(self.numVert):
                dist.insere(i,numpy.inf)
            dist.altera(origem-1,0)
            # Loop principal: Pega vertice u com menor distancia d e atualiza distancias
            while not dist.vazio():
                u,d = dist.remove()
                distancia[u] = d
                if u == destino-1:
                            return d
                # Olha cada vizinho v e o peso(u,v) w
                for v,w in self.listaAdj[u]:
                    try:
                        if dist.valor(v) > distancia[u] + w:
                            dist.altera(v, distancia[u] + w)
                    except:
                        pass

    def distanciaMedia(self):
        """Retorna a media das distancias entre todos pares (nao ordenados)
        de vertices do grafo"""
        if self.peso == 0:
            return 1
        else:
            somaDist = 0.0
            numPares = 0
            for u in range(self.numVert):
                distancias = self.Dijkstra(u+1)
                for v in range(u+1,self.numVert):
                    if distancias[v]!= numpy.inf:
                        somaDist += distancias[v]
                        numPares += 1
            return (somaDist/numPares)            
