from collections import deque
import numpy
import heapq

class MatrizAdjacencia:

    def __init__(self,caminho):
        """Dado o caminho do arquivo texto do grafo, constroi uma matriz de
        adjacencia."""
        with open(caminho,"r") as grafo:
            # Cria atributo "numero de vertices"
            self.numVert = int(grafo.readline())
            # Inicializa uma matriz quadrada de lado igual ao numero de
            # vertices completa de zeros.
            self.matrizAdj = numpy.zeros((self.numVert,self.numVert),"b")
            # Inicializando vetor grau de vertices
            self.grau = numpy.zeros(self.numVert,int)
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

    def AchagrauMin(self): #Acha o vertice de menor grau
        return min(self.grau)

    def AchagrauMax(self): #Acha o vertice de maior grau
        return max(self.grau)
    
    def AchanumAresta(self): #Acha o numero de arestas
        soma=0
        for i in self.grau: i = soma + i
        return soma/2

    def AchagrauMedio(self): #Acha o grau medio dos vertices
        soma=0
        for i in self.grau: i = soma + i
        return soma/self.numVert

    def AchagrauMediana(self): #Acha a mediana do grau dos vertices
        return numpy.median(self.grau)

    def numeroVertices(self):
        """Retorna o numero de vertices do grafo."""
        return self.numVert

    def estatistica(self):  #Retorna o grau mínimo, grau máximo, grau médio, mediana dos graus, número de vértices e númmero de arestas de forma compacta
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
        nivel = numpy.array([-1 for i in range(self.numVert)], int)#Inicializa um vetor de zeros, que armazena o nível de cada vértice na árvore geradora
        pai = numpy.array([-1 for i in range(self.numVert)], int)#Inicializa um vetor de zeros, que armazena o pai de cada vértice na árvore geradora
        descobertos = numpy.zeros(self.numVert, int)#Inicializa um vetor de zeros, que indica 1 para os vértices descobertos e 0 para o contrário
        fila = deque([u])#Inicializa fila da BFS
        descobertos[u] = 1
        nivel[u] = 0
        profundidade = 0
        while len(fila)>0:
            v = fila.popleft()#Tira um vetor da pilha e em seguida, descobre todos os vértices adjacentes, colocando os na pilha e atualizando seus níveis e pais
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
        nivel = numpy.array([-1 for i in range(self.numVert)], int)#Inicializa um vetor de zeros, que armazena o nível de cada vértice na árvore geradora
        pai = numpy.array([-1 for i in range(self.numVert)], int)#Inicializa um vetor de zeros, que armazena o pai de cada vértice na árvore geradora
        explorados = numpy.zeros((self.numVert), int)#Inicializa um vetor de zeros, que armazena 1 se o vértice foi explorado e 0 caso contrário
        pilha = deque([u])
        nivel[u] = 0
        profundidade = 0
        while len(pilha)>0:
            v = pilha.pop()#Tira um vetor da pilha e em seguida, descobre todos os vértices adjacentes, colocando os na pilha e, se não explorados, atualiza seus níveis e pais
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
            heapq.heappush(componentes,[-tamComp,inicioComp]) # Heap min para Heap max
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
            next(arvGer) # Pulando a primeira linha
            vertice = 1
            for linha in arvGer:
                if vertice == v:
                    return int(linha.split()[0])
                vertice += 1

    def pai_DFS(self,v):
        """Retorna o pai[v] na arvore induzida pela DFS."""
        with open("DFS.txt","r") as arvGer:
            next(arvGer) # Pulando a primeira linha
            vertice = 1
            for linha in arvGer:
                if vertice == v:
                    return int(linha.split()[0])
                vertice += 1

    def diametro(self):
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

class ListaAdjacencia:

    def __init__(self,caminho = ""):
        """Dado o caminho do arquivo texto do grafo, constroi uma lista de
        adjacencia."""
        with open(caminho,"r") as grafo:
            self.numVert = int(grafo.readline())#Cria atributo "número de vértices"
            self.listaAdj = [[] for i in range(self.numVert)]#Inicializa um vetor de tamanho dos números de vértices, com uma lista vazia em cada posição
            self.grau = [0 for i in range(self.numVert)]#Inicializa um vetor de zeros, que é um atributo que contabilizará o grau de cada vértice"
            for linha in grafo:#Lê linhas do arquivo e separa os dois vértices de cada linha do arquivo
                aresta = linha.split()
                u = min(int(aresta[0]),int(aresta[1]))
                v = max(int(aresta[0]),int(aresta[1]))
                if u > 0 and v <= self.numVert:#Se os vértices estão no formato corretos, adiciona as arestas ao vetor e incrementa o grau de ambos os vértices
                    self.listaAdj[u-1] += [v-1]
                    self.listaAdj[v-1] += [u-1]
                    self.grau[u-1] += 1
                    self.grau[v-1] += 1

    def numeroVertices(self):
        """Retorna o numero de vertices do grafo."""
        return self.numVert

    def AchagrauMin(self): #Acha o vertice de menor grau
        return min(self.grau)

    def AchagrauMax(self): #Acha o vertice de maior grau
        return max(self.grau)
    
    def AchanumAresta(self): #Acha o numero de arestas
        soma=0
        for i in self.grau: i = soma + i
        return soma/2

    def AchagrauMedio(self): #Acha o grau medio dos vertices
        soma=0
        for i in self.grau: i = soma + i
        return soma/self.numVert

    def AchagrauMediana(self): #Acha a mediana do grau dos vertices
        return numpy.median(self.grau)   

    def estatistica(self):#Retorna o grau mínimo, grau máximo, grau médio, mediana dos graus, número de vértices e númmero de arestas de forma compacta
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
        nivel = numpy.array([-1 for i in range(self.numVert)], int)#Inicializa um vetor de zeros, que armazena o nível de cada vértice na árvore geradora
        pai = numpy.array([-1 for i in range(self.numVert)], int)#Inicializa um vetor de zeros, que armazena o pai de cada vértice na árvore geradora
        descobertos = numpy.zeros(self.numVert,int)#Inicializa um vetor de zeros, que indica 1 para os vértices descobertos e 0 para o contrário
        descobertos[u] = 1
        fila = deque([u])
        nivel[u] = 0
        profundidade = 0
        while len(fila) > 0:
            v = fila.popleft()#Tira um vetor da pilha e em seguida, descobre todos os vértices adjacentes, colocando os na pilha e atualizando seus níveis e pais
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
        nivel = numpy.array([-1 for i in range(self.numVert)], int) #Inicializa um vetor de zeros, que armazena o nível de cada vértice na árvore geradora    
        pai = numpy.array([-1 for i in range(self.numVert)], int)#Inicializa um vetor de zeros, que armazena o pai de cada vértice na árvore geradora
        explorados = numpy.zeros((self.numVert), int)#Inicializa um vetor de zeros, que armazena 1 se o vértice foi explorado e 0 caso contrário
        pilha = [u]
        nivel[u] = 0
        profundidade = 0
        while len(pilha) > 0:
            v = pilha.pop()#Tira um vetor da pilha e em seguida, descobre todos os vértices adjacentes, colocando os na pilha e, se não explorados, atualiza seus níveis e pais
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
            heapq.heappush(componentes,[-tamComp,inicioComp]) # Heap min para Heap max
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
            next(arvGer) # Pulando a primeira linha
            vertice = 1
            for linha in arvGer:
                if vertice == v:
                    return int(linha.split()[0])
                vertice += 1

    def pai_DFS(self,v):
        """Retorna o pai[v] na arvore induzida pela DFS."""
        with open("DFS.txt","r") as arvGer:
            next(arvGer) # Pulando a primeira linha
            vertice = 1
            for linha in arvGer:
                if vertice == v:
                    return int(linha.split()[0])
                vertice += 1

    def diametro(self):
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
