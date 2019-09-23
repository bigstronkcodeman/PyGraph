# Graph Search Implementation
import math
class Graph:

    def __init__(self):
        self.adjDict = {}
        self.heuristics = {}
        self.numVertices = 0

    def addWeightedEdge(self, id1, id2, weight):
        if id1 not in self.adjDict:
            self.adjDict[id1] = {}
            self.numVertices += 1
        if id2 not in self.adjDict:
            self.adjDict[id2] = {}
            self.numVertices += 1
        self.adjDict[id1][id2] = weight
        self.adjDict[id2][id1] = weight

    def printPath(self, s, f, parent, nodesVisited):
        print("Nodes Visited: ", nodesVisited)
        path = [f]
        p = f
        while p in parent:
            path.insert(0, p)
            p = parent[p]
        path.insert(0, s)
        #print("Path taken: ", end='')
        #for i in range(len(path)):
        #    print(path[i], ' ', end='')
        #print()

    def insertSorted(self, pq, e, distances):
        i = 0
        while i in range(len(pq)):
            h1 = 0
            h2 = 0
            if pq[i] in self.heuristics:
                h1 = self.heuristics[pq[i]]
            if e in self.heuristics:
                h2 = self.heuristics[e]
            if distances[pq[i]] + h1 > distances[e] + h2:
                break
            i += 1
        pq.insert(i, e)

    def addHeuristic(self, id, h):
        if id in self.adjDict:
            self.heuristics[id] = h

    def bfs(self, s, f):
        visited = {}
        distance = {}
        parent = {}
        Q = []
        Q.append(s)
        distance[s] = 0
        nodesVisited = 0
        while len(Q) > 0 and Q[0] != f:
            u = Q.pop(0)
            visited[u] = True
            nodesVisited += 1
            for v in self.adjDict[u]:
                if v not in visited:
                    Q.append(v)
                    visited[v] = True
                    distance[v] = distance[u] + self.adjDict[u][v]
                    parent[v] = u
        nodesVisited += 1
        print("BFS:")
        self.printPath(s, f, parent, nodesVisited)
        print("Distance Travelled: ", distance[f], " kilometer(s)")

    def ucs(self, s, f):
        visited = {}
        distance = {}
        parent = {}
        pq = [s]
        nodesVisited = 0
        distance[s] = 0
        while len(pq) > 0 and pq[0] != f:
            u = pq.pop(0)
            visited[u] = True
            nodesVisited += 1
            for v in self.adjDict[u]:
                if v not in visited:
                    distance[v] = distance[u] + self.adjDict[u][v]
                    parent[v] = u
                    visited[v] = True
                    self.insertSorted(pq, v, distance)
                else:
                    if distance[v] > distance[u] + self.adjDict[u][v]:
                        distance[v] = distance[u] + self.adjDict[u][v]
                        parent[v] = u
                        if v in pq:
                            pq.remove(v)
                        self.insertSorted(pq, v, distance)
        nodesVisited += 1
        print("UCS:")
        self.printPath(s, f, parent, nodesVisited) 
        print("Distance Travelled: ", distance[f], " kilometer(s)")

    def astar(self, s, f):
        if len(self.heuristics) != self.numVertices:
            print("Not all vertices have been given heuristics")
        visited = {}
        distance = {}
        for i in self.adjDict:
            if i is not s:
                distance[i] = math.inf
            else:
                distance[i] = 0
        parent = {}
        pq = [s]
        nodesVisited = 0
        distance[s] = 0
        while len(pq) > 0 and pq[0] != f:
            u = pq.pop(0)
            visited[u] = True
            nodesVisited += 1
            for v in self.adjDict[u]:
                if v not in visited:
                    distance[v] = distance[u] + self.adjDict[u][v]
                    parent[v] = u
                    visited[v] = True
                    self.insertSorted(pq, v, distance)
                else:
                    if distance[v] > distance[u] + self.adjDict[u][v]:
                        distance[v] = distance[u] + self.adjDict[u][v]
                        parent[v] = u
                        if v in pq:
                            pq.remove(v)
                        self.insertSorted(pq, v, distance)
        nodesVisited += 1
        print("astar:")
        self.printPath(s, f, parent, nodesVisited) 
        print("Distance Travelled: ", distance[f], " kilometer(s)")

    def print(self):
        for d in self.adjDict:
            print(d, ' -> ', end='')
            i = 1
            for v in self.adjDict[d]:
                print(v, end='')
                if i != len(self.adjDict[d]):
                    print(', ', end='')
                i += 1
            print()

def tokenize(string, token):
    list = []
    substring = ""
    for letter in string:
        if letter != token:
            substring += letter
        else:
            list.append(substring)
            substring = ""
    if substring is not "":
        list.append(substring[:-1])
    return list

def readEdges(g):
    file = open("edges.txt", 'r')
    for line in file:
        strs = tokenize(line, ' ')
        g.addWeightedEdge(strs[0], strs[1], float(strs[2]))

def readHeuristic(g):
    file = open("heuristic.txt", 'r')
    for line in file:
        strs = tokenize(line, ' ')
        g.addHeuristic(strs[0], float(strs[1]))



g = Graph()
readEdges(g)
g.bfs('1436076226', '105012740')
g.ucs('1436076226', '105012740')
readHeuristic(g)
g.astar('1436076226', '105012740')
#g.addWeightedEdge('A', 'B', 12)
#g.addWeightedEdge('A', 'D', 5)
#g.addWeightedEdge('A', 'C', 2)
#g.addWeightedEdge('A', 'F', 10)
#g.addWeightedEdge('B', 'C', 3)
#g.addWeightedEdge('B', 'E', 3)
#g.addWeightedEdge('C', 'D', 8)
#g.addWeightedEdge('C', 'E', 0)
#g.addWeightedEdge('C', 'F', 1)
#g.addWeightedEdge('D', 'E', 2)
#g.addWeightedEdge('E', 'F', 2)
#g.addWeightedEdge('E', 'G', 2)
#g.addWeightedEdge('D', 'G', 1)
#g.addWeightedEdge('H', 'F', 2)
#g.addWeightedEdge('H', 'I', 0)
#g.addWeightedEdge('I', 'D', 1)
#g.addWeightedEdge('H', 'J', 2)
#g.addWeightedEdge('J', 'L', 10)
#g.addWeightedEdge('J', 'K', 1)
#g.addWeightedEdge('K', 'L', 1)
#g.addWeightedEdge('H', 'L', 11)
#g.addWeightedEdge('M', 'L', 1)
#g.addWeightedEdge('M', 'K', 3)
#g.addWeightedEdge('M', 'J', 2)

#g.addWeightedEdge('A', 'B', 2)
#g.addWeightedEdge('A', 'C', 7)
#g.addWeightedEdge('A', 'E', 33)
#g.addWeightedEdge('B', 'C', 3)
#g.addWeightedEdge('B', 'E', 5)
#g.addWeightedEdge('B', 'D', 4)
#g.addWeightedEdge('C', 'E', 1)
#g.addWeightedEdge('D', 'E', 2)
#g.addWeightedEdge('D', 'F', 2)
#g.addWeightedEdge('E', 'F', 1)

#g.addHeuristic('A', 0)
#g.addHeuristic('B', 1)
#g.addHeuristic('C', 2)
#g.addHeuristic('E', 4)
#g.addHeuristic('D', 3)
#g.addHeuristic('F', 5)

#g.addWeightedEdge('A', 'B', 1)
#g.addWeightedEdge('A', 'D', 3)
#g.addWeightedEdge('B', 'E', 6)
#g.addWeightedEdge('C', 'D', 1)
#g.addWeightedEdge('C', 'H', 6)
#g.addWeightedEdge('C', 'F', 2)
#g.addWeightedEdge('D', 'E', 2)
#g.addWeightedEdge('D', 'F', 8)
#g.addWeightedEdge('E', 'F', 3)
#g.addWeightedEdge('F', 'H', 1)
#g.addWeightedEdge('A', 'E', 5)
#g.addWeightedEdge('E', 'H', 1)

#g.addHeuristic('A', 13)
#g.addHeuristic('B', 2)
#g.addHeuristic('C', 9)
#g.addHeuristic('D', 7)
#g.addHeuristic('E', 4)
#g.addHeuristic('F', 3)
#g.addHeuristic('H', 0)

#g.addWeightedEdge('A', 'B', 2)
#g.addWeightedEdge('A', 'D', 2)
#g.addWeightedEdge('A', 'F', 10)
#g.addWeightedEdge('A', 'C', 0)
#g.addWeightedEdge('B', 'D', 1)
#g.addWeightedEdge('C', 'D', 1)
#g.addWeightedEdge('C', 'E', 0)
#g.addWeightedEdge('D', 'F', 1)
#g.addWeightedEdge('D', 'E', 0)

#g.addHeuristic('A', 0)
#g.addHeuristic('B', -5)
#g.addHeuristic('C', -50)
#g.addHeuristic('D', -500)
#g.addHeuristic('E', -5000)
#g.addHeuristic('F', 10)

#g.addWeightedEdge('A', 'C', 10)
#g.addWeightedEdge('A', 'B', 1)
#g.addWeightedEdge('B', 'C', 1)
#g.bfs('A', 'F')
#g.ucs('A', 'F')
#g.astar('A', 'F')
