import random
import math
import sys

class Node:
    def __init__(self, x, y, name):
        self.x = x
        self.y = y
        self.coord = (x,y)
        self.name = name
        self.neighbors = []
        self.pNode = None

class GridGraph:
    def __init__(self):
        self.cells = []

    def addGridNode(self, x, y, name):
        self.cells.append(Node(x, y, name))

    #adds undirected edge between first and second node
    def addUndirectedEdge(self, first, second):
        real = False
        oneX = first.x
        twoX = second.x
        oneY = first.y
        twoY = second.y
        if oneX + 1 is twoX and oneY is twoY or oneX- 1 is twoX and oneY is twoY:
            real = True
        if oneX is twoX and oneY + 1 is twoY or oneX is twoX and oneY - 1 is twoY:
            real = True

        if real:
            
            idx1 = -1
            idx2 = -1
            lstOfCells = self.cells
            for node in lstOfCells:
                if node.x is oneX and node.y is oneY:
                    idx1 = self.cells.index(node)
                if node.x is twoX and node.y is twoY:
                    idx2 = lself.cells.index(node)
            
            if idx1 != -1 and idx2  != -1:
                c1 = self.cells[idx2]
                c2 = self.cells[idx1]
                self.cells[idx1].neighbors.append(c1)
                self.cells[idx2].neighbors.append(c2)
        # self.cells = lstOfCells

    #removes undirected edge between first and second node
    def removeUndirectedEdge(self, first, second):
       
        idx1 = -1
        idx2 = -1
        oneX = first.x
        twoX = second.x
        oneY = first.y
        twoY = second.y
        
        lst = self.cells
        for node in lst:
            if node.x is oneX and node.y is oneY:
                idx1 = self.cells.index(node)
            if node.x is twoX and node.y is twoY:
                idx2 = self.cells.index(node)
        
        if idx1 != -1 and idx2 != -1:
            c1 = self.cells[idx2]
            c2 = self.cells[idx1]
            self.cells[idx1].neighbors.remove(c1)
            self.cells[idx2].neighbors.remove(c2)
        

    #returns all the cells that exist in the graph
    def getAllNodes(self):
        return self.cells
    
def minDist(lstOfDistances, checkVisited):
    answer = None
    m = sys.maxsize
    lst = lstOfDistances.keys()
    for node in lst:
        if node not in checkVisited and lstOfDistances[node] <= m:
            m = lstOfDistances[node]
            answer= node
    return answer

# creates random nodes and with random, unweighted bidirectional edges
def createRandomGridGraph(n):
    graph = GridGraph()
    count = 0
    for y in range(0,n):
        for x in range(0,n):
            graph.addGridNode(x, y, count)
            count += 1
    lst = graph.cells
    for item in lst:

        x = item.x
        y = item.y
        unconfirmedNeighbors = []

        potentialNeighbor1 = item.name-1
        size = n * n
        condition1 = bool(0 <= potentialNeighbor1 < size)
        if condition1:
            
            if y == lst[potentialNeighbor1].y:
                unconfirmedNeighbors.append(potentialNeighbor1)
        potentialNeighbor2 = item.name+1
        condition2 =bool( 0 <= potentialNeighbor2 < size)
        if condition2:
            if y == lst[potentialNeighbor2].y:
                unconfirmedNeighbors.append(potentialNeighbor2)
        potentialNeighbor3 = item.name+n
        condition3 = bool(0 <= potentialNeighbor3 < size)
        if condition3:
            if x == lst[potentialNeighbor3].x:
                unconfirmedNeighbors.append(potentialNeighbor3)
        potentialNeighbor4 = item.name-n
        condition4 = bool(0 <= potentialNeighbor3 < size)
        if condition4:
            if x == lst[potentialNeighbor4].x:
                unconfirmedNeighbors.append(potentialNeighbor4)
        
        for possNeighbor in unconfirmedNeighbors:
            aCell = lst[possNeighbor] 
            if aCell not in lst[possNeighbor].neighbors:
                
                randval = random.randint(0, 1)
                
                if randval > 0:
                    c1 = lst[possNeighbor]
                    item.neighbors.append(c1)
                    lst[possNeighbor].neighbors.append(item)
    graph.cells = lst
    return graph



def aStarHelper(tempNode, destination):
    # deltaX = abs(destination.x - tempNode.x)
    # deltaY = abs(destination.y - tempNode.y)
    return abs(destination.x - tempNode.x)+ abs(destination.y - tempNode.y)

def astar(sourceNode, destNode):
    lstOfDistances = {}
    checkVisited = set()
    lstOfDistances[sourceNode] = 0
    optimalPath = []
    mapOfDistances = {}
    tempNode = sourceNode

    while tempNode is not None:
        if tempNode is destNode:
            optimalPath.insert(0, tempNode)
            pNode = tempNode.pNode
            while pNode:
                optimalPath.insert(0, pNode)
                pNode = pNode.pNode
            break
        checkVisited.add(tempNode)
        for neighbor in tempNode.neighbors:
            cnd2 = lstOfDistances[tempNode] + 1 + aStarHelper(neighbor, destNode)
            if not(neighbor in lstOfDistances) or (lstOfDistances[neighbor] > cnd2):  
                lstOfDistances[neighbor] = cnd2
                neighbor.pNode = tempNode
        tempNode = minDist(lstOfDistances, checkVisited)

    return optimalPath


g = createRandomGridGraph(100)
lst = g.cells
for item in lst: print("X: ",item.x, " Y: ",item.y, " ->  Value:  ",item.name, sep = " ")
size = len(lst)
print("\t*****Running AStar on grid with ",size," cells*****")
path = astar(g.cells[0],g.cells[9999])
print("AStar path: ")
if len(path) is 0:
    print("Error, path was not found")
else:
    print("\t -> \t".join("[X:" + str(node.x) + " Y:" + str(node.y) + "]" for node in path))
#7 extra credit for astar
print("Number of Nodes finalized in A Star: ", len(path))