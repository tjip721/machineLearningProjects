
import heapq
import queue
import math
import numpy as np
from robot import *

class environment:

    def __init__(self, length=10, width=10, delta = 1):
        self.length = length
        self.width = width
        self.delta = delta
        self.map = [['' for x in range(self.width)] for y in range(self.length)]
        self.features = []
        self.robots = None

    
    def addFeature(self, featureCoords):
        #Features should be list of 2d coords
        for coordPair in featureCoords:
            x = coordPair[0]
            y = coordPair[1]
            self.map[y][x] = 'x'
        return
    
    def addGoal(self, coordPair):
        x = coordPair[0]
        y = coordPair[1]
        self.goalX = x
        self.goalY = y
        self.map[y][x] = 'G'
        return

    def addRobot(self, rb):
        self.robot = rb
        self.map[rb.y][rb.x] = 'R'
        return   

    def moveRobot(self, coordPair, theta):
        x = coordPair[0]
        y = coordPair[1]
        if self.map[y][x] != 'x' and self.robot.legalMove(x,y,theta):
            self.map[self.robot.y][self.robot.x] = ' '
            self.robot.updateState(x, y, theta)
            self.map[y][x] = 'R'
            return True 
        else:
            return False

    def print(self, x=None):
        if not x:
            x = self.map
        for row in x:
            print(row)
        print('\n')

    def prettyPrint(self, x=None):
        if not x:
            x = self.map
        maxLen = 0
        for row in x:
            for col in row:
                maxLen = max(maxLen, len(str(col)))
        for row in x:
            printStr = ''
            rowDivide = ''
            for col in row:
                colStr = str(col)
                while len(colStr) < maxLen:
                    colStr += ' '
                printStr = printStr + '|' + colStr
            printStr += '|'
            print(printStr)
            for x in printStr:
                rowDivide += '_'
            print(rowDivide)

        print('\n')

    #implements a search using the A* heuristic method from robot position to the goal
    def planAStar(self, h):
        path = [[' ' for x in row] for row in self.map]
        x = self.robot.x
        y = self.robot.y
        moveCount = 0
        path[y][x] = moveCount
        heap = []
        searched = [[0 for x in row] for row in self.map]
        searched[y][x] = 1
        moves = [[1,0], [0,1], [-1,0], [0,-1]]
        searchCounter = 0
        while x != self.goalX or y != self.goalY:
            for move in moves:
                delx = move[0]
                dely = move[1]
                x1 = x+delx
                y1 = y+dely
                if x1 >= 0 and x1 < self.width and y1 >= 0 and y1 < self.length and self.map[y1][x1] != 'x' and searched[y1][x1] != 1:
                    #implement f = h + move cost
                    dist = math.sqrt(delx*delx + dely*dely)
                    f = dist + h[y1][x1]
                    heapq.heappush(heap, (f, searchCounter, x1, y1))
                    searchCounter += 1
                    searched[y1][x1] = 1
            if len(heap) <= 0:
                break
            else:
                newPosition = heapq.heappop(heap)
                x = newPosition[2]
                y = newPosition[3]
                moveCount += 1
                path[y][x] = (moveCount)
        return path
            

    def planDP(self, costFxn=[1,1,1,1], moves = [[1,0],[-1,0],[0,1],[0,-1]]):
        x = self.goalX
        y = self.goalY
        costGraph = [[float('inf') for x in row] for row in self.map]
#        moveGraph = [['' for x in row] for row in self.map]
        #start at goal, work outwards BFS, applying cost of moves to adjacent cells
        #continue as long as cells are updated
        costGraph[y][x] = 0
        updateQ= queue.Queue()
        updateQ.put([y,x])
        while not updateQ.empty():
            nextMove = updateQ.get()
            x = nextMove[1]
            y = nextMove[0]

            for ii, move in enumerate(moves):
                delx = move[1]
                dely = move[0]
                x1 = x + delx
                y1 = y + dely
                moveCost = costGraph[y][x] + costFxn[ii]
                if x1 >= 0 and x1 < self.width and y1 >= 0 and y1 < self.length and costGraph[y1][x1] > moveCost and self.map[y1][x1] != 'x':
                    updateQ.put([y1,x1])
                    costGraph[y1][x1] = costGraph[y][x] + costFxn[ii]
            

        return costGraph




    
world = environment()
world.print()
feature = [[1,ii] for ii in range(world.length-1)]
world.addFeature(feature)
world.addGoal([9,7])
robot = robot(0,0,0,world)
world.addRobot(robot)
world.print()
world.moveRobot([1,1],0)
world.moveRobot([0,1],0)
world.prettyPrint()
hueristic = []
world.prettyPrint(hueristic)
for y in range(world.length):
    hueristic.append([])
    for x in range(world.width):
        hueristic[y].append( math.sqrt((world.goalX-x)**2 + (world.goalY-y)**2) )
pathPlan = world.planAStar(hueristic)
world.prettyPrint(pathPlan)
dpPlan = world.planDP()
world.prettyPrint(dpPlan)