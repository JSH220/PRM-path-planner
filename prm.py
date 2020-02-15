from ImageProcessing import ImageProcessing
from AStar import AStar
import random
import cv2
import numpy as np
class PRM(object):
    def __init__(self): 
        #represent graph as Ajacency list
        # to get all neighbors, just call self.graph[index]
        self.graph = [None]
        self.vertex = [None]
        #Vertex Set, vertex as form (index, (x, y), grayscale), remember to get value of (x,y)
        #vertex[0] := index, vertex[1] := (x,y), vertex[2]: grayscale
        #need to do G[y][x], index from 0 to numberOfPoints, easy to get. 
        self.edge = [] #Edge Set, may not be neccessary 
        self.TwoDMatrix = None
        self.botRadius = 0
        self.imageName = ''
        self.obstValue = 130
        self.avoidBuffer = 3
    # isWayBlocked : ((int, int), (int, int)) -> bool
    # isWayBlocked, given two point in the Image, check if the line between 
    # the two point is blocked by obstacles  

    def outOfIndex(self, x,y):
        height = len(self.TwoDMatrix)
        width = len(self.TwoDMatrix[0])
        if x < 0 or x >= width: 
            return True
        if y < 0 or y >= height:
            return True
        return False  
    def onObstable(self, x,y):
        for cy in range (y-self.botRadius - self.avoidBuffer, y+self.botRadius + self.avoidBuffer):
            for cx in range (x - self.botRadius - self.avoidBuffer, x + self.botRadius + self.avoidBuffer):
                if (not self.outOfIndex(cx,cy)) and self.TwoDMatrix[cy][cx] < 50:
                    return True  
        return False      

    def isWayBlocked(self, p1, p2):
        (x1, y1) = p1
        (x2, y2) = p2

        if x1 == x2 and y1 == y2:
            return False
        elif x1 == x2:
            y_start = min(y1, y2)
            y_end = max(y1, y2)
            currentY = y_start
            while currentY < y_end:
                if self.onObstable(x1, currentY):
                    return True
                currentY += self.botRadius

        elif y1 == y2: 
            x_start = min(x1, x2)
            x_end = max(x1, x2)
            currentX = x_start
            while currentX < x_end:
                if self.onObstable(currentX, y1):
                    return True
                currentX += self.botRadius
        else:    
            slopeYX = (y2 - y1)/(x2 - x1)
            if x1 < x2:
                x_start = x1
                x_end = x2
                y_start = y1
            else:
                x_start = x2
                x_end = x1
                y_start = y2

            currentX = x_start
            while currentX < x_end:
                currentY = int(y_start + slopeYX * (currentX - x_start))
                if self.onObstable(currentX, currentY):
                    return True
                currentX += self.botRadius

            slopXY = (x2 - x1)/(y2 - y1)
            if y1 < y2:
                y_start = y1
                y_end = y2
                x_start = x1
            else:
                y_start = y2
                y_end = y1
                x_start = x2
            currentY = y_start
            while currentY < y_end:
                currentX = int(x_start + slopXY * (currentY - y_start))
                if self.onObstable(currentX, currentY):
                    return True
                currentY += self.botRadius
        return False           

    def testWayBlocked(self, imageName, p1, p2, botRadius):
        ImgProInstance = ImageProcessing()
        ImgMatrix = ImgProInstance.TranformJPGto2DArray(imageName)
        self.TwoDMatrix = ImgMatrix
        self.botRadius = botRadius
        self.imageName = imageName
        image = cv2.imread(imageName)
        cv2.line(image, p1, p2, (255,0,0), botRadius)


        blocked = False
        #bolcked = self.isWayBlocked(p1, p2)
        if self.isWayBlocked(p1, p2):
            print("blocked")
        else:
            print("not ")
        cv2.imshow('images',image)
        cv2.waitKey(0)
        return 0
    #initialize : (file, int, int) -> 0  (if no problem)
    # initialize, given a imageFilename, and how many points you want to generate
    # for the PRM, and the botRadius, initialize all the values of the instance,
    # especially, an ajacency list to represent a graph spreading whole image,
    # if there is no obstacle between two vertex, then there is an edge bewteen them 
    def initialize(self, imageName, numberOfPoints, botRadius):
        #extract value from image
        ImgProInstance = ImageProcessing()
        ImgMatrix = ImgProInstance.TranformJPGto2DArray(imageName)
        # for i in range(0, len(ImgMatrix)):
        #     for j in range(0, len(ImgMatrix[0])):
        #         if ImgMatrix[i][j] > 50 and ImgMatrix[i][j] < 200:
        #             print(ImgMatrix[i][j])

        self.imageName = imageName
        self.TwoDMatrix = ImgMatrix
        self.botRadius = botRadius
        height = len(ImgMatrix)
        width = len(ImgMatrix[0])
        print(height, width)
        i = 0
        while (i < numberOfPoints):
            x = random.randint(0, width-1)
            y = random.randint(0, height-1)
            #if on obstacle 
            if not self.onObstable(x,y):
                self.vertex.append((i+1, (x,y), ImgMatrix[y][x]))
                i = i+1       

        for i in range(1, numberOfPoints+1):
            neighbors = []
            for j in range(1, numberOfPoints+1):
                if i == j: 
                    pass        
                elif self.isWayBlocked(self.vertex[i][1], self.vertex[j][1]):
                    pass
                else:
                    (tmpIndex, (tempX, tempY), NotImportant) = self.vertex[j] 
                    if i < j:
                        #add to Edge set
                        neighbors.append((tmpIndex, (tempX, tempY)))
                        self.edge.append((self.vertex[i], self.vertex[j]))
                    else: 
                        #already in edge set, just add to neighbors 
                        neighbors.append((tmpIndex, (tempX, tempY)))
            
            self.graph.append(neighbors)   
        return 0 

    # findWay ((x,y), (x,y))
    # findWay, given a source coordinates, based on the Probabilistic Road Map,
    # using A* algorithm      
    def findWay(self, p1, p2):
        (x1,y1) = p1
        (x2,y2) = p2
        if self.onObstable(x1,y1): 
            print("sorry, you start at an obstable")
            return -1
        if self.onObstable(x2,y2):
            print("sorry, you want to go to an obstable")
            return -1
        self.vertex[0] = (0, (x1,y1),255)
        
        sourceNeighbors = []
        for i in range (1, len(self.vertex)):
            (tmpIndex, (tempX, tempY), NotImportant) = self.vertex[i]
            if self.isWayBlocked((tempX,tempY), (x1,y1)):
                pass
            else:
                sourceNeighbors.append((i, (tempX, tempY)))
            if self.isWayBlocked((tempX,tempY), (x2,y2)):
                pass
            else:    
                self.graph[i].append((len(self.vertex), (x2,y2)))

        self.vertex.append((len(self.vertex), (x2,y2), 255))
        self.graph[0] = sourceNeighbors        
                     
        AstarInstance = AStar()
        resultRoad = AstarInstance.run(self.graph, (0,(x1,y1)),(len(self.vertex)-1, (x2,y2)), "temp.txt")
        resultRoad.insert(0, (0, (x1,y1)))
        self.vertex[0] = None
        self.vertex.pop()
        # self.graph[0] = None

        return resultRoad

    def draw(self,result):
        test = cv2.imread(self.imageName)

        graph_node=[]
        for li in self.graph:
            graph_node += li
        graph_node = set(graph_node)

        for (index, (x,y)) in graph_node:
            cv2.circle(test, (x,y), 2, (255,0,0),-1)
        for edge_draw in self.edge:
            cv2.line(test, edge_draw[0][1], edge_draw[1][1], (255,0,0), 1)


        for (index, (x,y)) in result:
            cv2.circle(test, (x,y), 3, (0, 0, 255), -1)

        i = 0
        while i < len(result) - 1:
            cv2.line(test, result[i][1], result[i+1][1], (0,0,255), 2)
            i += 1

        cv2.imshow('images',test)
        cv2.waitKey(0)    

if __name__ == '__main__':
    image_name = 'test1.jpg'
    start_point=(35,35)
    end_point=(850,650)

    # test = cv2.imread('test.jpg')
    # cv2.circle(test, (200,650), 5, (0,0,255)) 
    # cv2.imshow('images',test)
    # cv2.waitKey()

    prm = PRM()
    # prm.testWayBlocked(image_name,(135,35), (120, 550),5)
    prm.initialize(image_name, 20, 2)
    result= prm.findWay(start_point,end_point)
    prm.draw(result)