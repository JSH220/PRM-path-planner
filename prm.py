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
        for cy in range (y-self.botRadius, y+self.botRadius+1):
            for cx in range (x-self.botRadius, x+self.botRadius+1):
                if self.outOfIndex(x,y):
                    pass
                elif self.TwoDMatrix[cy][cx] < 50:
                    return True
                else:
                    pass    
        return False       
    def isWayBlocked(self, p1, p2):
        (ix1, iy1) = p1
        (ix2, iy2) = p2
        x1 = min(float(ix1),float(ix2))
        x2 = max(float(ix1),float(ix2))
        y1 = min(float(iy1),float(iy2))
        y2 = max(float(iy1),float(iy2))
        if x1==x2 and y1==y2:
   
            return False
        elif x1 == x2:       

            for currentY in range(int(y1), int(y2)):
                for offset in range(-self.botRadius, self.botRadius+2):
                    if self.outOfIndex(x1+offset, currentY):
                        pass 
                    elif  self.TwoDMatrix[currentY][int(x1)+offset] < 50: 
                        return True  
                    else:
                        pass
        elif y1 == y2: 

            for currentX in range(int(x1), int(x2)):
                for offset in range(-self.botRadius, self.botRadius+2):
                    if self.outOfIndex(currentX, y1+offset): 
                        pass
                    elif  self.TwoDMatrix[int(y1)+offset][currentX] < 50: 
                        return True
                    else:
                        pass    
        else:    

            slopeYX = (y2 - y1)/(x2 - x1)
            for currentX in range(int(x1),int(x2)):
                currentY = int(y1 + slopeYX * (currentX - x1))
                for offset in range(-self.botRadius, self.botRadius+2):
                    if self.outOfIndex(currentX, currentY+offset): 
                        pass
                    elif  self.TwoDMatrix[currentY+offset][currentX] < 50: 
                        return True
                    else:
                        pass 
            slopeXY = (x2 - x1)/(y2 - y1)            
            for currentY in range(int(y1),int(y2)):
                currentX = int(x1 + slopeXY * (currentY - y1))
                for offset in range(-self.botRadius, self.botRadius+2):
                    if self.outOfIndex(currentX+offset, currentY):
                        pass 
                    elif  self.TwoDMatrix[currentY][currentX+offset] < 50: 
                        return True  
                    else:
                        pass         

        return False                 


    #initialize : (file, int, int) -> 0  (if no problem)
    # initialize, given a imageFilename, and how many points you want to generate
    # for the PRM, and the botRadius, initialize all the values of the instance,
    # especially, an ajacency list to represent a graph spreading whole image,
    # if there is no obstacle between two vertex, then there is an edge bewteen them 
    def initialize(self, imageName, numberOfPoints, botRadius):
        #extract value from image
        ImgProInstance = ImageProcessing()
        ImgMatrix = ImgProInstance.TranformJPGto2DArray(imageName)
        self.TwoDMatrix = ImgMatrix
        self.botRadius = botRadius
        height = len(ImgMatrix)
        width = len(ImgMatrix[0])
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
                if self.vertex[i][2] < 50 or self.vertex[j][2] < 50:  
                    pass
                elif i == j: 
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
        test = cv2.imread('test.jpg')
        for (index, (x,y)) in result:
            cv2.circle(test, (x,y), 10, (0,0,255))
        
        graph_node=[]
        for li in self.graph:
            graph_node += li
        graph_node = set(graph_node)

        for (index, (x,y)) in graph_node:
            cv2.circle(test, (x,y), 5, (0,255,0))

        cv2.imshow('images',test)
        cv2.waitKey(0)    

if __name__ == '__main__':
    image_name = 'test.jpg'
    start_point=(35,35)
    end_point=(200,650)

    # test = cv2.imread('test.jpg')
    # cv2.circle(test, (200,650), 5, (0,0,255)) 
    # cv2.imshow('images',test)
    # cv2.waitKey()

    prm = PRM()
    prm.initialize(image_name, 20, 2)
    result= prm.findWay(start_point,end_point)
    prm.draw(result)