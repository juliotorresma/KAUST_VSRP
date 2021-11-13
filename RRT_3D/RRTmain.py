import random
import random
import math
import pygame

class RRTMAp:
    def __init__(self,start,goal,MapDimensions,obsdim,obsnum):
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.mapH, self.mapW, self.mapD = self.MapDimensions

        
        self.obstacles = []
        self.obsdim = obsdim
        self.obsNumber = obsnum

        #Colors
        self.gray = (70,70,70)
        self.Blue = (0,0,255)
        self.green = (0,255,0)
        self.Red = (255,0,0)
        self.white = (255,255,255) 

    def drawPath(self,path):
        #Draw coordinates in pybullet enviroment
        pass




class RRTGraph:
    def __init__(self,start,goal,MapDimensions,obsdim,obsnum):
        (x,y,z) = start 
        self.start = start 
        self.goal = goal
        #True if reach final coordinate
        self.goalFlag = False
        self.maph, self.mapw, self.mapd = MapDimensions

        self.x = []
        self.y = []
        self.z = []
        self.parent = []
        #initialize tree
        self.x.append(x)
        self.y.append(y)
        self.y.append(z)
        self.parent.append(0)

        #initialize tree
        self.x.append(x)
        self.y.append(y)
        self.z.append(z)
        self.parent.append(0)
        #obstacles
        self.obastacles = []
           
        self.obsNum = obsnum
        #path
        self.goalState = None
        self.path = []

    def makeobs(self,margenError = 20):
        pass

    def add_node(self,n,x,y,z):
        self.x.insert(n,x)
        self.y.append(y)
        self.z.append(z)


    def remove_node(self,n):
        self.x.pop(n)
        self.y.pop(n)
        self.z.pop(n)

    def add_edge(self,parent,child):
        self.parent.insert(child, parent)

    def remove_edge(self,n):
        self.parent.remove(n)
    def number_of_nodes(self):
        return len(self.x)
    def distance(self,n1,n2):
        (x1,y1,z1) = (self.x[n1],self.y[n1],self.z[n1])
        (x2,y2,z2) = (self.x[n2],self.y[n2],self.z[n2])

        px = (float(x1)-float(x2))**2
        py = (float(y1)-float(y2))**2
        pz = (float(z1)-float(z2))**2

        return (px+py+pz)**(0.5)
    def sample_envir(self):
        x = int(random.uniform(0,self.mapw))
        y = int(random.uniform(0,self.maph))
        z = int(random.uniform(0,self.mapd))

        return x,y,z
    def nearest(self,n):
        pass

    def isFree(self):
        n = self.number_of_nodes()-1
        (x,y,z) = (self.x[n], self.y[n],self.z[n])
        obs = self.obastacles.copy()
        while len(obs)>0:
            rectang = obs.pop(0)
            if rectang.collidepoint(x,y):
                self.remove_node(n)
                return False
        return True
    def crossObstacle(self,x1,x2,y1,y2):
        pass


    def connect(self,n1,n2):
        pass
    def step(self,nnear,nrand,dmax=5):
        pass
    def path_to_goal(self):
        pass

    def getPathCoords(self):
        pass
    #Expand to the goal direction
    def bias(self,ngoal):
        pass
    def expand(self):
        pass
    def cost(self):
        pass



     

