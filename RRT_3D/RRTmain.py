import random
import random
import math
import pygame
import pybullet as p

class RRTMAp:
    def __init__(self,start,goal,MapDimensions,obstacles,robot_id,endEffectorIndex,robot_constrains):
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.mapH, self.mapW, self.mapD = self.MapDimensions
        self.robot = robot_id
        self.robot_constrains = robot_constrains
        self.endEffectorIndex = endEffectorIndex
        self.obstacles = obstacles
        self.obsNumber = len(obstacles)

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
    def __init__(self,start,goal,MapDimensions,obstacles,robot_id,endEffectorIndex,robot_constrains):
        (x,y,z) = start 
        self.start = start 
        self.goal = goal
        self.endEffectorIndex = endEffectorIndex
        #True if reach final coordinate
        self.goalFlag = False
        self.maph, self.mapw, self.mapd = MapDimensions
        self.robot = robot_id
        self.robot_constrains = robot_constrains
        self.x = []
        self.y = []
        self.z = []
        self.parent = []
        #initialize tree
        self.x.append(x)
        self.y.append(y)
        self.z.append(z)
        self.parent.append(0)

        #initialize tree
        self.x.append(x)
        self.y.append(y)
        self.z.append(z)
        self.parent.append(0)
        #obstacles
        self.obastacles = obstacles
           
        self.obsNum = len(obstacles)
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
        x = int(random.uniform(-self.mapw,self.mapw))
        y = int(random.uniform(-self.maph,self.maph))
        z = int(random.uniform(0,self.mapd))

        return x,y,z
    def nearest(self,n):
        dmin = self.distance(0,n)
        nnear = 0
        for i in range(0,n):
            if self.distance(i,n)<dmin:
                dmin = self.distance(i,n)
                nnear = i
        return nnear

    def isFree(self):
        n = self.number_of_nodes()-1
        (x,y,z) = (self.x[n], self.y[n],self.z[n])
        obs = self.obastacles.copy()
        self.setIK([x,y,z])
        while len(obs)>0:
            obstacle_i = obs.pop(0)
            if len(p.getContactPoints(obstacle_i,self.robot)) > 0:
                self.remove_node(n)
                return False
        return True
    def crossObstacle(self,x1,x2,y1,y2,z1,z2):
        obs = self.obastacles.copy()
        while len(obs)>0:
            obstacle_i = obs.pop(0)
            for i in range(0,101):
                u = i/100
                x=x1*u + x2*(1-u)
                y=y1*u + y2*(1-u)
                z=z1*u + z2*(1-u)
                
                self.setIK([x,y,z])
                if len(p.getContactPoints(obstacle_i,self.robot)) > 0:
                    return True
        return False
    def setIK(self,position):
        
        jointPoses = p.calculateInverseKinematics(self.robot, self.endEffectorIndex,
                                                    position, 1, self.robot_constrains[0], self.robot_constrains[1], 
                                                        self.robot_constrains[2], self.robot_constrains[3])
        p.setJointMotorControlArray(bodyUniqueId=self.robot,
                                        jointIndices=[1, 2, 3, 4, 5, 6, 7],
                                        controlMode=p.POSITION_CONTROL,
                                        targetPositions=jointPoses[:7],
                                        targetVelocities=[0, 0, 0, 0, 0, 0, 0],
                                        forces=[500, 500, 500, 500, 500, 500, 500],
                                        positionGains=[0.03, 0.03, 0.03, 0.03, 0.03, 0.03, 0.03],
                                        velocityGains=[1, 1, 1, 1, 1, 1, 1])

    def connect(self,n1,n2):
        (x1,y1,z1) = (self.x[n1],self.y[n1],self.z[n1])
        (x2,y2,z2) = (self.x[n2],self.y[n2],self.z[n2])
        

        if self.crossObstacle(x1,x2,y1,y2,z1,z2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1,n2)
            return True
    def step(self,nnear,nrand,dmax=3):
        d = self.distance(nnear,nrand)
        if d>dmax:
            u = dmax/d
            (xnear,ynear,znear) = (self.x[nnear],self.y[nnear],self.z[nnear])
            (xrand,yrand,zrand) = (self.x[nrand],self.y[nrand],self.z[nrand])
            (px,py,pz) = (xrand-xnear,yrand-ynear,zrand-znear)
            theta = math.atan2(py,px)
            theta2 = math.atan2(pz,px)
            (x,y,z) = (int(xnear+dmax * math.cos(theta)),
                    int(ynear+dmax * math.sin(theta)),
                    int(znear+dmax * math.sin(theta2)))
            self.remove_node(nrand)
            if abs(x-self.goal[0])<dmax and abs(y - self.goal[1])<dmax and abs(z-self.goal[2])<dmax:
                self.add_node(nrand,self.goal[0], self.goal[1], self.goal[2])
                self.goalState = nrand
                self.goalFlag = True
            else:
                self.add_node(nrand,x,y,z)
    def path_to_goal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalState)
            newpos = self.parent[self.goalState]
            while(newpos!=0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.goalFlag

    def getPathCoords(self):
        pathCoords = []
        for node in self.path:
            x,y,z = (self.x[node],self.y[node],self.z[node])
            pathCoords.append((x,y,z))
        return pathCoords
    #Expand to the goal direction
    def bias(self,ngoal):
        n = self.number_of_nodes()
        self.add_node(n,ngoal[0],ngoal[1],ngoal[2])
        nnear = self.nearest(n)
        self.step(nnear,n)
        self.connect(nnear,n)
        return self.x,self.y,self.z,self.parent
    def expand(self):
        n = self.number_of_nodes()
        x,y,z = self.sample_envir()
        self.add_node(n,x,y,z)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest,n)
            self.connect(xnearest,n)
        return self.x,self.y,self.z,self.parent
    def cost(self):
        pass



     



     

