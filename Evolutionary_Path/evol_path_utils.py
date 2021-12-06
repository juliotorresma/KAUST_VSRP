import random 
import numpy as np
import math
import pygame

class EvolMAp:
    def __init__(self,start,goal,MapDimensions,obsdim,obsnum):
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.mapH, self.mapW = self.MapDimensions

        #window settings
        self.MapWindowName = "RRT Path Planning"
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.mapW,self.mapH))
        self.map.fill((255,255,255))
        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1


        self.obstacles = []
        self.obsdim = obsdim
        self.obsNumber = obsnum

        #Colors
        self.gray = (70,70,70)
        self.Blue = (0,0,255)
        self.green = (0,255,0)
        self.Red = (255,0,0)
        self.white = (255,255,255)

    def drawMap(self,obstacles):
        pygame.draw.circle(self.map,self.green,self.start,self.nodeRad+5,0)
        pygame.draw.circle(self.map,self.Red,self.goal,self.nodeRad+20,0)
        self.drawObs(obstacles)
        

    def drawPath(self,path):
        for node in path:
            pygame.draw.circle(self.map,self.Red,node,self.nodeRad+3,0)

    def drawObs(self,obstacles):
        obstaclesList = obstacles.copy()
        while (len(obstaclesList)>0):
            obstacle = obstaclesList.pop(0)
            pygame.draw.rect(self.map,self.gray,obstacle)

class EvolGraph:
    def __init__(self,start,goal,MapDimensions,obsdim,obsnum):
        #True if reach final coordinate
        self.goalFlag = False
        self.maph, self.mapw = MapDimensions
        self.START = start
        self.FINISH = goal
        self.goalState = False
        #obstacles
        self.obastacles = []
        self.obsDim = obsdim    
        self.obsNum = obsnum
        #path
        self.goalState = None
        self.path = []

    def create_proposes(self,N,number=30,radius = 10):
        #Recibo un individuo y sacar sus coordenadas
        # 10 posibles futuros individuos utilizando trigo
        x,y = N[0]
        next_gen = []
        for i in range(number):
            next_ind = np.zeros(2)
            next_gen.append([next_ind,0])
        
        
        for i in range(number):

            free = False
            while(free == False):
                #Hacer un random de 0 a 360 grados para sacar las posibles opciones 
                random_angle = random.randint(0, 360)
                #Obtener las cordenadas de las siguientes 10 muestras
                y_pos = math.sin(random_angle) * radius + y
                x_pos = math.cos(random_angle) * radius + x
                obs = self.obastacles.copy()
                while len(obs)>0:
                    rectang = obs.pop(0)
                    if rectang.collidepoint(x_pos,y_pos):
                        free = False
                        break
                    else:
                        free = True
                next_gen[i][0][0] = x_pos
                next_gen[i][0][1] = y_pos
        return next_gen
    def makeRandomRect(self):
        #Generate random coordinates inside map boundieries
        uppercornerx = int(random.uniform(0,self.mapw-self.obsDim))
        uppercornery = int(random.uniform(0,self.maph-self.obsDim))

        return (uppercornerx,uppercornery)

    def makeobs(self,margenError = 20):
        obs = []
        
        for i in range(0,self.obsNum):
            #Temporary holds an rectangle object
            rectang = None
            startgoalcol = True
            while startgoalcol:
                upper = self.makeRandomRect()
                rectang = pygame.Rect(upper,(self.obsDim,self.obsDim))
                if rectang.collidepoint(self.START) or rectang.collidepoint(self.FINISH):
                    startgoalcol = True
                else:
                    startgoalcol = False
            obs.append(rectang)
        self.obastacles = obs.copy()
        return obs
    def path_to_goal(self,val):
        self.goalFlag = val

    def euclidean_distance(self,start_ind):
        [x1,y1] = start_ind
        [x2,y2] = self.FINISH
        px = (float(x1)-float(x2))**2
        py = (float(y1)-float(y2))**2

        return (px+py)**(0.5)

    def fitness(self,population):
        for i in range(len(population)):
            fitness_ind = self.euclidean_distance(population[i][0])
            population[i][1] = fitness_ind
        return population

    def survivor_selection(self,population):
        population.sort(key = lambda population: population[1])
        best_i = population[0:len(population)//2]

        return random.choice(best_i)

    



