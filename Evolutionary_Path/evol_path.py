from evol_path_utils import *
import pygame
import time
dimensions = (600,1000)
start = (50,50)
goal = (510,510)
obsdim = 40
obsnum = 150
iteration = 0

pygame.init()
map = EvolMAp(start,goal,dimensions,obsdim,obsnum)
graph = EvolGraph(start,goal,dimensions,obsdim,obsnum)

obstacles = graph.makeobs()
map.drawMap(obstacles)

dimensions = (600,1000)

ind = [start,0]        
gens = graph.create_proposes(ind)
gens = graph.fitness(gens)
elite = graph.survivor_selection(gens)



while(graph.goalFlag == False and iteration < 300):
    gens = graph.create_proposes(elite)
    gens = graph.fitness(gens)
    elite = graph.survivor_selection(gens)

    X,Y = elite[0][0],elite[0][1]

    if graph.euclidean_distance([X,Y]) < 10:
        graph.path_to_goal(True)
        break
    time.sleep(0.05)

    pygame.draw.circle(map.map,map.Red,(X,Y),map.nodeRad+2,0)
    print(elite)
    pygame.display.update()
    iteration+=1


pygame.display.update()
pygame.event.clear() 
pygame.event.wait(0)



