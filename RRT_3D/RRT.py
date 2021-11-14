import pygame
import pybullet as p
import pybullet_data
from RRTmain import RRTGraph
from RRTmain import RRTMAp
import time
import numpy as np
import math
import keyboard

def main():
    dimensions = (20,20,20)
    start = (-0.003, -0.63, 4.1850000000000005)
    goal = (-15, 15, 10)
    obsdim = 30
    iteration = 0
    obstacles = []
    obstacle = [[15.1,0, 15.9],[-15.1, 2.1, 10.9],[10.1, 20.1, 7.9],[10.1, -20.1, 7.9]]

    #Initialize world

    clid = p.connect(p.SHARED_MEMORY)
    if clid < 0:
        p.connect(p.GUI)
        # p.connect(p.SHARED_MEMORY_GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    # p.resetDebugVisualizerCamera(0.9, -100, -30, [0, 0, 0.4])
    p.resetDebugVisualizerCamera(80.0, -500, -400, [0, 0, 1.0])
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    model_path = '/assets/robots/kuka/iiwa14_robotiq85.urdf'
    obstacle_path = '/assets/objects/pybullet_data/cube.urdf'
    start_end_path = '/assets/objects/pybullet_data/sphere2.urdf'
    scale = 300
    scale_obs = 100
    kukaId = p.loadURDF(model_path, [0, 0, 0], globalScaling=scale*0.1)
    startId = p.loadURDF(start_end_path, [-0.003, -0.63, 5*(scale*0.1)])
    endId = p.loadURDF(start_end_path, [-15, 15, 10])
    if obstacle:
        for i in obstacle:
            obstacles.append(p.loadURDF(obstacle_path, i, useFixedBase=1, flags=2, globalScaling=scale_obs*0.1))
    
    p.loadURDF("random_urdfs/001/001.urdf", [0, 0.3, 0.3])
    p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])
    #p.resetBasePositionAndOrientation(obstacleId, obstacle, [0, 0, 0, 1])
    jointIndex = [1, 2, 3, 4, 5, 6, 7]
    kukaEndEffectorIndex = 8
    numJoints = len(jointIndex)

    # lower limits for null space
    ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
    # upper limits for null space
    ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
    # joint ranges for null space
    jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
    # restposes for null space
    rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
    # joint damping coefficents
    jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

    robot_constrains = [ll,ul,jr,rp,jd]

    p.setGravity(0, 0, 0)

    #Solve RRT Problem
    #map = RRTMAp(start,goal,dimensions,obstacles,kukaId,
    #                   kukaEndEffectorIndex,robot_constrains)
    graph = RRTGraph(start,goal,dimensions,obstacles,kukaId,
                       kukaEndEffectorIndex,robot_constrains)
    #Execute RRT simulation
    iteration  = 0 
    
    while(True and iteration<10000):
        x,y,z = graph.sample_envir()
        n = graph.number_of_nodes()
        graph.add_node(n,x,y,z)
        graph.add_edge(n-1,n)
        x1,y1,z1 = graph.x[n],graph.y[n],graph.z[n]
        x2,y2,z2 = graph.x[n-1],graph.y[n-1],graph.z[n-1]
        #p.loadURDF(start_end_path, [x,y,z])
        if (graph.isFree()):
            graph.setIK([x,y,z])
            #if not graph.crossObstacle(x1,x2,y1,y2,z1,z2):
            #    p.addUserDebugLine([x1,y1,z1], [x2,y2,z2],lineColorRGB=[0, 0, 1],lineWidth=2.0)
            
        p.stepSimulation()
        
        iteration+=1

    

    
        
    


    

if __name__=='__main__':
    main()
