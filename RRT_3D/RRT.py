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
    dimensions = (40,40,60)
    start = (-15, -15, 10)
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
    startId = p.loadURDF(start_end_path, [-15, -15, 10])
    endId = p.loadURDF(start_end_path, [-15, 15, 10])
    #p.removeCollisionShape(kukaId)
    if obstacle:
        for i in obstacle:
            obs_i = p.loadURDF(obstacle_path, i, useFixedBase=1, flags=2, globalScaling=scale_obs*0.1)
            #p.removeCollisionShape(obs_i)
            obstacles.append(obs_i)

    p.loadURDF("random_urdfs/001/001.urdf", [0, 0.3, 0.3])
    p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])
    pos = [-15, -15, 10]
        # end effector points down, not up (in case useOrientation==1)
    orn = p.getQuaternionFromEuler([0, -math.pi, 0])
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
    
    while(not graph.path_to_goal()):
        n = graph.number_of_nodes()
        if iteration%5 == 0:
            X, Y, Z, Parent = graph.bias(goal)
            p.addUserDebugLine([X[-1],Y[-1],Z[-1]], [X[Parent[-1]],Y[Parent[-1]],Z[Parent[-1]]],lineColorRGB=[0, 0, 1],lineWidth=3.0)
            p.addUserDebugText(".",[X[-1],Y[-1],Z[-1]],[0,0,1],5)
        else:
            X, Y, Z, Parent = graph.expand()
            p.addUserDebugLine([X[-1],Y[-1],Z[-1]], [X[Parent[-1]],Y[Parent[-1]],Z[Parent[-1]]],lineColorRGB=[0, 0, 1],lineWidth=3.0)
            p.addUserDebugText(".",[X[-1],Y[-1],Z[-1]],[0,0,1],5)
        #print("Entra")
        iteration+=1
        p.stepSimulation()
    
    print("Coordenadas",graph.getPathCoords())
    path_2_follow = graph.getPathCoords().copy()
    for coords in path_2_follow:
        p.addUserDebugText(".",[coords[0],coords[1],coords[2]],[1,0,0],5)
    i=0
    '''
    while len(path_2_follow) > 0:
        i += 1
        p.stepSimulation()
        pos = path_2_follow.pop(0)
        graph.setIK([pos[0],pos[1],pos[2]])

        p.stepSimulation()
    '''
    while(True):
        p.stepSimulation()
    
    #p.disconnect()
    

    

    
        
    


    

if __name__=='__main__':
    main()
    
