import pygame
import pybullet as p
import pybullet_data
from pygame.constants import SCALED
from RRTmain import RRTGraph
from RRTmain import RRTMAp
import time
import numpy as np
import math
import keyboard


def main():
    dimensions = (3,3,3)
    start = [-.3, -.5, .5]
    goal = [.6, .5, .5]
    obsdim = 30
    iteration = 0
    obstacles = []
    obstacle = [[.5,.5,1.3]]

    #Initialize world

    clid = p.connect(p.SHARED_MEMORY)
    if clid < 0:
        p.connect(p.GUI)
        # p.connect(p.SHARED_MEMORY_GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    # p.resetDebugVisualizerCamera(0.9, -100, -30, [0, 0, 0.4])
    p.resetDebugVisualizerCamera(2.0, -5, -4, [.5, 1, .5])
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")

    model_path = '../CS221/rethink/sawyer_description/urdf/sawyer_with_gripper.urdf'
    obstacle_path = '/assets/objects/pybullet_data/cube.urdf'

    kukaId = p.loadURDF(model_path, useFixedBase=1)
    jointLimits = {'lower':[],'upper':[]}
    for i in range(14):
        info = p.getJointInfo(kukaId,i)
        jointLimits['lower'].append(info[8])
        jointLimits['upper'].append(info[9])
    #Draw initial and end point
    p.addUserDebugText(".",goal,[1,0,0],5)
    p.addUserDebugText(".",start,[0,1,0],5)

    if obstacle:
        for i in obstacle:
            obs_i = p.loadURDF(obstacle_path, i, useFixedBase=1, flags=2)
            obstacles.append(obs_i)

    orn = p.getQuaternionFromEuler([0, -math.pi/4, 0])
    jointIndex = [1, 2, 3, 4, 5, 6, 7]
    kukaEndEffectorIndex = 9
   

    p.setGravity(0, 0, 0)

    jointPoses = p.calculateInverseKinematics(kukaId,kukaEndEffectorIndex,goal, orn,
        lowerLimits=jointLimits['lower'], upperLimits=jointLimits['upper'])
    for _ in range(len(jointPoses)):
        p.resetJointState(kukaId, _, jointPoses[_])
    
    
    #Solve RRT Problem
    
    graph = RRTGraph(start,goal,dimensions,obstacles,kukaId,
                       kukaEndEffectorIndex,jointLimits['lower'],jointLimits['upper'])
    #Execute RRT simulation
    iteration  = 0 
    
    while(not graph.path_to_goal()):
        n = graph.number_of_nodes()
        if iteration%2 == 0:
            X, Y, Z, Parent = graph.bias(goal)
            p.addUserDebugLine([X[-1],Y[-1],Z[-1]], [X[Parent[-1]],Y[Parent[-1]],Z[Parent[-1]]],lineColorRGB=[0, 0, 1],lineWidth=1.0)
            p.addUserDebugText(".",[X[-1],Y[-1],Z[-1]],[0,0,1],1)
            print([X[-1],Y[-1],Z[-1]])
        else:
            X, Y, Z, Parent = graph.expand()
            p.addUserDebugLine([X[-1],Y[-1],Z[-1]], [X[Parent[-1]],Y[Parent[-1]],Z[Parent[-1]]],lineColorRGB=[0, 0, 1],lineWidth=1.0)
            p.addUserDebugText(".",[X[-1],Y[-1],Z[-1]],[0,0,1],1)
        iteration+=1
        #p.stepSimulation()
    
    print("Coordenadas",graph.getPathCoords())
    path_2_follow = graph.getPathCoords().copy()
    for coords in path_2_follow:
        p.addUserDebugText(".",[coords[0],coords[1],coords[2]],[1,0,0],5)
    i=0
    #Aqui
    jointPoses = p.calculateInverseKinematics(kukaId,9,start, orn,
        lowerLimits=jointLimits['lower'], upperLimits=jointLimits['upper'])
    for _ in range(len(jointPoses)):
        p.resetJointState(kukaId, _, jointPoses[_])
    while len(path_2_follow) > 0:
        i += 1
        pos = path_2_follow.pop(0)
        jointPoses = p.calculateInverseKinematics(kukaId,9,pos, 
            lowerLimits=jointLimits['lower'], upperLimits=jointLimits['upper'])
        for _ in range(len(jointPoses)):
            p.resetJointState(kukaId, _, jointPoses[_])
        p.stepSimulation()
        time.sleep(0.15)
    
    jointPoses = p.calculateInverseKinematics(kukaId,9,start, orn,
        lowerLimits=jointLimits['lower'], upperLimits=jointLimits['upper'])
    for _ in range(len(jointPoses)):
            p.resetJointState(kukaId, _, jointPoses[_])
    
    p.disconnect()

if __name__=='__main__':
    main()
    
