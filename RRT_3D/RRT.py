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
    dimensions = (30,30,30)
    start = (-15, -15, 10)
    goal = (-15, 15, 10)
    obsdim = 30
    iteration = 0
    obstacle = [[15.1,0, 15.9],[-15.1, 2.1, 10.9],[10.1, 20.1, 7.9],[10.1, -20.1, 7.9]]

    #Initialize world

    clid = p.connect(p.SHARED_MEMORY)
    if clid < 0:
        p.connect(p.GUI)
        # p.connect(p.SHARED_MEMORY_GUI)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    # p.resetDebugVisualizerCamera(0.9, -100, -30, [0, 0, 0.4])
    p.resetDebugVisualizerCamera(40.0, -500, -400, [0, 0, 1.0])
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    model_path = '/assets/robots/kuka/iiwa14_robotiq85.urdf'
    obstacle_path = '/assets/objects/pybullet_data/cube.urdf'
    start_end_path = '/assets/objects/pybullet_data/sphere2.urdf'
    scale = 300
    scale_obs = 100
    kukaId = p.loadURDF(model_path, [0, 0, 0], globalScaling=scale*0.1)
    startId = p.loadURDF(start_end_path, [-15, -15, 10])
    endId = p.loadURDF(start_end_path, [-15, 15, 10])
    if obstacle:
        for i in obstacle:
            obstacleId = p.loadURDF(obstacle_path, i, useFixedBase=1, flags=2, globalScaling=scale_obs*0.1)
    
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

    p.setGravity(0, 0, 0)

    #Solve RRT Problem
    #map = RRTMAp(start,goal,dimensions,obsdim,len(obstacle))

    #Execute RRT simulation
    while(True):
        for i in range(numJoints):
            p.resetJointState(kukaId, jointIndex[i], rp[i])
        if keyboard.is_pressed('q'):
            p.disconnect()
            break

    

    
        
    


    

if __name__=='__main__':
    main()
    
