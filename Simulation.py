import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import math
import random

import MonteCarlo2
from MonteCarlo2 import*
from MonteCarlo2 import Position_vector
from MonteCarlo2 import Velocity_vector

#initial setup
p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used for loadURDF
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(0)

#environment setup

#ground setup
planeId = p.loadURDF("plane.urdf", [0,0,0], [0,0,0,1]) 
#p.createCollisionShape(p.GEOM_PLANE) #ground plane
#p.createMultiBody(0,0)

#walls
boxHalfLength = 10
boxHalfWidth = 0.1
boxHalfHeight = 2
#object specs
wall = p.createCollisionShape(p.GEOM_BOX,halfExtents=[boxHalfLength,boxHalfWidth,boxHalfHeight])
#mass, object, other thing(-1 for failure),position, orientation
massObstacles = 0
wallBottom = p.createMultiBody(massObstacles,wall, -1,[10,0,1])
wallTop = p.createMultiBody(massObstacles,wall, -1,[10,20,1])
wallLeft = p.createMultiBody(massObstacles,wall, -1,[0,10,1], [0,0,1,1])
wallRight = p.createMultiBody(massObstacles,wall, -1,[20,10,1], [0,0,1,1])

#some cube obstacles
#will change to create different size cubes upon generation
l = 1
w = 1
h = 1
cub = p.createCollisionShape(p.GEOM_BOX,halfExtents=[l, w, h])
cubePos1 = [5,5,1]
cube1 = p.createMultiBody(massObstacles, cub, -1, cubePos1)

#vector that has all the known correspondence, used as z_t
correspondence = [cubePos1]


#boxId = p.loadURDF("cube.urdf",cubeStartPos, cubeStartOrientation) #generated box

#generated car
carpos = [10, 10, 0.1]
car = p.loadURDF("racecar/racecar.urdf", carpos[0], carpos[1], carpos[2])


#generate particles
#generate within the "border" of x=20 and y=20 with a random angle from 0 to 360 at time 0
numParticle = 100 #number of particles
particles = []
for i in range(0,numParticle):
    genX = random.randint(0, 20)
    genY = random.randint(0, 20)
    genAngle = random.randint(0, 360)
    x_t = Position_vector(genX, genY, genAngle, 0)
    particles.append(x_t)





#gets joint info for car
# numJoints = p.getNumJoints(car)
# for joint in range(numJoints):
#     print(p.getJointInfo(car, joint))


#variables for control
targetVel = 10  #rad/s
maxForce = 100 #Newton
#camera vars
focus_position = [10,10,0] #where camera is focused upon, here it is the origin
cameraDistance = 20 #distance camera is from focus position
cameraYas = 0 #camera left and right rotation
cameraPitch = -89.9 #camera up and down rotation, set to -89.9 bc 90 makes it blank(for now)
p.setRealTimeSimulation(1)

#keep track of time
t = 0

while(1):
    #camera setup to top down view
    p.resetDebugVisualizerCamera(cameraDistance, cameraYas, cameraPitch, focus_position)
    #Keys to change camera
    keys = p.getKeyboardEvents()
    if (keys.get(122)):  #Z
        cameraYas+=1
    if keys.get(97):   #A
        cameraYas-=1
    if keys.get(99):   #C
        cameraPitch+=1
    if keys.get(102):  #F
        cameraPitch-=1
    if keys.get(100):  #D
        cameraDistance+=.5
    if keys.get(120):  #X
        cameraDistance-=.5
    p.stepSimulation()

    #as simulation runs, it gets the position and orientation of the car
    #Pos, Or = p.getBasePositionAndOrientation(car)
    #print(Pos)
    

    #controling the car using the arrow keys
    for k, v in keys.items():
        pos = list(p.getBasePositionAndOrientation(car)[0])
        orn = list(p.getEulerFromQuaternion(p.getBasePositionAndOrientation(car)[1]))

        #joints 2 and 3 used to move rear wheels
        if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in range(2, 4):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = targetVel,
                                        force = maxForce)
            p.setJointMotorControl2(car, 7, p.VELOCITY_CONTROL,
                                        targetVelocity = targetVel,
                                        force = maxForce)
            p.setJointMotorControl2(car, 5, p.VELOCITY_CONTROL,
                                        targetVelocity = targetVel,
                                        force = maxForce)
            p.stepSimulation()
            
        if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
            Vel = 0
            for joint in range(2, 8):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = Vel,
                                        force = maxForce)
            p.stepSimulation()
          
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in range(2, 4):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = -targetVel,
                                        force = maxForce)
            p.setJointMotorControl2(car, 7, p.VELOCITY_CONTROL,
                                        targetVelocity = -targetVel,
                                        force = maxForce)
            p.setJointMotorControl2(car, 5, p.VELOCITY_CONTROL,
                                        targetVelocity = -targetVel,
                                        force = maxForce)
            
            p.stepSimulation()

        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
            Vel = 0
            for joint in range(2, 8):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = Vel,
                                        force = maxForce)
            p.stepSimulation()
        

        #turning left and right
        #have issue with car not being able to back up and turn at the same time
        if(k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
            #steering using hinge
            p.setJointMotorControl2(car, 6, p.POSITION_CONTROL,
                                        targetPosition = 1,
                                        force = maxForce)
            p.setJointMotorControl2(car, 4, p.POSITION_CONTROL,
                                        targetPosition = 1,
                                        force = maxForce)
            
            p.stepSimulation()

        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_WAS_RELEASED)):
            Vel = 0
            for joint in range(2, 8):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = Vel,
                                        force = maxForce)
            p.setJointMotorControl2(car, 6, p.POSITION_CONTROL,
                                        targetPosition = 0,
                                        force = maxForce)
            p.setJointMotorControl2(car, 4, p.POSITION_CONTROL,
                                        targetPosition = 0,
                                        force = maxForce)
            p.stepSimulation()

        if(k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
            p.setJointMotorControl2(car, 6, p.POSITION_CONTROL,
                                        targetPosition = -1,
                                        force = maxForce)
            p.setJointMotorControl2(car, 4, p.POSITION_CONTROL,
                                        targetPosition = -1,
                                        force = maxForce)
            
            p.stepSimulation()

        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_WAS_RELEASED)):
            Vel = 0
            for joint in range(2, 8):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = Vel,
                                        force = maxForce)
            p.setJointMotorControl2(car, 6, p.POSITION_CONTROL,
                                        targetPosition = 0,
                                        force = maxForce)
            p.setJointMotorControl2(car, 4, p.POSITION_CONTROL,
                                        targetPosition = 0,
                                        force = maxForce)
            
            p.stepSimulation()
        
    #get control
    curVel = p.getBaseVelocity(car)
    #This returns a list of two vector3 values (3 floats in a list) 
    #representing the linear velocity [x,y,z] and angular velocity 
    #[wx,wy,wz] in Cartesian worldspace coordinates.
    linVel = np.sqrt(curVel[0][0]**2 + curVel[0][1]**2)
    angVel = curVel[1][2]
    control = Velocity_vector(linVel,angVel,t)
    
    print(control.w)

    #X_prev, set of particle with each with x, y, angle, and time
    #u_t, velocity vector that is controled by user for now
    #z_t, relative location given by sensors
    #m, numbers of particles in set X_prev
    #particles = MonteCarlo2(X_prev=particles, u_t= control, z_t=correspondence, m=numParticle)

    time.sleep(.01)
