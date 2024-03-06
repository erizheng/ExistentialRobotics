import numpy as np
import pybullet as p
import pybullet_data
import time
import os
import math

#initial setup
p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(0)

#load assets
planeId = p.loadURDF("plane.urdf", [0,0,0], [0,0,0,1]) 
#p.createCollisionShape(p.GEOM_PLANE) #ground plane
#p.createMultiBody(0,0)

#top
boxHalfLength = 10
boxHalfWidth = 0.1
boxHalfHeight = 2
#object specs
rec = p.createCollisionShape(p.GEOM_BOX,halfExtents=[boxHalfLength,boxHalfWidth,boxHalfHeight])
#mass, object, other thing(-1 for failure),position, orientation
massObstacles = 0
# wallBottom = p.createMultiBody(massObstacles,rec, -1,[10,0,1])
# wallTop = p.createMultiBody(massObstacles,rec, -1,[10,20,1])
# wallLeft = p.createMultiBody(massObstacles,rec, -1,[0,10,1], [0,0,1,1])
# wallRight = p.createMultiBody(massObstacles,rec, -1,[20,10,1], [0,0,1,1])



#parameters of box
cubeStartPos = [0,0,0.5]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])

#boxId = p.loadURDF("cube.urdf",cubeStartPos, cubeStartOrientation) #generated box

#generated car
carpos = [0, 0, 0.1]
car = p.loadURDF("racecar/racecar.urdf", carpos[0], carpos[1], carpos[2])


numJoints = p.getNumJoints(car)
for joint in range(numJoints):
    print(p.getJointInfo(car, joint))

targetVel = 6  #rad/s
maxForce = 100 #Newton
p.setRealTimeSimulation(1)

while(1):
    focus_position = [10,10,0] #where camera is focused upon, here it is the origin
    cameraDistance = 20 #distance camera is from focus position
    cameraYas = 0 #camera left and right rotation
    cameraPitch = -89.99 #camera up and down rotation, set to -89.9 bc 90 makes it blank(for now)
    #p.resetDebugVisualizerCamera(cameraDistance, cameraYas, cameraPitch, focus_position)
    p.stepSimulation()

    #as simulation runs, it gets the position and orientation of the car
    #Pos, Or = p.getBasePositionAndOrientation(car)
    #print(Pos)

    #controling the car
    keys = p.getKeyboardEvents()
    for k, v in keys.items():
        pos = list(p.getBasePositionAndOrientation(car)[0])
        orn = list(p.getEulerFromQuaternion(p.getBasePositionAndOrientation(car)[1]))

        #joints 2 and 3 used to move rear wheels
        if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in range(2, 4):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = targetVel,
                                        force = maxForce)
            p.stepSimulation()
            
        if (k == p.B3G_UP_ARROW and (v & p.KEY_WAS_RELEASED)):
            Vel = 0
            for joint in range(2, 4):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = Vel,
                                        force = maxForce)
            p.stepSimulation()
          
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            for joint in range(2, 4):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = -targetVel,
                                        force = maxForce)
            p.stepSimulation()

        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_WAS_RELEASED)):
            Vel = 0
            for joint in range(2, 4):
                p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
                                        targetVelocity = Vel,
                                        force = maxForce)
            p.stepSimulation()
        

        #turning left and right
        #have issue with car not being able to back up and turn at the same time
        if(k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
            # Vel=8
            # for joint in [3,7]:
            #     p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
            #                             targetVelocity = Vel,
            #                             force = maxForce)
            # for joint in [2,5]:
            #     p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
            #                             targetVelocity = Vel/6,
            #                             force = maxForce)
            
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
            p.stepSimulation()

        if(k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
            # Vel=6
            # for joint in [3,7]:
            #     p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
            #                             targetVelocity = Vel/4,
            #                             force = maxForce)
            # for joint in [2,5]:
            #     p.setJointMotorControl2(car, joint, p.VELOCITY_CONTROL,
            #                             targetVelocity = Vel,
            #                             force = maxForce)
                
            #right hinge and left jinge
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
            p.stepSimulation()
        


    time.sleep(.01)
