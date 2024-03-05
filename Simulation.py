import numpy as np
import pybullet as p
import pybullet_data
import time

#initial setup
p.connect(p.GUI)
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(0)

#load assets
#planeId = p.loadURDF("plane.urdf", [0,0,0], [0,0,0,1]) 

p.createCollisionShape(p.GEOM_PLANE) #ground plane
p.createMultiBody(0,0)

#top
boxHalfLength = 10
boxHalfWidth = 0.1
boxHalfHeight = 2
#object specs
rec = p.createCollisionShape(p.GEOM_BOX,halfExtents=[boxHalfLength,boxHalfWidth,boxHalfHeight])
#mass, object, other thing(-1 for failure),position, orientation
wallBottom = p.createMultiBody(0,rec, -1,[10,0,1])
wallTop = p.createMultiBody(0,rec, -1,[10,20,1])
wallLeft = p.createMultiBody(0,rec, -1,[0,10,1], [0,0,1,1])
wallRight = p.createMultiBody(0,rec, -1,[20,10,1], [0,0,1,1])



#parameters of box
cubeStartPos = [0,0,0.5]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])

#boxId = p.loadURDF("cube.urdf",cubeStartPos, cubeStartOrientation) #generated box

#generated car
car = p.loadURDF("racecar/racecar_differential.urdf", [2,2,0.1], cubeStartOrientation)


while(1):
    focus_position = [10,10,0] #where camera is focused upon, here it is the origin
    cameraDistance = 20 #distance camera is from focus position
    cameraYas = 0 #camera left and right rotation
    cameraPitch = -89.99 #camera up and down rotation, set to -89.9 bc 90 makes it blank(for now)
    p.resetDebugVisualizerCamera(cameraDistance, cameraYas, cameraPitch, focus_position)
    p.stepSimulation()

    #as simulation runs, it gets the position and orientation of the car
    Pos, Or = p.getBasePositionAndOrientation(car)

    print(Pos)
    time.sleep(.01)
