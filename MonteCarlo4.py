import numpy as np
from math import *
import random
import time
from sympy import var, Sum, pi, limit, oo
import matplotlib.pyplot as plt
import scienceplots
plt.style.use(['science', 'notebook'])



def distance(x1, x2, y1, y2):
    d = np.sqrt((x2-x1)**2 + (y2-y1)**2)
    return d

  #a is mean, b is sigma^2
    def prob_norm(a,b):
        return 1/(np.sqrt(2*np.pi *b)) * np.exp(-(a**2)/2*b)
    


def calc_dist(position, landmarks, particles, measured_distace):
    #measured distance is distance from every landmark to robot position, should be sensor value
        #if can't get distance then it will be n/a
        #for now we will manually final distance from robot to landmarks
    #for every landmark find the distance from it to the particle
        #add noise to every distance with np.random
    
    #find the difference between the measured distance and the particle distance
        #this will act as the weight of each particle, so the smaller the value the more accurate 
    return dif

def calc_weight():
    #take the difference from calc_dist and create a normal distribution from it
    #multiply all the weights together of the landmarks(since they are in an list of list with the length of landmarks rn)
    #divide each particle weight, by the total sum, to normalize with landmarks
    #assign to each particle and divide by total agaian to normalize with particles
    return

def predict():#updating particles after each step
    #for every particle, move it according to input controls

    calc_weight()
    resample()
    return

def resample():
    return


move_step_x = 1 #amount x moves
move_step_y = 1
#Pos, [x, y] position
def move(Pos, move_step_x, move_step_y):#robot moving for each step
    Pos[0] +=  move_step_x
    Pos[1] +=  move_step_y



#X, [x, y],robot a nd y position
#W, [[x, y, w]], list of particles and their respective x and y position as well as the weight
#m, number of particles
#Map, [[x, y]], list of landmark positions
def MonteCarlo(X, W, m, Map):
    return
