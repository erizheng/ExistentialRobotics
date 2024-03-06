import numpy as np
from math import *
import random
import time
from sympy import var, Sum, pi, limit, oo
import matplotlib.pyplot as plt
import scienceplots
plt.style.use(['science', 'notebook'])

############2D implementation

class Position_vector:
    def __init__(self, x, y, angle, time):
        self.x = x
        self.y = y
        self.angle = angle
        self.time = time

#v is linear velocity, and w is angular
class Velocity_vector:
    def __init__(self, v, w, time):
        self.v = v
        self.w = w
        self.time = time

class Landmark_vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y


#test landmarks
L1 = Landmark_vector(0, 0)
L2 = Landmark_vector(0, 0)
L3 = Landmark_vector(0, 0)
landmark = [L1, L2, L3]

#similar to prob_norm_dist
def prob_norm(a,b):
    return 1/(np.sqrt(2*np.pi *b)) * np.exp(-(a**2)/2*b)

def samp_norm_dist(b):
    temp = 0
    for i in range(1, 12):
        temp += random.randint(-1, 1)

    return b/6 * temp

#weigthed values for robot speci motion error
alpha1 = 1
alpha2 = 1
alpha3 = 1
alpha4 = 1
alpha5 = 1
alpha6 = 1

#based off page 97, the probability of x_t given control and x_prev
def prob_vel_model(x_t, u_t, x_prev):
    mew = (1/2)*(((x_prev.x - x_t.x)*np.cos(x_prev.angle) + (x_prev.y - x_t.y)*np.sin(x_prev.angle))/((x_prev.y - x_t.y)*np.cos(x_prev.angle) - (x_prev.x - x_t.x)*np.sin(x_prev.angle)))

    x_star = ((x_prev.x + x_t.x))/(2) + mew*(x_prev.y + x_t.y)
    y_star = ((x_prev.y + x_t.y))/(2) + mew*(x_prev.x + x_t.x)

    r_star = np.sqrt((x_prev.x - x_star)**2 + (x_prev.y - y_star)**2)

    change_delta = np.arctan2(x_t.y - y_star, x_t.x - x_star) - np.arctan2(x_prev.y - y_star, x_prev.x - x_star)
    change_time = x_t.time - x_prev.time

    v_hat = (change_delta/change_time)* r_star
    w_hat = (change_delta/change_time)
    gamma = (x_t.angle - x_prev.angle)/change_time - w_hat

    return prob_norm(u_t.v - v_hat, alpha1*np.abs(u_t.v) + alpha2*np.abs(u_t.w)) * prob_norm(u_t.w - w_hat, alpha3*np.abs(u_t.v) + alpha4*np.abs(u_t.w)) * prob_norm(gamma, alpha5*np.abs(u_t.v) + alpha6*np.abs(u_t.w))

#produces current position vector with previous and control
def motion_velocity_model(u_t, x_prev):
    v_hat = u_t.v + samp_norm_dist(alpha1*np.abs(u_t.v) + alpha2*np.abs(u_t.w))
    w_hat = u_t.w + samp_norm_dist(alpha3*np.abs(u_t.v) + alpha4*np.abs(u_t.w))
    gamma = samp_norm_dist(alpha5*np.abs(u_t.v) + alpha6*np.abs(u_t.w))

    time = time.time()

    x_next = x_prev.x - (v_hat/w_hat)*np.sin(x_prev.angle) + (v_hat/w_hat)*np.sin(x_prev.angle + w_hat*(time - x_prev.time))
    y_next = x_prev.y + (v_hat/w_hat)*np.cos(x_prev.angle) - (v_hat/w_hat)*np.cos(x_prev.angle + w_hat*(time - x_prev.time))

    angle_next = x_prev.angle + w_hat*(time - x_prev.time) + gamma*(time - x_prev.time)

    return Position_vector(x_next, y_next, angle_next, time)




#max senor range, 1 units?
z_max = 100
#variance
sig = 4 #normalized variance, max is 1
#assume correspondence is known
num_landmarks = 3

z_m = 0.2
z_hit = 0.7
z_short = 0.05
z_rand = 0.05
landmark = [0, 5, 20]

#z_m, measured distance from sensor
#z_expect, expected distance from particle position x_t
#k, index of landmark

def distance(x1, x2, y1, y2):
    d = np.sqrt((x2-x1)**2 + (y2-y1)**2)

    return d

#a, is x 
#b, is mean
#c, is variance, sig^2
def prob_norm_dist(a, b, c):
    return 1/(2* np.pi *c) * np.exp( - (a-b)**2 / (2 * c))

#guassian
def p_hit(z_measured, x_t, k):
    #x1, y1 will be landmarks
    z_expect = distance(landmark[k].x, x_t.x, landmark[k].y, x_t.y)
    N = (prob_norm_dist(z_measured, z_expect, sig**2))
    N2 = (prob_norm_dist(0, 0, sig**2))
    #n= normalize guassian
    if(z_measured >= 0 and z_measured <= z_max):
        return N/N2 #*n
    else:
        return 0
    
#random item in way
def p_short(z_measured, x_t, k):
    lamda = 1
    z_expect = distance(landmark[k].x, x_t.x, landmark[k].y, x_t.y)

    if(z_expect == 0 or z_measured == 0):
        return 1 #or 0 idk
    
    n = 1/(1- np.exp(-lamda * z_expect))
    
    if(z_measured >= 0 and z_measured <= z_expect):
        return n*lamda * np.exp(-lamda*z_measured)
    else:
        return 0
    
    
#if max sensor
#if at max then lower probability
def p_max(z):
    if(z == z_max):
        return 0
    else:
        return 1

#random noise
#just some random noises
def p_rand(z):
    if(z >= 0 and z< z_max):
        return (1/z)
    else:
        return 0
    

#landmarks, array location of landmarks
#z_t, relative location given by sensor
#x_t, pos of particles
#This calculates the probability of the range scans 

#will give the probability of the robot's location given sensor values
def measure_model(z_t, x_t, landmarks):
    q = 1
    #for each known location of landmark with index k
    for k in range(0, len(landmarks)):
        p = (z_hit * p_hit(z_t[k], x_t, k)) + (z_short * p_short(z_t[k], x_t, k)) 
        + (z_m * p_max(z_t[k])) + (z_rand * p_rand(z_t[k]))

        q= q*p

    return q


#combines the probability of the motion model with the measurement model
def weighted_prob(x_t, u_t, x_prev, z_t, landmarks):
    return prob_vel_model(x_t, u_t, x_prev) * measure_model(z_t, x_t, landmarks)

#X_prev, set of particle with x, y, angle, and time
#m, numbers of particles in set X_prev
#z_t, relative location given by sensors
def MonteCarlo2(Xprev, u_t, z_t, m):
    Xnot = Xt = []
    #w_t is weigthed value, a probability 
    #x_t is the position vector
    x_t = Position_vector(0,0,0,0)
    w_t = 0

    #Xprev, array of position X with size m
    #for each particle in set X_prev
    for i in range(0, m):
        #sample motion model
        #x_t, new cur location
        x_t = motion_velocity_model(u_t, Xprev[i])

        #measurement model
        #w_t, weight/probability of x_t
        w_t = weighted_prob(x_t, u_t, Xprev[i], z_t, i, landmark)

        Xnot = Xnot + [x_t, w_t] #add tuple

        #draw i with prob of w_t
        plt.scatter(x_t.x, x_t.y, w_t, color="black", alpha=0.5)

        #add x_t to X_t
        Xt.append(x_t)
        
    plt.ylim(top = 2)    
    plt.show()

    return Xt


