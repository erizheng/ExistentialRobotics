import numpy as np
from math import *
import random
import time
from sympy import var, Sum, pi, limit, oo
import matplotlib.pyplot as plt
import scienceplots
plt.style.use(['science', 'notebook'])


#a, is x 
#b, is mean
#c, is variance, sig^2
def prob_norm_dist(a, b, c):
    return 1/(2* np.pi *c) * np.exp( - (a-b)**2 / (2 * c))



#
def samp_norm_dist(b):
    temp = 0
    for i in range(1, 12):
        temp += random.randint(-1, 1)

    return b/6 * temp

def motion_model(u_t, x_prev):
    x_t = x_prev + u_t + samp_norm_dist(sig**2)
    return x_t


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

#guassian
def p_hit(z_measured, x_t, k):
    z_expect = abs(landmark[k] - x_t)
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
    z_expect = abs(landmark[k] - x_t)

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
def obs_model(z_t, x_t, landmarks):
    q = 1
    #for each known location of landmark with index k
    for k in range(0, len(landmarks)):
        p = (z_hit * p_hit(z_t[k], x_t, k)) + (z_short * p_short(z_t[k], x_t, k)) 
        + (z_m * p_max(z_t[k])) + (z_rand * p_rand(z_t[k]))

        q= q*p

    return q



#X_prev, set of particle
#m, numbers of particles in set X_prev
#z_t, relative location given by sensors
def MCL(Xprev, u_t, z_t, m):
    Xnot = Xt = []
    x_t = w_t = 0

    #Xprev, array of position X with size m
    #for each particle in set X_prev
    for i in range(0, m):
        #sample motion model
        #x_t, new cur location
        x_t = motion_model(u_t, Xprev[i])

        #measurement model
        #w_t, weight/probability of x_t
        w_t = obs_model(z_t, x_t, landmark)

        Xnot = Xnot + [x_t, w_t] #add tuple

        #draw i with prob of w_t
        plt.bar(x_t, w_t, 0.5, color='black')

        #add x_t to X_t
        Xt.append(x_t)
        
    plt.ylim(top = 2)    
    plt.show()

    return Xt

#running
p = np.random.randint(0, 100, size=1000)
z = [3, 2, 17] #temp values, don't know what to do with z actually
MCL(p, 0, z, len(p))


############2D implementation

class Position_vector:
    def __init__(self, x, y, angle, time):
        self.x = x
        self.y = y
        self.angle = angle
        self.time = time

class Velocity_vector:
    def __init__(self, v, w, time):
        self.v = v
        self.w = w
        self.time = time


#similar to prob_norm_dist
def prob_norm(a,b):
    return 1/(np.sqrt(2*np.pi *b)) * np.exp(-(a**2)/2*b)

#weigthed values for robot speci motion error
alpha1 = 1
alpha2 = 1
alpha3 = 1
alpha4 = 1
alpha5 = 1
alpha6 = 1

#based off page 97
def prob_vel_model(x_t, u_t, x_prev):
    mew = (1/2)(((x_prev.x - x_t.x)*np.cos(x_prev.angle) + (x_prev.y - x_t.y)*np.sin(x_prev.angle))/((x_prev.y - x_t.y)*np.cos(x_prev.angle) - (x_prev.x - x_t.x)*np.sin(x_prev.angle)))

    x_star = ((x_prev.x + x_t.x))/(2) + mew*(x_prev.y + x_t.y)
    y_star = ((x_prev.y + x_t.y))/(2) + mew*(x_prev.x + x_t.x)

    r_star = np.sqrt((x_prev.x - x_star)**2 + (x_prev.y - y_star)**2)

    change_delta = np.arctan2(x_t.y - y_star, x_t.x - x_star) - np.arctan2(x_prev.y - y_star, x_prev.x - x_star)
    change_time = x_t.time - x_prev.time

    v_hat = (change_delta/change_time)* r_star
    w_hat = (change_delta/change_time)
    gamma = (x_t.angle - x_prev.angle)/change_time - w_hat

    return prob_norm(u_t.v - v_hat, alpha1*np.abs(u_t.v) + alpha2*np.abs(u_t.w)) * prob_norm(u_t.w - w_hat, alpha3*np.abs(u_t.v) + alpha4*np.abs(u_t.w)) * prob_norm(gamma, alpha5*np.abs(u_t.v) + alpha6*np.abs(u_t.w))

def motion_velocity_control(u_t, x_prev):
    v_hat = u_t.v + samp_norm_dist(alpha1*np.abs(u_t.v) + alpha2*np.abs(u_t.w))
    w_hat = u_t.w + samp_norm_dist(alpha3*np.abs(u_t.v) + alpha4*np.abs(u_t.w))
    gamma = samp_norm_dist(alpha5*np.abs(u_t.v) + alpha6*np.abs(u_t.w))

    time = time.time()

    x_next = x_prev.x - (v_hat/w_hat)*np.sin(x_prev.angle) + (v_hat/w_hat)*np.sin(x_prev.angle + w_hat*(time - x_prev.time))
    y_next = x_prev.y + (v_hat/w_hat)*np.cos(x_prev.angle) - (v_hat/w_hat)*np.cos(x_prev.angle + w_hat*(time - x_prev.time))

    angle_next = x_prev.angle + w_hat*(time - x_prev.time) + gamma*(time - x_prev.time)

    return Position_vector(x_next, y_next, angle_next, time)

