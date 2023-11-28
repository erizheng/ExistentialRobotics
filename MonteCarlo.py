import numpy as np
from math import *
import random
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
z_max = 10
#variance
sig = 0.4
#assume correspondence is known
num_landmarks = 3

z_hit = 0.5
z_short = 0.1
z_rand = 0.1
landmark = [0, 1, 2]

#z_m, measured distance from sensor
#z_expect, expected distance from particle position x_t
#k, index of landmark

#guassian
def p_hit(z_measured, x_t, k):
    z_expect = abs(landmark[k] - x_t)
    N = (prob_norm_dist(z_measured, z_expect, sig**2))
    #n= samp_norm_dist(sig**2)
    if(z_measured >= 0 & z_measured <= z_max):
        return N
    else:
        return 0
    
#random item in way
def p_short(z_measured, x_t, k):
    lamda = 0.4
    z_expect = abs(landmark[k] - x_t)
    n = 1/(1- np.exp(-lamda * z_expect))
    
    if(z_measured >= 0 & z_measured <= z_expect):
        return n*lamda * np.exp(-lamda*z_measured)
    else:
        return 0
    
#if max sensor
def p_max(z):
    if(z == z_max):
        return 1
    else:
        return 0

#random noise
def p_rand(z):
    if(z >= 0 & z< z_max):
        return (1/z_max)
    else:
        return 0


#landmarks, array location of landmarks
#z_t, relative location given by sensor
#x_t, pos of particles
def obs_model(z_t, x_t, landmarks):
    #liklihood, range from 0 to 1?
    q = 1
    #for each known location of landmark with index k
    for k in range(0, len(landmarks)):
        p = (z_hit * p_hit(z_t[k], x_t, k)) + (z_short * p_short(z_t[k], x_t, k)) + (z_max * p_max(z_t[k])) + (z_rand * p_rand(z_t[k]))
        #p = p_hit(z_t[k], x_t, k) * p_short(z_t[k], x_t, k) * p_max(z_t[k])* p_rand(z_t[k])
        q= q*p

    return random.random()




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
        plt.bar(x_t, w_t, 5, color='black')

        #add x_t to X_t
        Xt.append(x_t)
        
    plt.ylim(top = 2)    
    plt.show()

    return Xt



#running

p = np.random.randint(0, 1000, size=100)
z = [1, 2, 10] #temp values, don't know what to do with z actually
MCL(p, 1, z, len(p))