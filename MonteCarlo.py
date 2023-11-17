import numpy as np
from math import *
import random
import matplotlib.pyplot as plt
import scienceplots
plt.style.use(['science', 'notebook'])


def prob_norm_dist(a, b):
    return 1/(2* np.pi *b) * np.exp( - (a)**2 / (2 * b))

#try to implement gaussian here
def samp_norm_dist(b):
    temp = 0
    for i in range(1, 12):
        temp += random.randint(-1, 1)

    return b/6 * temp

def motion_model(u_t, x_prev):
    x_t = x_prev + u_t + samp_norm_dist(0.1)
    return x_t

def obs_model(z_t, x_t, m):
    #liklihood, range from 0 to 1
    q = 1
    for k in range(len(z_t)):
        #ssomething
        q=1

    return random.random()




#X_prev, set of particle
#m, numbers of particles in set X_prev
def MCL(Xprev, u_t, z_t, m):
    Xnot = Xt = []
    x_t = w_t = 0

    #for each particle in set X_prev
    for i in range(1, m):
        #sample motion model
        #x_t, probability of new cur location
        x_t = motion_model(u_t, Xprev[i])

        #measurement model
        #w_t, weight of it
        w_t = obs_model(z_t, x_t, m)

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
z = [1, 1, 1] #temp values, don't know what to do with z actually
MCL(p, 1, z, len(p))