import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import sympy as smp
from math import *
import scienceplots
plt.style.use(['science', 'notebook'])
from ParticleGeneration import*


#def move/change particle and prob
#parameters: x value array(where these particles are), 
# input command(left or right), w value array(the weight of each particle)
#returns new x values and the corresponding weight
generate(0, 1)

#for every particle in x, we create a gaussian graph and add them together
# they can all have different "mean"(location)
x = {1, 2, 3, 4, 5}
def controlGraph(x):
    count, bins, ignored = plt.hist(x, 30, density='True')
    plt.plot(bins, gauss(bins,1,0.1) + gauss(bins,2,0.1))
    plt.show()

controlGraph(x)

