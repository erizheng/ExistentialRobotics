import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import sympy as smp
from math import *
import scienceplots
plt.style.use(['science', 'notebook'])

def generate():
    #generate random variable according to specific distribution
    bins = np.linspace(0, 3, 100)
    # lamda = 2
    # f = (1/variance*np.sqrt(2*np.pi))*np.exp(-(1/2)((x-mean)/variance)**2)
    # F = (1/np.sqrt(2*np.pi))*np.exp(-(x**2)/2)

    mu = 0 #location
    sigma = 1 #width, how spread out, deviation
    x = np.random.normal(mu, sigma, 100) #array of distribution
    #x is the x coordinate value of the graph
    #probably need to do like a for loop for each x value and plug 
    # it into the gaussian equation to get the y value(the weight)

    # plt.figure(figsize=(8,3))
    # plt.plot(x,f,label=r'$f(x)$')
    # plt.plot(x,F,label=r'$F(x)$')
    # plt.legend()
    # plt.xlabel('$x$', fontsize=20)
    # plt.legend()
    # plt.show()

    # Us = np.random.rand(10000)
    #F_inv_Us = -np.log(1-Us)/lamda
    # F_inv_Us = x[np.searchsorted(F[:-1], Us)]

    plt.figure(figsize=(8,3))
    # plt.plot(x,f,label=r'$f(x)$')
    # plt.hist(F_inv_Us, histtype = 'step', color ='red', density= 'norm', bins=100, label ='$F^{-1}(u)$')
    # plt.legend()
    # plt.xlabel('$x$', fontsize=20)
    # plt.legend()
    # plt.show()

    count, bins, ignored = plt.hist(x, histtype= 'step', color='red', density='norm', bins=100)
    plt.plot(bins, 1/(sigma * np.sqrt(2 * np.pi)) * np.exp( - (bins - mu)**2 / (2 * sigma**2) ), linewidth=2, color='r')
    plt.show()