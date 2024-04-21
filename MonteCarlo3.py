import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.image as mpimg
from ipywidgets import Button, HBox, VBox, Output
from IPython.display import display, Javascript, clear_output
import matplotlib.animation as ani
#from google.colab import output
import time
import keyboard



def handle_key_event(key):
    pf.MCL(key)

class MCPF:

    def __init__(self, map, num_part):

        self.measure_noise = 10
        self.motion_noise = 0.5
        self.std = 20
        self.map = map

        self.robot_x = 330
        self.robot_y = 230
        self.landmarks = [[235, 100], [420, 150]]
        #particles [[x,y,w], ...]
        self.particles = [[0,0,0]]
        self.num_particles = num_part

        #plotting
        self.fig, self.ax = plt.subplots()
        self.ax.imshow(map)
        
        #plot without the particles
        self.scatter = self.ax.scatter(self.robot_x, self.robot_y, color= "black", marker= "s", s= 500)
        plt.axis("off")
        plt.show()

        self.generate_part()
        self.draw_robot("Initial")

    def calc_dist(self):
        #measured_dist is an array
        measured_dist = [] #want to make it so measured distance takes in LiDAR
        particle_dist = []

        for i in range(len(self.landmarks)):
            #landmarks[i][0], x coordinate
            #landmarks[i][1], y coordinate
            dist_temp1 = np.sqrt(((self.robot_x - self.landmarks[i][0])**2 + (self.robot_y - self.landmarks[i][1])**2)) * np.ones(self.num_particles)
            measured_dist.append(dist_temp1)

            dist_temp2 = np.sqrt(((self.particles[:,0] - self.landmarks[i][0])**2 + (self.particles[:,1] - self.landmarks[i][1])**2) * np.ones(self.num_particles))
            particle_dist.append(dist_temp2)

        #adds Guassian noise to particle
        particle_with_noise = particle_dist + np.random.normal(0, self.measure_noise, (2,self.num_particles))

        difference = np.abs(particle_with_noise - measured_dist)

        return difference

    #a is mean, b is sigma^2
    def prob_norm(a,b):
        return 1/(np.sqrt(2*np.pi *b)) * np.exp(-(a**2)/2*b)

    def update(self):
        weight = self.calc_dist()
        w = self.prob_norm(weight, self.std^2)

        #multiply all the weights for each landmark
        final_w = 1
        for i in self.landmarks:
            final_w = final_w * w[i]

        #normalize with landmarks
        final_w = final_w/np.sum(final_w)

        self.particles[:,2] = final_w
        #normalize with num of particles
        self.particles[:,2] = self.particles[:,2]/np.sum(self.particles[:,2])
    
    #controls for moving robot
    def robot_move(self, event):

        self.step_size = 20 # speed of robot

        if event == 'left':
            self.robot_x -= self.step_size # robot moves left when left arrow key is pressed

        if event == 'right':
            self.robot_x += self.step_size # robot moves left when left arrow key is pressed

        if event == 'down':
            self.robot_y += self.step_size

        if event == 'up':
            self.robot_y -= self.step_size

        self.scatter.set_offsets([(self.robot_x, self.robot_y)]) 

    #particle generation
    def generate_part(self):
        self.particles = np.zeros((self.num_particles, 3))
        self.particles[:,0] = np.random.uniform(160, 490, self.num_particles)
        self.particles[:,1] = np.random.uniform(150, 350, self.num_particles)
        self.particles[:,2] = np.ones(self.num_particles) / self.num_particles

        #draws on map
        self.scatter_particles = self.ax.scatter(self.particles[:,0], self.particles[:,1], color="red", marker="o", s=self.particles[:,2] * 500)
    
    #drawing
    def draw_robot(self, stage = ""):
        clear_output(wait = True)
        self.fig, self.az = plt.subplots()
        self.ax.imshow(self.map)
        self.scatter = self.ax.scatter(self.robot_x, self.robot_y, color= "black", marker= "s", s= 500)
        self.scatter_part = self.ax.scatter(self.particles[:,0], self.particles[:,1], c = self.particles[:,2], cmap = cm.jet, marker = "o", s= 500/ self.num_particles)
        plt.axis('off')
        plt.title(stage)
        # plt.draw()
        plt.close()
        plt.show()

    
    def prediction(self, event):
        if event == 'left':
            self.particles[:,0] -= np.random.normal(self.motion_noise + self.step_size, 2.5, self.num_particles) # shifts particles with control input and motion noise

        if event == 'right':
            self.particles[:,0] += np.random.normal(self.motion_noise + self.step_size, 2.5, self.num_particles) # shifts particles with control input and motion noise

        if event == 'down':
            self.particles[:,1] += np.random.normal(self.motion_noise + self.step_size, 2.5, self.num_particles)

        if event == 'up':
            self.particles[:,1] -= np.random.normal(self.motion_noise + self.step_size, 2.5, self.num_particles)

        # updates position of robot on plot of map
        self.scatter_particles.set_offsets(self.particles)

        #clear_output()
        self.draw_robot("Prediction Step")
        plt.pause(0.1)

        # perform update step after prediction step
        self.update()

        # perform resampling after update step
        self.resampling()

    def resampling(self):
        # changes x-coordinate of particles with a distribution of the weights of each particle
        self.particles[:,0] = np.random.choice(self.particles[:,0], size = self.num_particles, p = self.particles[:,2])

        # changes y-coordinate of particles with a distribution of the weights of each particle
        self.particles[:,1] = np.random.choice(self.particles[:,1], size = self.num_particles, p = self.particles[:,2])

        # assigns particle weight's uniformly (each particle has the same weight now)
        self.particles[:,2] = np.ones(self.num_particles) / self.num_particles

        # updates particle states on plot of map
        self.scatter_particles.set_offsets(self.particles)

        #clear_output()
        self.draw_robot_plot("Resampling Step")
        plt.pause(0.1)
        
    #the top level controller for this program
    def MCL(self, key):
        if key in ['ArrowLeft', 'a']:
            self.robot_move('left')
            self.prediction('left')
        elif key in ['ArrowRight', 'd']:
            self.robot_move('right')
            self.prediction('right')
        elif key in ['ArrowUp', 'w']:
            self.robot_move('up')
            self.prediction('up')
        elif key in ['ArrowDown', 's']:
            self.robot_move('down')
            self.prediction('down')


img = mpimg.imread('grid_map.png')
pf = MCPF(img, 50)
#pf.init_js_event()
while 1:
    k = keyboard.wait()
    handle_key_event(k)
    print(k)
        
    
    