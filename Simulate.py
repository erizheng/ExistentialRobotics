import numpy as np
import scipy as sp
import matplotlib.pyplot as plt
import sympy as smp
import pygame
from math import *

pygame.init()

"""Define color"""
red = (200, 0, 0)
blue = (0, 0, 255)
green = (0, 155, 0)
yellow = (155, 155, 0)
white = (255, 255, 255)
black = (0, 0, 0)


display_width = 560
display_height = 100
world_size = display_width

class robot:
	def __init__(self):
		self.x = 0
		self.y = 20
		self.orientation = 0

	def set(self, x, y,orientation):
		self.x = x
		self.y = y
		self.orientation = orientation

	def move(self, turn, forward):
		self.orientation = self.orientation + turn
		#cos value 0degree->1 , 90degree->0, 180degree->-1
		#we must pass degree in radian
		self.x = self.x + forward*cos(self.orientation*pi/180)
		#sin value 0degree->0 , 90degree->1, 180degree->0
		#we must pass degree in radian
		self.y = self.y - forward*sin(self.orientation*pi/180)
		
		self.orientation %= 360
		self.x %= world_size
		self.y %= world_size
		
	def draw(self):
		bot_img = pygame.image.load("square.png")
		DEFAULT_IMAGE_SIZE = (50, 50)
		img = pygame.transform.scale(bot_img, DEFAULT_IMAGE_SIZE)
		print 
		self.orientation
		screen.blit(img, (self.x, self.y))
		
		
		

class Simulator(object):
	def main(self , screen , robot):
		clock = pygame.time.Clock()
		robot.draw()
		delta_orient = 0.0
		delta_forward = 0.0
		while 1:
			clock.tick(30)
			screen.fill(white)
			for event in pygame.event.get():
				if event.type == pygame.QUIT:
					return
				if event.type == pygame.KEYDOWN:

					if event.key == pygame.K_RIGHT:
						delta_forward = 1
					elif event.key == pygame.K_LEFT:
						delta_forward = -1
				elif event.type == pygame.KEYUP:
					if event.key == pygame.K_RIGHT or event.key == pygame.K_LEFT:
						delta_orient = 0.0
						delta_forward = 0.0

			robot.move(delta_orient, delta_forward)
			robot.draw()
			pygame.display.flip()



if __name__ == '__main__':
	pygame.init()
	pygame.display.set_caption("Move")
	screen = pygame.display.set_mode((display_width,display_height))
	robot = robot()
	Simulator().main(screen , robot)

