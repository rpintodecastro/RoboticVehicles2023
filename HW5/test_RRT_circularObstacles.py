# -*- coding: utf-8 -*-
# ME290 Robotic Vehicles 
#
# Test RRT: is it able to code with circular obstacles?
# 
#-----------------------------------------------------------------

import Environment as env
import matplotlib.pyplot as plt

from RRT import Node, RRT


cSpace = env.ConfigurationSpace() # create configuration space for the problem

start = Node(x=1, y=1) # starting position in 2D 
goal = Node(x=9, y=9) # goal position in 2D

# define obstacles
Obstacles=[env.Rectangle(ox=5, oy=5, dx=0.5, dy=4)]
Obstacles.append(env.Rectangle(ox=2, oy=2, dx=0.5, dy=1))
Obstacles.append(env.Circle(x_center=8, y_center=2, radius=2))    

maxDist=0.5 # maximum distance that robot can travel to connect two nodes in RRT
maxIter=3000 # maximum number of iterations
rrt = RRT(start, goal, confspace=cSpace, obstacles=Obstacles, max_dist=maxDist, max_iter=maxIter)

# execute RRT and show results
path = rrt.planning()
rrt.draw_graph()
rrt.draw_path(path)
plt.show()    