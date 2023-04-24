# -*- coding: utf-8 -*-
# ME290 Robotic Vehicles 
#
# Test RRT: is it able to generate a path that does not overlap with obstacles?
# 
#-----------------------------------------------------------------


import Environment as env
import matplotlib.pyplot as plt

from RRT import Node, RRT


cSpace = env.ConfigurationSpace() # create configuration space for the problem

start = Node(x=1, y=1) # starting position in 2D 
goal = Node(x=9, y=9) # goal position in 2D

# define obstacles
Obstacles=[env.Rectangle(ox=5, oy=5, dx=0.5, dy=4.5)]
Obstacles.append(env.Rectangle(ox=2, oy=2, dx=0.5, dy=1))
Obstacles.append(env.Circle(x_center=7, y_center=7, radius=1))    
Obstacles.append(env.Circle(x_center=9, y_center=7, radius=1))    
Obstacles.append(env.Circle(x_center=7, y_center=9, radius=0.5))    

maxDist=2*1.5 # maximum distance that robot can travel to connect two nodes in RRT
endDist=0.5 # stop RRT if distance to goal is below endDist
maxIter=3000 # maximum number of iterations
rrt = RRT(start, goal, confspace=cSpace, obstacles=Obstacles, max_dist=maxDist, end_dist=endDist, max_iter=maxIter)

# execute RRT and show results
path = rrt.planning()
rrt.draw_graph()
rrt.draw_path(path)
plt.show()    