# -*- coding: utf-8 -*-
# ME290 Robotic Vehicles 
#
# Auxiliary classes to process obstacles (e.g. Rectangles, Circles) as well 
# the environment 
#
#-----------------------------------------------------------------

import matplotlib.pyplot as plt
import numpy as np
import Line
import Circle as auxCircle

#---------------------------------------------------
# Define a 2D Configuration space 
#---------------------------------------------------
class ConfigurationSpace:
    def __init__(self, xmax=10, ymax=10):
        self.xmax = xmax # maximum value for the x coordinate
        self.ymax = ymax # maximum value for the y coordinate

#---------------------------------------------------
# Define a generic Obstacle
#---------------------------------------------------
class Obstacle:
    def __init__(self, type):
        self.type = type

#---------------------------------------------------
# Rectangle Obstacle
#---------------------------------------------------      
class Rectangle(Obstacle):
    def __init__(self, ox, oy, dx, dy):
        '''        
            ox,oy = center of the rectangle 
            dx = half width 
            dy = hald height
        '''
        super().__init__(type='rectangle')
        self.ox = ox
        self.oy = oy
        self.dx = dx
        self.dy = dy      
        
    def draw(self):
        '''
        draw the obstacle in a 2D plane
        '''
        ox=self.ox
        oy=self.oy
        dx=self.dx
        dy=self.dy
        plt.plot([ox - dx, ox + dx, ox + dx, ox - dx, ox - dx],
                  [oy - dy, oy - dy, oy + dy, oy + dy, oy - dy], "-k")
        
    def segment_intersects_rectangle(self, p1, p2):
        # Does this rectangle interst with line segment (p1,p2)?
        # p1 and p2 are the endpoints of the segment (defined by Nodes)
 
        # define coordinates of the rectangle 
        xmin=self.ox - self.dx
        xmax=self.ox + self.dx
        ymin=self.oy - self.dy 
        ymax=self.oy + self.dy 
        rectangle = (xmin, ymin, xmax, ymax )  # (x1, y1, x2, y2)
 
        #----------------------------------------------
        # TODO: Problem 2
        #----------------------------------------------
        
        return False #"The rectangle does not intersect the line segment

#---------------------------------------------------
# Circle Obstacle
#---------------------------------------------------      
class Circle(Obstacle):
    def __init__(self, x_center, y_center, radius):
        '''        
            ox,oy = center of the rectangle 
            dx = half width 
            dy = hald height
        '''
        super().__init__(type='circle')
        self.x_center = x_center
        self.y_center = y_center
        self.radius = radius
        
    def draw(self):
        '''
        draw the obstacle in a 2D plane
        '''
        
        angles = np.linspace(0, 2*np.pi, 100)
        
        # Calculate the x and y coordinates of the circle
        x = self.x_center + self.radius*np.cos(angles)
        y = self.y_center + self.radius*np.sin(angles)
        
        
        # Plot the circle
        plt.plot(x, y, "-k")
        
        # Set the aspect ratio to be equal
        #plt.set_aspect('equal')

    def segment_intersects_circle(self, p1, p2):
        
        # Does this rectangle interst with line segment (p1,p2)?
        # p1 and p2 are the endpoints of the segment (defined by Nodes)
        line = ((p1.x, p1.y), (p2.x, p2.y)) # 
        circle = (self.x_center, self.y_center, self.radius)  # (cx, cy, r)
        
        #----------------------------------------------
        # TODO: Problem 2
        #----------------------------------------------
        return False #print("The line does not intersect the circle.")
    
if __name__ == "__main__":    
    
    # define some obstacles
    Obstacles=[Rectangle(ox=5, oy=5, dx=0.5, dy=4)]
    Obstacles.append(Rectangle(ox=2, oy=2, dx=0.5, dy=1))
    #Obstacles.append(Circle(x_center=8, y_center=2, radius=2))        
    
    # draw all the obstacles
    for obs in Obstacles:     
        obs.draw()     

    plt.grid()
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')    