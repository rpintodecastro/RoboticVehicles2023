# -*- coding: utf-8 -*-
"""
# ME290 Robotic Vehicles 

Auxiliary functions to check intersection between lines and circle
"""

import math

def does_line_intersect_circle(line, circle):
    """
    Function to check if a line intersects a circle.
    """
    x1, y1 = line[0]
    x2, y2 = line[1]
    cx, cy, r = circle

    # Compute the coefficients of the quadratic equation (ax^2 + bx + c = 0)
    a = (x2 - x1)**2 + (y2 - y1)**2
    b = 2 * ((x2 - x1) * (x1 - cx) + (y2 - y1) * (y1 - cy))
    c = cx**2 + cy**2 + x1**2 + y1**2 - 2 * (cx * x1 + cy * y1) - r**2

    # Compute the discriminant (b^2 - 4ac)
    discriminant = b**2 - 4*a*c

    # If the discriminant is negative, the line and circle do not intersect
    if discriminant < 0:
        return False

    # If the discriminant is zero, the line and circle intersect at one point
    elif discriminant == 0:
        # Compute the value of t at the intersection point
        t = -b / (2*a)
        # If t is between 0 and 1, the intersection point is on the line segment
        if 0 <= t <= 1:
            return True
        else:
            return False

    # If the discriminant is positive, the line and circle intersect at two points
    else:
        # Compute the values of t at the two intersection points
        t1 = (-b + math.sqrt(discriminant)) / (2*a)
        t2 = (-b - math.sqrt(discriminant)) / (2*a)
        # If either t value is between 0 and 1, the intersection point is on the line segment
        if 0 <= t1 <= 1 or 0 <= t2 <= 1:
            return True
        else:
            return False
        
if __name__ == "__main__":    
        
    line = ((1, 1), (10, 1)) # line ((x1,y1),(x2,y2))
    circle = (3, 1, 1)  # circle: (cx, cy, r)
    if does_line_intersect_circle(line, circle):
        print("The line intersects the circle.")
    else:
        print("The line does not intersect the circle.")        