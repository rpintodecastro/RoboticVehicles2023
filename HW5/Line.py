# -*- coding: utf-8 -*-
"""
# ME290 Robotic Vehicles 

Auxiliary functions to check intersection between lines and rectangles
"""

def orientation(p, q, r):
    """
    Function to find the orientation of the ordered triplet (p, q, r).
    Returns 0 if p, q, and r are colinear.
    Returns 1 if the direction of the triplet is clockwise.
    Returns 2 if the direction of the triplet is counterclockwise.
    """
    val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
    if val == 0:
        return 0
    elif val > 0:
        return 1
    else:
        return 2


def do_line_segments_intersect(line1, line2):
    """
    Function to check if two line segments intersect.
    """
    p1, q1 = line1
    p2, q2 = line2

    # Find the four orientations needed for general and special cases
    o1 = orientation(p1, q1, p2)
    o2 = orientation(p1, q1, q2)
    o3 = orientation(p2, q2, p1)
    o4 = orientation(p2, q2, q1)

    # General case
    if o1 != o2 and o3 != o4:
        return True

    # Special Cases
    # p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if o1 == 0 and is_point_on_segment(p1, p2, q1):
        return True

    # p1, q1 and q2 are colinear and q2 lies on segment p1q1
    if o2 == 0 and is_point_on_segment(p1, q2, q1):
        return True

    # p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if o3 == 0 and is_point_on_segment(p2, p1, q2):
        return True

    # p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if o4 == 0 and is_point_on_segment(p2, q1, q2):
        return True

    # If none of the above cases apply
    return False


def is_point_on_segment(p, q, r):
    """
    Function to check if a point r lies on a line segment pq.
    """
    if (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
        q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1])):
        return True
    return False

def does_rectangle_intersect_segment(rectangle, segment):
    """
    Function to check if a rectangle intersects a line segment.
    """
    x1, y1, x2, y2 = rectangle
    line_segment = ((x1, y1), (x2, y1))  # Top side of rectangle
    if do_line_segments_intersect(line_segment, segment):
        return True

    line_segment = ((x2, y1), (x2, y2))  # Right side of rectangle
    if do_line_segments_intersect(line_segment, segment):
        return True

    line_segment = ((x2, y2), (x1, y2))  # Bottom side of rectangle
    if do_line_segments_intersect(line_segment, segment):
        return True

    line_segment = ((x1, y2), (x1, y1))  # Left side of rectangle
    if do_line_segments_intersect(line_segment, segment):
        return True

    return False

if __name__ == "__main__":    
    
    #--------------------------------------------------------------------------
    # check if two lines intersect
    line1 = ((1, 1), (3, 1)) #line1=(x1,y1), (x2,y2)#
    line2 = ((2, 0), (2, 1)) #line1=(x3,y3), (x4,y4)#
    if do_line_segments_intersect(line1, line2):
        print("The two line segments intersect.")
    else:
        print("The two line segments do not intersect.")

    #--------------------------------------------------------------------------        
    # check if line and rectangle intersect
    rectangle = (0, 0, 2, 2) #line1=(x1,y1), (x2,y2)#
    line_segment = ((1, 1), (3, 3)) #line1=(x3,y3), (x4,y4)#
    if does_rectangle_intersect_segment(rectangle, line_segment):
        print("The rectangle intersects the line segment.")
    else:
        print("The rectangle does not intersect the line segment.")        