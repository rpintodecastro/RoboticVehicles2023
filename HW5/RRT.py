# ME290 Robotic Vehicles 
#
# Template for implementation RRT (Rapidly exploring random trees)
# 
#-----------------------------------------------------------------

import random  # random number generators
import math
import matplotlib.pyplot as plt

import Environment as env # simple toolbox to process environment obstacles 

#------------------------------------------------------
# Node Class that will be used in the RRT's  search tree 
#------------------------------------------------------
class Node:
    def __init__(self, x, y):
        self.x = x # x position of the vehicle 
        self.y = y # y position of the vehicle 
        self.parent = None # parent node

#-------------------------------------------------------
#  Rapidly exploring random tree (RRT) class
#
#--------------------------------------------------------
class RRT:
    def __init__(self, start, goal, obstacles,confspace, max_dist=0.1, end_dist=0.1, max_iter=1000):
        '''        
        Initializes RRT object
        Parameters
        ----------
        start: Node
            initial state
        goal: Node
            goal state
        obstacles: array of Obstacles
        max_dist : float, optional
            maximum distance to expand nodes. The default is 0.1.
        end_dist: float, optional
            stop RRT algorithm if distance to goal node <end_dist
        max_iter : int, optional
            Maximum number of iterations for RRT algorithm. The default is 1000.
        '''        
        self.start = start # initial position
        self.goal = goal # desired goal position
        self.confspace = confspace # configuration space
        self.obstacles = obstacles # list of obstacles 
        self.max_dist = max_dist # distance to expand
        self.end_dist = end_dist # stop RRT algorithm if distantce to goal <end_dist
        self.max_iter = max_iter # maximum number of iterations
        self.node_list = [self.start]  # 

    def planning(self):
        '''
            Plans the motion of a vehicle using RRT
        ------------
        Returns: array of Nodes 
            Path generated by RRT represented as an array of nodes
        '''
        
        for i in range(self.max_iter):            
            
            rand_node = self.get_sample() # randomly sample a new state    
            nearest_node = self.find_nearest_node(rand_node) # find RRT node that is nearest to the sample            
            new_node = self.planMotion(nearest_node, rand_node, self.max_dist) # connect nearest node with sample node 
                        
            if self.check_isCollisionFree(new_node):# is the node in the collision free space?                 
                if self.check_isEdgeSafe(nearest_node,new_node):# is the path in the collision free space?
                    
                    self.node_list.append(new_node)# add new node to list
                    new_node.parent = nearest_node # define parent
                
                    if self.distance_to_goal(new_node) <= self.end_dist: # are we close to the goal state?                         
                        if self.check_isEdgeSafe(new_node, self.goal): # is the path from latest node to goal safe? 
                            return self.generate_path(len(self.node_list) - 1)
        return None
    
    def check_isEdgeSafe(self, node1, node2):
        '''
        Verify if the straigh-line edge (path) connecting node1 and node2
        belongs to the free space
        
        Parameters: node1 and node 2 are Nodes of the RRT's search tree
        Returns: boolean: edge is safe [true/false]
        '''
        obstacles=self.obstacles
        for obs in obstacles:     
            #--------------------------------------------------------------
            # Problem 2b 
            #----------------- TODO------------
            #--------------------------------------------------------------
            if obs.type =='rectangle':
                 return True  # safe (path is safe)
                
            elif obs.type == 'circle':
                 return True  # safe (path is safe)
            else:
                raise ValueError("Unknown obtacle type!!!")
        return True  # safe (path is safe)
        
    def get_sample(self):
        '''
            Generate a new sample of the state space        
        Returns: New sample (Node)
        '''
        
        xrand = random.uniform(0, self.confspace.xmax) # sample x dimension
        yrand = random.uniform(0, self.confspace.ymax) # sample y dimension
        return Node( xrand, yrand)

    def find_nearest_node(self, node):
        '''
            Identify the nearest node in the tree
        Parameters: 
            Node = target node
        Returns:   
        nearest_node : Node
        '''        
                            
        nearest_node = self.node_list[0]
        for n in self.node_list:
            if self.distance(n, node) < self.distance(nearest_node, node):
                nearest_node = n
        return nearest_node

    def planMotion(self, from_node, to_node, max_dist):
        '''
            plan motion between two nodes ("from_" and "to_")

        Parameters
        ----------
        from_node : Node
            starting node
        to_node : Node
            end node
        max_dist : int
            maximum distance we are allowed to move along the new edge.
        Returns
        -------
        Node
            A new node that is "max_dist" away from "from_node"
        '''
        
        
        d = self.distance(from_node, to_node) # what is the distance between the two nodes?
        
        # if the nodes are very close, the "destination" node will be the new node 
        if d <= max_dist:
            return to_node
        else:
            # generate a straigh line between the two nodes
            # create a new node that is "max_dist" away from "from_node"
            ratio = max_dist / d
            x = from_node.x + (to_node.x - from_node.x) * ratio
            y = from_node.y + (to_node.y - from_node.y) * ratio
            return Node(x, y)

    def check_isCollisionFree(self, node):
        '''
            Check if new node is in a collision free
        Parameters
        ----------
        node : Node
            New candidate node
        Returns
        -------
        Boolean: is node in the collision-free space? [true/false]
        '''
        obstacles=self.obstacles
        for obs in obstacles:     
            
            if obs.type =='rectangle':
                ox=obs.ox
                oy=obs.oy
                dx=obs.dx
                dy=obs.dy
                if ox - dx <= node.x <= ox + dx and oy - dy <= node.y <= oy + dy:
                    return False  # collision (node not in free space)
            elif obs.type == 'circle':
                #--------------------------------------------
                #----------- TODO --------
                #   PROBLEM 2a
                #--------------------------------------------                
                x_center = obs.x_center # x center of circle
                y_center = obs.y_center # y center of circle
                radius = obs.radius     # circle radius
            else:
                raise ValueError("Unknown obtacle type!!!")
        return True  # safe (node is colission-free space)


    def generate_path(self, end_node_index):
        '''
            Generates the final path from goal node back to starting node

        Parameters
        ----------
        end_node_index : int
            index of last node in the tree 

        Returns
        -------
            path = array of Nodes            
        '''
        path = [self.goal] # initializes the array of nodes with the goal
        node = self.node_list[end_node_index] # last node we visited during search
        while node is not self.start: # 
            path.append(node)   # add node to path
            node = node.parent  # follow parent 
        path.append(self.start)
        return path[::-1] # reverse array in order to obtain starting node as first node in the array

    @staticmethod
    def distance(node1, node2):
        '''
            compute Eucledian distance between two nodes 
        '''
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    def distance_to_goal(self, node):
        '''
            compute distance from node to goal
        '''
        return self.distance(node, self.goal)

    def draw_graph(self):
        '''
            plot the RRT tree and the obstacles
        '''        
        # draw all the nodes in the tree
        for node in self.node_list:
            if node.parent:
                plt.plot([node.x, node.parent.x], [node.y, node.parent.y], "-k", marker = '.')
        # draw all the obstacles
        for obs in self.obstacles:     
            obs.draw()            
        # draw start/end goal
        plt.plot(self.start.x, self.start.y, "xr",marker = 'o')
        plt.plot(self.goal.x, self.goal.y, "xg",marker = 'o')        
        plt.plot
        plt.grid()
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')   
        
        
    def draw_path(self,path):
        if path:
            print("Path found!")
            # print the coordinates of the nodes in the path
            for node in path:
                print(f"({node.x:.2f}, {node.y:.2f})")
    
                # highlight the final path in red
            plt.plot([node.x for node in path], [node.y for node in path], 'b-', linewidth=2)
                
        else:
            print("Could not find a path.")
    
if __name__ == "__main__":    
    
    cSpace = env.ConfigurationSpace() # create configuration space for the problem (e.g. a 2D space)
    
    start = Node(x=1, y=1) # starting position in 2D 
    goal = Node(x=9, y=9) # goal position in 2D
    
    # define obstacles
    Obstacles=[env.Rectangle(ox=5, oy=5, dx=0.5, dy=4)]
    Obstacles.append(env.Rectangle(ox=2, oy=2, dx=0.5, dy=1))
    
    
    maxDist=0.5 # [m] maximum distance that robot can travel to connect two nodes in RRT
    maxIter=3000 # maximum number of iterations for RRT algorithm
    rrt = RRT(start, goal, confspace=cSpace, obstacles=Obstacles, max_dist=maxDist, max_iter=maxIter)
    
    # execute RRT and show results
    path = rrt.planning()
    rrt.draw_graph()
    rrt.draw_path(path)
    plt.show()    