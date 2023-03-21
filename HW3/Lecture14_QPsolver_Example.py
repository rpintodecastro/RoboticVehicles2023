# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 05:56:18 2023

Simple QP problem

@author: ricardo
"""

import numpy as np
from qpsolvers import Problem, solve_qp, solve_problem # https://pypi.org/project/qpsolvers/


# Solve MPC
# min (1/2)z'Pz+q'z
# s.t. G<=h
# s.t. A=b

P = np.array([[1,0], [0,1]],dtype='float')  
q = np.array([-6, -7],dtype='float')  
G = np.array([ [-3,  -2],
               [-1,   1], 
               [ 1,   1],
               [ 2/3,  -1]],dtype='float')
h = np.array([-6, 3, 7, 4/3],dtype='float')


problem = Problem(P, q, G, h, A=None, b=None)
sol = solve_problem(problem, solver="cvxopt")
print(f"QP solution: x = {sol.x}")

