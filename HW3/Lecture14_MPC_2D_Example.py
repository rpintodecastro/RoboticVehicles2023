# -*- coding: utf-8 -*-
"""
Created on Sat Mar  4 05:59:19 2023

ME290: Robotic Vehicles

Simple MPC example

@author: ricardo
"""

import numpy as np
from qpsolvers import Problem, solve_qp, solve_problem # https://pypi.org/project/qpsolvers/
from scipy import sparse
import matplotlib.pyplot as plt   # MATLAB plotting functions

# Parameters defining the system
Ts = 0.1    # [s] sample time
m = 1.0           # system mass

# System matrices
Ad = np.array([[1, Ts], [0, 1]])
Bd = np.array([[0], [Ts/m]])

# Initial  states
x0 = np.array([200, 0])
[nx, nu] = Bd.shape

# constraints
umax = np.array([1]) 
umin =  -umax
xmax = np.array([ 300, 3])
xmin = -xmax

# Objective function
Q = sparse.diags([1, 1])
QN = sparse.diags([1, 1])
R = np.array([0.1])

# Prediction horizon
N = 2

# Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
# - quadratic objective
P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
                       sparse.kron(sparse.eye(N), R)], format='csc')

# - linear dynamics
Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), Ad)
Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)
Aeq = sparse.hstack([Ax, Bu])
leq = np.hstack([-x0, np.zeros(N*nx)])
#ueq = leq

# - input and state constraints
lineq = np.hstack([np.kron(np.ones(N+1), xmin), np.kron(np.ones(N), umin)])
uineq = np.hstack([np.kron(np.ones(N+1), xmax), np.kron(np.ones(N), umax)])

q=np.zeros(nx*(N+1)+nu*N)


# Simulate in closed loop
nsim = round(100/Ts)   # simulation time
# initialize logging variables
sim_x = np.zeros((nsim,nx)) 
sim_u = np.zeros((nsim,nu))
sim_t = np.zeros((nsim,1))
for i in range(nsim):
    # Solve MPC
    # min z'Pz+q'z
    # s.t. G<=h
    # s.t. A=b
    problem = Problem(P, q=q, G=None, h=None, A=Aeq, b=leq, lb=lineq, ub=uineq)
    sol = solve_problem(problem, solver="cvxopt")


    # Check solver status
    if not sol.found:
       raise ValueError('QPsolver did not solve the problem!')
     
    # Apply first control input to the plant
    ctrl =sol.x[-N*nu:-(N-1)*nu]
    x0 = Ad.dot(x0) + Bd.dot(ctrl)
    
    # Update initial state
    leq[:nx] = -x0
    
    #save results
    sim_t[i,0]=i*Ts
    sim_x[i,:]=x0
    sim_u[i,:]=ctrl

#plotting results
plt.subplot(311)
plt.plot(sim_t, sim_x[:,0])
plt.grid()
plt.ylabel('x1')
plt.subplot(312)
plt.plot(sim_t, sim_x[:,1])
plt.ylabel('x2')
plt.grid()
plt.ylim(-10, 1)
plt.subplot(313)
plt.plot(sim_t, sim_u)
plt.ylim(-3, 1)
plt.ylabel('u')
plt.grid()
plt.xlabel('time [s]')