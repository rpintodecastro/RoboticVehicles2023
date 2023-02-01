# -*- coding: utf-8 -*-

"""
ME290: Robotic Vehicles

Interface to numerical simulation tools

"""

import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as  integrate
from scipy.integrate import odeint

class Sim:
    
    def simulate_euler(self, f_dot, x0, u, Tsample):
        """
        Simulate system dynamics 
            x_dot = f_dot(x,u)
            
        Inputs:
        ----------
        f_dot:  function pointer to f_dot(x,u)
        x0 : np.array of initial states with shape 1xNSTATES
        u  : np.array of control input with shape NUM_SIM_STEPS x nINPUTS
        Tsample : float with  simulaton sample time [s]
    
        Returns
        -------
        x  : np.array with simulation states with shape  NUM_SIM_STEPS x nINPUTS
    
        """
        n_states = x0.size # number of states
        N = u.size         # number of simulation steps
    
        x = np.zeros((N, n_states))      # create state vector over simulation horizon
        x[0,:] = x0 # set initial state
        
            
        # simulate 
        for k in range(1,N):
            x_old = x[k-1,:]  # extract state in previous time state
            u_k = u[k,:]      # current control input

            # Euler discretization
            x_k = x_old + Tsample*f_dot(x_old, u_k)
            #-------------
            x[k,:] = x_k     # save state    
        return x;


if __name__ == "__main__":

    print("*****Test Simulation function **********")
    
    mdl = Sim()

    def f_dot(x,u):
        # test a simple integrator
        # x_dot = u
        return np.array([u])
    
    # ************ check right hand side of x_dot  = f_dot(x,u)    
    x0= np.array( [0]);    # initial state [vx, vy]
    u = 0
    print(f_dot(x0, u))
    print(x0.size)

    #************ Simulate system **********     
    Tsample = 0.1 # [s] sampling time for discretization of vehicle model   
    Nsim = 20; # number of simulation steps
    u = np.zeros((20,1)) # control input
    u[11:20]=1
    
    x=mdl.simulate_euler(f_dot, x0,u,Tsample) # simulate
    
    # ************* plot results using matplotlib
    time = np.arange(0,Nsim)
    time= time * Tsample    
    plt.plot(time,x)
    plt.xlabel("time [s]")
    plt.ylabel("states")
    plt.legend( ['x1=v_x', 'x2=v_y'])
    plt.show()
