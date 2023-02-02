# -*- coding: utf-8 -*-

"""
UC Merced
ME290: Robotic Vehicles

Template for "Kinematic Single Track Model"

"""

# Import relevant libraries
import numpy as np
import matplotlib.pyplot as plt
import math 
import MyODEsim

class KSTModel:
    """
        Kinematic Single Track Model (KSTModel)
    """
    def __init__(self):
        # Parameters of the vehicle model
        self.L = 1.22+1.18;   # wheel base [m]
        self.V = 10;   # longitudinal velocity [m/s]
        
    def f_dot(self, x, u):
        """
        Implements  right hand side of 
            x_dot - f_dot(x,u)
            
        Inputs: 
        -------------    
        x : np.array,  system state  with shape 1xNSTATE
        u : np.array,  input steering angle with shape 1x1
            
        Returns: 
        --------------
        x_dot : np.array of the vector field of the system with shape 1xNSTATES
        """
        X = x[0];    #  X position
        Y = x[1];    # Y position
        psi = x[2]   # extract yaw angle
        vx = self.V  # longitudinal velocity 
        vy = 0       # lateral velocity 
        steer = u[0] # stering angle

        #*************************
        # !!!!!!!! TODO !!!!!!
        #*************************
        X_dot = 0;
        Y_dot = 0;   
        Psi_dot = 0 
        
        # build vector field
        x_dot = np.array([X_dot, Y_dot, Psi_dot], dtype='float64')
        return x_dot
        


if __name__ == "__main__":
   
    kstm = KSTModel()
    
    u_const = 0     # steering angle [rad]
    # check right hand side of x_dot  = f_dot(x,u)    
    x0= np.array( [0, 0, 0]);    # initial state [X, Y, psi]
    u = np.array([u_const])      # steering angle
    
    # ***** Simulate KSTM  *****    
    Tsample = 0.01 # [s] sampling time for discretization of vehicle model   
    Tsim = 200 #[s] simulation time
    Nsimsteps = round(Tsim/Tsample); # number of simulation steps
    u = np.zeros((Nsimsteps,1)) # control input
    u[:]= u_const
    
    sim = MyODEsim.Sim()
    x=sim.simulate_euler(kstm.f_dot, x0,u,Tsample) # simulate
    
    # *****  plot X vs Y  ***** 
    plt.figure(dpi=1200)
    time = np.arange(0,Nsimsteps)
    X = x[:,0]
    Y = x[:,1]
    psi=x[:,2]
    time= time * Tsample    
    plt.plot(X,Y)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.show()

    # *****  plot X,Y, yaw-angle  ***** 
    f, (ax1, ax2, ax3) = plt.subplots(3, 1, sharey=True)
    ax1.plot(time, X)    
    ax1.set_ylabel('X[m]')
    ax2.plot(time, Y)    
    ax2.set_ylabel('Y[m]')
    ax3.plot(time, psi)    
    ax3.set_ylabel('yaw[rad]')
    ax3.set_xlabel('time [s]')
    
    