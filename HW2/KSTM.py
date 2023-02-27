# -*- coding: utf-8 -*-

"""
UC Merced
ME290: Robotic Vehicles

Kinematic Single Track Model

"""

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
        self.lx = 1;          # distance of reference point 
        
    def f_dot(self, x, u):
        """
        Implements the right hand side of 
            x_dot - f_dot(x,u)
            
        Inputs: 
        -------------    
        x : np.array of system state  with [X,Y,psi]
        u : np.array of system inputs [v, steer]
            
        Returns: 
        --------------
        x_dot : np.array of the field of the system with shape 1xNSTATES
        """
        X = x[0];    # extract global X position of reference point
        Y = x[1];    # extract global Y position of reference point
        psi = x[2]   # extract yaw angle
        steer = u[1] # stering angle
        vx = u[0]    # longitudinal velocity of reference point

        #****************************************
        # TO BE COMPLETED             
        #****************************************

        Psi_dot = 0 # yaw rate
        X_dot = 0;
        Y_dot = 0;   
        
        # build vector field
        x_dot = np.array([X_dot, Y_dot, Psi_dot], dtype='float64')
        return x_dot
        
    def yawrate_2_steering(self,u):
        '''
        converts desired yaw-rate into steering angle

        Parameters
        ----------
        u : np.array of system inputs [v, yaw-rate]

        Returns
        -------
        u : np.array of system inputs [v, steer]

        '''
        V = u[0]
        u_yawrate = u[1]
        steer=np.arctan2(self.L*u_yawrate, V)
        return np.array([V,steer])


if __name__ == "__main__":
   
    #**** BUILD kinematic vehicle model *****
    kstm = KSTModel()
    
    #********* Parameters ************
    u_const = 1*3.14/180   # steering angle [rad]
    V = 20                 # velocity 

    
    # ***** Simulate KSTM  *****    
    x0= np.array( [0, 0, 0]);    # initial state [X, Y, psi]

    Tsample = 0.01 # [s] sampling time for discretization of vehicle model   
    Tsim = 20 #[s] simulation time
    Nsimsteps = round(Tsim/Tsample); # number of simulation steps

    u = np.zeros((Nsimsteps,2)) # control input
    u[:]= [V, u_const]


    sim = MyODEsim.Sim()
    x=sim.simulate_euler(kstm.f_dot, x0,u,  Tsample) # simulate
    
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
    
    