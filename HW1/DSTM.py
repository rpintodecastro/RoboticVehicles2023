# -*- coding: utf-8 -*-

"""
ME290: Robotic Vehicles

Kinematic Single Track Model

"""

import numpy as np
import matplotlib.pyplot as plt
import math 
import MyODEsim

class DSTModel:
    """
        Kinematic Single Track Model (KSTM)
    """
    def __init__(self):
        # Parameters of the vehicle model
        self.V = 30;    # longitudinal velocity [m/s]

        self.lf = 1.22; # distance from Center of Gravity to front axle [m]
        self.lr = 1.18; # distance from Center of Gravity to rear axle [m]


        self.m = 1000   # vehicle mass [kg]
        self.Iz = 1130  # yaw inertia [kg⋅m2]        
        
        self.Cf = 3*30e3 # cornering stiffness, front tire [N/rad]
        self.Cr = 3*35e3 # cornering stiffness, rear tire [N/rad]

    def f_dot(self, x, u):
        """
        Implements the right hand side of 
            x_dot - f_dot(x,u)
            
        Inputs: 
        -------------    
        x : np.array of system state  with shape 1xNSTATE
        u : np.array of system input with shape 1xNINPUTS
            
        Returns: 
        --------------
        x_dot : np.array of the field of the system with shape 1xNSTATES
        """
        X = x[0];   # extract global X position
        Y = x[1];   # extract global Y position
        psi = x[2]  # extract yaw angle
        beta = x[3] # side-slip angle
        v_psi = x[4] # yaw-rate
        steer = u[0] # stering angle
        
        #---------------------------------------------
        # TODO        
        #---------------------------------------------
        X_dot = 0
        Y_dot = 0
        Psi_dot = 0
        beta_dot = 0
        v_psi_dot = 0
        
        # build vector field
        x_dot = np.array([X_dot, Y_dot, Psi_dot, beta_dot, v_psi_dot],dtype='float64')
        return x_dot



if __name__ == "__main__":
   
    kstm = DSTModel()

    u_const = 0      # steering angle [rad]

    # check right hand side of x_dot  = f_dot(x,u)    
    x0= np.array( [0, 0, 0, 0, 0]);    # initial state [X, Y, psi, vy, v_psi]
    u = np.array([u_const])

    # Simulate KSTM    
    Tsample = 0.01 # [s] sampling time for discretization of vehicle model   
    Nsim = 20000; # number of simulation steps
    u = np.zeros((Nsim,1)) # control input
    u[:]= u_const      # steering angle
    sim = MyODEsim.Sim()
    x=sim.simulate_euler(kstm.f_dot, x0,u,Tsample) # simulate
    
    # plotting using matplotlib
    plt.figure(dpi=300)   
    time = np.arange(0,Nsim)
    X = x[:,0]
    Y = x[:,1]
    psi=x[:,2]

    
    # *****  plot X vs Y  *****     
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


    print(x[-1,:])