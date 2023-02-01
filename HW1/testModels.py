# -*- coding: utf-8 -*-

"""
ME290: Robotic Vehicles

Compare different vehicle models

"""

import numpy as np
import matplotlib.pyplot as plt
import math 
import MyODEsim
import KSTM
import DSTM

# PLOT X vs Y position of the vehicle
def myplotXY(x,style):

    # plotting using matplotlib
    X = x[:,0]
    Y = x[:,1]    
    plt.plot(X,Y,style)
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")

# PLOT X, Y and psi over time
def myplot3(x,style):
 
    Nsim =x.shape
    Nsim = Nsim[0]
    time = np.arange(0,Nsim)
    time= time * Tsample
    X  = x[:,0]
    Y = x[:,1]
    psi=x[:,2]
    
    plt.subplot(311)
    plt.plot(time, X,style)    
    plt.ylabel('X[m]')    
    plt.subplot(312)    
    plt.plot(time,Y,style)
    plt.ylabel('Y[m]')
    plt.subplot(313)    
    plt.plot(time, psi,style)    
    plt.ylabel('yaw-angle [rad]')
    plt.xlabel('time [s]')
    
if __name__ == "__main__":
    

    V=30   # set velocity [m/s]
    
    # create vehicle models
    kstm = KSTM.KSTModel()
    dstm = DSTM.DSTModel()
    kstm.V = V # set velocity for kinematic STM
    dstm.V = V # set velocity for dynamic STM

    # definit steering angle and initial state    
    u_const = 0      # steering angle [rad]
    x0_pos= np.array( [0, 0, 0]);    # initial state [X, Y, psi]
    x0_dstm = np.array([0, 0, 0, 0, 0]) # [X, Y, psi, vy, yaw-rate]

    Tsample = 0.01 # [s] sampling time for discretization of vehicle model   
    Tsim = 200 #[s] simulation time
    Nsimsteps = round(Tsim/Tsample); # number of simulation steps
    u = np.zeros((Nsimsteps,1)) # control input
    u[:]= u_const
    
    # Simulate KSTM and DSTM   
    plt.figure(dpi=300)    
    sim = MyODEsim.Sim()
    x_kstm=sim.simulate_euler(kstm.f_dot, x0_pos,  u, Tsample) # simulate
    x_dstm=sim.simulate_euler(dstm.f_dot, x0_dstm, u, Tsample)
    
    # plot X vs Y
    plt.figure(dpi=300)
    myplotXY(x_kstm,'-')
    myplotXY(x_dstm,'--')
    plt.legend(['KSTM','DSTM'])
    plt.show()

    # plot X,Y, psi
    myplot3(x_kstm,'-')
    myplot3(x_dstm,'--')
    plt.legend(['KSTM','DSTM'])
    plt.savefig('plot.png', dpi=300)
