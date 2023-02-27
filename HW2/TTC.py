# -*- coding: utf-8 -*-
"""
Created on Fri Feb 24 10:09:41 2023

ME290: Robotic Vehicles

Template for Homework #2 
Problem 2: Trajectory Tracking Controller (TTC)

Implements algorithms for TTC
"""


import numpy as np
import matplotlib.pyplot as plt
import math 


class TrajectoryTracking:
    """
        Trajectory Tracking Controller (TTC)
    """    
    
    def __init__(self):
        # Parameters of the vehicle model and controller
        self.lx = 1;     # distance of reference point 
        self.L = 1;      # wheel base of the vehicle
        self.kX = 0.1;   # proportional gain (error X )
        self.kY = 0.1;   # proportional gain (error Y)
        
    def control_io(self, x, p_ref, p_ref_dot):
        ''' Control algorithm to implement TTC
        
        Inputs: 
        -------------    
        x : np.array with shape [X, Y, psi]
            states of vehicle
        p_ref : np.array with shape [X_ref, Y_ref] 
            vehicle desired position in inertial frame
        p_ref_dot : np.array wtih shape [X_ref_dot, Y_ref_dot] 
            vehicle desired velocity in inertial frame

        Returns: 
        --------------
        u : np.array with shape [V, yaw-rate]
            control actions (velocity and yaw -rate)
        '''
        p = np.array([x[0], x[1]]) # current position
        psi = x[2]                 # current yaw-angle 
        err = p_ref-p              # tracking error
        
        '''
        *********** TO BE COMPLETED *************
        '''
        u = np.array([0,0])

        return u
    
    def control_io_steering(self, x, p_ref, p_ref_dot):
        '''
        Wrapper function for TTC controller
            1) Computes desired vehicle yaw-rate using control_io() 
            2) Converters yaw-rate into desired steering angle 
        Parameters
        ----------
        x : np.array with shape [X, Y, psi]
            states of vehicle
        p_ref : np.array with shape [X_ref, Y_ref] 
            vehicle desired position in inertial frame
        p_ref_dot : np.array wtih shape [X_ref_dot, Y_ref_dot] 
            vehicle desired velocity in inertial frame

        Returns: 
        --------------
        u : np.array with shape [V, steeringAngle]
            control actions (velocity and steering angle of front wheels)

        '''
        u=self.control_io( x, p_ref, p_ref_dot)
        
        V = u[0]
        u_yawrate = u[1]
        steer=np.arctan2(self.L*u_yawrate, V)
        return np.array([V,steer])        
 
    
    def myplotXY(self, x,p_ref,style):
        '''
        PLOT X vs Y position of the vehicle

        Parameters
        ----------
        x :  np.array with shape NSimSteps x 3 
            states of vehicle, [X, Y, psi]
        p_ref : np.array with shape NSimSteps x 2 
            vehicle desired position in inertial frame, [X_ref, Y_ref] 
        style : string
            line style for plotting

        Returns
        -------
        None.

        '''
        # plotting using matplotlib
        X = x[:,0]
        Y = x[:,1] 
        X_ref=p_ref[:,0]
        Y_ref=p_ref[:,1]
        
        plt.plot(X,Y,style)
        plt.plot(X_ref,Y_ref,style)
        
        plt.plot()
        plt.xlabel("X [m]")
        plt.ylabel("Y [m]")
        plt.show()


    def myplot3(self, x,p_ref, style, Tsample):
        '''
        PLOT X, Y and psi over time

        Parameters
        ----------
        x :  np.array with shape NSimSteps x 3 
            states of vehicle, [X, Y, psi]
        p_ref : np.array with shape NSimSteps x 2 
            vehicle desired position in inertial frame, [X_ref, Y_ref] 
        style : string
            line style for plotting
        Tsample: float
            sampling time
        Returns
        -------
        None.

        '''     
        Nsim =x.shape
        Nsim = Nsim[0]
        time = np.arange(0,Nsim)
        time= time * Tsample
        X  = x[:,0]
        Y = x[:,1]
        psi=x[:,2]
        X_ref = p_ref[:,0]
        Y_ref = p_ref[:,1]
        
        plt.subplot(311)
        plt.plot(time, X,style)   
        plt.plot(time, X_ref,style)   
        
        plt.ylabel('X[m]')    
        plt.subplot(312)    
        plt.plot(time,Y,style)
        plt.plot(time, Y_ref,style)   
    
        plt.ylabel('Y[m]')
        plt.subplot(313)    
        plt.plot(time, psi,style)    
        plt.ylabel('yaw-angle [rad]')
        plt.xlabel('time [s]')
        plt.show()
    
 

if __name__ == "__main__":
    #Simple test of TTC
    ttc= TrajectoryTracking()
    
    x  = np.array([10,0,0])
    p_ref = np.array([10,0])
    p_ref_dot = np.array([0,0])
    
    u=ttc.control_io(x, p_ref, p_ref_dot)
    
    print(u)