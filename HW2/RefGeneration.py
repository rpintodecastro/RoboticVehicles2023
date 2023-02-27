# -*- coding: utf-8 -*-
"""
Created on Fri Feb 24 10:09:41 2023

ME290: Robotic Vehicles

Template for Homework #2 

Implements Trajectory Reference Generation
"""


import numpy as np
import matplotlib.pyplot as plt
import math 

class RefTrajector:
    
    def constant_point(self, X_ref, Y_ref, Nsimsteps, Tsample):
        '''
        

        Parameters
        ----------
        X_ref : float
            Desired X position
        Y_ref : fload
            desired Y position

        Returns
        -------
        p_ref : np.array 
            reference position with shape NUM_SIM_STEPS x 2 
        p_ref_dot: np.array
            reference velocity with shape NUM_SIM_STEPS x 2 

        '''
        p_ref = np.zeros((Nsimsteps,2))  
        p_ref[:] = [X_ref,Y_ref]
        p_ref_dot = np.zeros((Nsimsteps,2)) # control input
        
        return (p_ref,p_ref_dot) 

    def straight_line(self, X_ref, Y_ref, Nsimsteps, Tsample):
        '''
        

        Parameters
        ----------
        X_ref : float
            initial X position
        Y_ref : fload
            initial Y position

        Returns
        -------
        p_ref : np.array 
            reference position with shape NUM_SIM_STEPS x 2 
        p_ref_dot: np.array
            reference velocity with shape NUM_SIM_STEPS x 2 

        '''        
        Vref = 5 # [m/s] reference velocity
        t_range = np.arange(0,Nsimsteps)*Tsample            

        #****************************************
        # TO BE COMPLETED            
        #****************************************
        p_ref = np.zeros((Nsimsteps,2))  
        p_ref_dot = np.zeros((Nsimsteps,2))  
        
        return (p_ref,p_ref_dot) 

    def slalom(self, X_ref, Y_ref, Nsimsteps, Tsample):
        '''
        

        Parameters
        ----------
        X_ref : float
            initial X position
        Y_ref : fload
            initial Y position

        Returns
        -------
        p_ref : np.array 
            reference position with shape NUM_SIM_STEPS x 2 
        p_ref_dot: np.array
            reference velocity with shape NUM_SIM_STEPS x 2 

        '''        
        t_range = np.arange(0,Nsimsteps)*Tsample

        #****************************************
        # TO BE COMPLETED            
        #****************************************
        p_ref = np.zeros((Nsimsteps,2))  
        p_ref_dot = np.zeros((Nsimsteps,2))  
        
        return (p_ref,p_ref_dot)         

    def circle(self, X_ref, Y_ref, Nsimsteps, Tsample):
        '''
        

        Parameters
        ----------
        X_ref : float
            initial X position
        Y_ref : fload
            initial Y position

        Returns
        -------
        p_ref : np.array 
            reference position with shape NUM_SIM_STEPS x 2 
        p_ref_dot: np.array
            reference velocity with shape NUM_SIM_STEPS x 2 

        '''        
         
        t_range = np.arange(0,Nsimsteps)*Tsample

        #****************************************
        # TO BE COMPLETED            
        #****************************************
        p_ref = np.zeros((Nsimsteps,2))  
        p_ref_dot = np.zeros((Nsimsteps,2))  
        
        return (p_ref,p_ref_dot)       
    
if __name__ == "__main__":
    #....
    refTraj= RefTrajector()

    X_ref = 10
    Y_ref = 0
    Nsimsteps = 100
    Tsample = 1
    (p_ref, p_ref_dot) = refTraj.constant_point(X_ref, Y_ref, Nsimsteps, Tsample)
    #(p_ref, p_ref_dot) = refTraj.straight_line(X_ref, Y_ref, Nsimsteps, Tsample)
    #(p_ref, p_ref_dot) = refTraj.slalom(X_ref, Y_ref, Nsimsteps, Tsample)
    #(p_ref, p_ref_dot) = refTraj.circle(X_ref, Y_ref, Nsimsteps, Tsample)
    
    
    
    print(p_ref)
    
    plt.subplot(411)
    plt.plot(p_ref[:,0])
    plt.ylabel('X*')
    
    plt.subplot(412)
    plt.plot(p_ref[:,1])
    plt.ylabel('Y*')

    plt.subplot(413)
    plt.plot(p_ref_dot[:,0])
    plt.ylabel('d/dt X*')
    
    plt.subplot(414)
    plt.plot(p_ref_dot[:,1])
    plt.ylabel('d/dt Y*')
    