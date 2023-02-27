# -*- coding: utf-8 -*-
"""
ME290: Robotic Vehicles

Template for Homework #2 
Problem 1: Velocity Controller
"""

import numpy as np
import matplotlib.pyplot as plt
import math 


class LongitudinalModel:    
    def __init__(self):
        # default parameters in the problem
        self.m = 1000      # [kg] vehicle mass
        self.tau = 1.5     # [s] time constant of the powertrain  
        self.Fr  = 100     # [N] constant resistive force
        self.Kp  = 0.9     # proportional gain of the controller
        self.u_rate = 1000 # [N/s] maximum rate of charge for motor force 
        
    def f_dot(self, x, u):
        '''
        implement right hand side of differential equation
            x_dot = f_dot(x,u)
            
        Parameters
        ----------
        x : np.array with shape 1x2
            states: x=[velocity, Force]
        u : float
            desired motor force [N]

        Returns
        -------
        x_dot : np.array with shape 1x2
            time derivative of [velocity, Force]

        '''
        v=x[0]  # current vehicle velocity [m/s]
        Fm=x[1] # current motor force [N]
        
        '''
        *********** TO BE COMPLETED *************
        '''
        x1_dot = 0
        x2_dot = 0
        '''
        *********** *************
        '''

        # build vector field
        x_dot = np.array([ x1_dot, x2_dot], dtype='float64')
        return x_dot    
    
    def control(self, x, vref, vref_d,vref_dd):
        '''
        implements velocity controller algorithm

        Parameters
        ----------
        x : np.array with shape 1x2
            states: x=[velocity, Force]
        vref : float
           reference velocity [m/s]
        vref_d : float
           reference acceleration [m/s2]
        vref_dd : float
           reference jerk [m/s3]

        Returns
        -------
        u : float
            desired motor force [N]

        '''
        kp=self.Kp # proportional gain
        v=x[0]     # load initial velocity
        m=self.m   # load vehicle mass
        
        '''
        *********** TO BE COMPLETED *************
        '''        
        u = 0
        '''
        ************************
        '''
                
        return np.array([u])

    def build_v_ref(self, Nsimsteps, Tsample, vavg, omega_v ):
        '''
        Generate reference velocity for 

        Parameters
        ----------
        Nsimsteps : int
            number of simulation time steps
        Tsample : float
            sampling time of the simulation [s]
        vavg : float
            desired average vehicle velocity  [m/s]
        omega_v : float
            frequency of the oscillation in ref. velocity [rad/s]

        Returns
        -------
        v_ref : np.array with shape Nsimstepsx1
           reference velocity [m/s]
        v_ref_d : np.array with shape Nsimstepsx1
            reference acceleration [m/s2]
        v_ref_dd : np.array with shape Nsimstepsx1
            reference jerk [m/s3].

        '''
        '''
        *********** TO BE COMPLETED *************
        '''  
        v_ref    = np.zeros((Nsimsteps,1)) # control input
        v_ref_d  = np.zeros((Nsimsteps,1)) # control input
        v_ref_dd = np.zeros((Nsimsteps,1)) # control input
        '''
        ************************
        '''  
        
        return (v_ref, v_ref_d, v_ref_dd)
        
    def simulate_control_euler(self, x0, u_fun, v_ref, v_ref_dot,v_ref_dd, Tsample):
        """
        Simulate system dynamics 
            x_dot = f_dot(x,u)
            
        Inputs:
        ----------
        f_dot:  function handler
            function pointer to f_dot(x,u)
        x0 : np.array with shape 1xNSTATES
            initial states 
        v_ref : np.array with shape Nsimstepsx1
           reference velocity [m/s]
        v_ref_dot : np.array with shape Nsimstepsx1
            reference acceleration [m/s2]
        v_ref_dd : np.array with shape Nsimstepsx1
            reference jerk [m/s3].
        Tsample : float
            simulaton sample time [s]
    
        Returns
        -------
        x  : np.array  with shape  NUM_SIM_STEPS x nSTATES
            simulation states
        u  : np.array  with shape  NUM_SIM_STEPS x nINPUTS
            control inputs generated during simulation
        """
        n_states = x0.size # number of states
        (N,n_p) = v_ref.shape         # number of simulation steps
    
        x = np.zeros((N, n_states))      # create state vector over simulation horizon
        x[0,:] = x0 # set initial state
        
        # check control function
        u_tmp = u_fun(x0, v_ref[0,:], v_ref_dot[0,:], v_ref_dd[0,:])
        n_u = u_tmp.size   
        
        u =  np.zeros((N, n_u)) 
        # simulate 
        for k in range(1,N):
            x_old = x[k-1,:]  # extract state in previous time state
            u_old = u[k-1,:] # extract previous control action

            u_k = u_fun(x_old, v_ref[k-1,:], v_ref_dot[k-1,:],v_ref_dd[k-1,:])
            
            # enforce rate limit
            u_k = self.rate_limit(u_k, u_old, Tsample)
            u[k,:] = u_k    
            # Euler discretization
            x_k = x_old + Tsample*self.f_dot(x_old, u_k)
            #-------------
            x[k,:] = x_k     # save state    
        return (x,u);

    def rate_limit(self, u, u_old, Tsample):
        '''
        Implements rate limit on control input u:
          -u_rate_max <=  d/dt u <= u_rate_max

        Parameters
        ----------
        u : float
            current control input
        u_old : float
            control input in previous time step
        Tsample : float
            sampling time

        Returns
        -------
        u_out : float
            control input complient with rate limit

        '''
        u_rate_max = self.u_rate_max
        

        '''
        *********** TO BE COMPLETED *************
        '''                
        u_out = u
        '''
        ************************
        '''                
        return u_out
    
    def plotResults(self,x ,v_ref,u):
        '''
        Plot Simulation results

        Parameters
        ----------
        x  : np.array  with shape  NUM_SIM_STEPS x nSTATES
            vehicle states
        v_ref np.array  with shape  NUM_SIM_STEPS x 1
            reference velocity
        u  : np.array  with shape  NUM_SIM_STEPS x nINPUTS
            control inputs generated during simulation

        Returns
        -------
        None.
        '''
        
        
        '''
        *********** TO BE COMPLETED *************
        '''                

    
    
if __name__ == "__main__":
   
    model=LongitudinalModel()

    # ***** DEFINE CONTROL PARAMETERS *****    
    model.Kp = 0.9         # proportional gain of the controller
    model.u_rate_max = 500 # [N/s] rate limit of engine force
    
    x0 = np.array([10, 0]) # initial state [v, Force]
    Tsample = 0.01         # [s] sampling time for discretization of vehicle model   
    Tsim    = 30           # [s] simulation time
    Nsimsteps = round(Tsim/Tsample); # number of simulation steps

    # ***** REFERENCE VELOCITY *****        
    vavg_Ref = 15 # [m/s] average reference velocity
    omega_v = 0.0 # [rad/s] frequency of oscillations of reference velocity 
    (v_ref, v_ref_d, v_ref_dd)=model.build_v_ref(Nsimsteps, Tsample, vavg_Ref, omega_v)

    # ***** SIMULATE RESULTS *****            
    (x,u)=model.simulate_control_euler( x0, model.control, v_ref, v_ref_d, v_ref_dd, Tsample) # simulate
    
    # ***** PLOT RESULTS *****   
    model.plotResults(x, v_ref, u)
    