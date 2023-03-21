# -*- coding: utf-8 -*-
"""
ME290: Robotic Vehicles

Template for Homework #2 
Problem 1: Velocity Controller
"""

import numpy as np
import matplotlib.pyplot as plt
import math 
import control as ct  # MATLAB-like functions (https://python-control.readthedocs.io/)


class LQRcontrol:
    def __init__(self,Ad, Bd, Q, R):
        '''
        Initialize LQR Controller 
        
        System Model: x[k+1]=Ad x[k]+ Bd u[k]

        Cost: min sum x[k]Qx[k] + u[k]Ru[k]         
        Parameters
        ----------
        Ad : np array with shape  nx x nx
            System matrix
        Bd : np array with shape  nx x nu
            Input matrix.
        Q : np array with shape  nx x nx
            state weight that penalizes quadratic deviations from oring
        R : np array with shape  nu x nu
            input weight that penalizes control effort

        Returns
        -------
        None.

        '''
        
        # initialize system matrices
        self.Ad = Ad
        self.Bd = Bd        
        [nx, nu] = Bd.shape
        self.nu = nu
        
        # initialize weight matrices
        self.Q = Q
        self.R = R

        #************ TODO !!!!***************
        self.Klqr = np.zeros( (nu,nx) ) # LQR gain

    def control(self, x, vref, vref_d,vref_dd):
        '''
        implements LQR controller

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

        #************ TODO !!!!***************
        u = np.zeros( (1,self.nu) )
        return np.array([u])
    
    
class LongitudinalModel:    
    def __init__(self, m, tau, Fr):
        self.m = m     #[kg] vehicle mass
        self.tau = tau # [s] time constant of the powertrain  
        self.Fr  = Fr  # [N] constant resistive force
        self.Kp  = 0.9 # proportional gain of the controller
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
        v=x[0]
        Fm=x[1] # motor force
        x1_dot = (Fm-self.Fr)/self.m
        x2_dot = (u-Fm)/self.tau
        
        # build vector field
        x_dot = np.array([ x1_dot, x2_dot], dtype='float64')
        return x_dot    
    

    def build_v_ref(self, Nsimsteps, Tsample, vavg, omega_v ):
        '''
        Generate reference velocity  

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
        t_range = np.arange(0,Nsimsteps)*Tsample   
        v_ref = np.zeros((Nsimsteps,1)) # control input
        v_ref_d=np.zeros((Nsimsteps,1)) # control input
        v_ref_dd=np.zeros((Nsimsteps,1)) # control input

        v_ref[:,0] = vavg+np.sin(omega_v*t_range)
        v_ref_d[:,0] = -omega_v*np.cos(omega_v*t_range)
        v_ref_dd[:,0] = omega_v*omega_v*np.sin(omega_v*t_range)
        
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
            u[k,:] = u_k    
            # Euler discretization
            x_k = x_old + Tsample*self.f_dot(x_old, u_k)
            #-------------
            x[k,:] = x_k     # save state    
        return (x,u);
    
    
    def plotResults(self,x ,v_ref,u, Tsample):
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
        Tsample: float
            sampling time
        Returns
        -------
        None.
        '''
        [N,nu]=u.shape
        time = np.arange(N)*Tsample
        
        plt.subplot(311)
        plt.plot(time, x[:,0])
        plt.plot(time, v_ref)
        plt.ylabel('v [m/s]')
        plt.subplot(312)
        plt.plot(time, u)
        plt.ylabel('u [N]')
    
        plt.subplot(313)
        plt.plot(time, x[:,0]-v_ref[:,0])
        plt.ylabel('err [m/s]')     
        plt.xlabel('time [s]')
        
if __name__ == "__main__":

    # Parameters
    Tsample = 0.01   # [s] sampling time for discretization of vehicle model   
    m   = 1   #[kg] vehicle mass
    tau = 1.5 # [s] time constant of the powertrain  
    Fr  = 0   # [N] constant resistive force
    
    model=LongitudinalModel(m, tau, Fr)
 
    # State-space model
    Ad = np.array([[1, Tsample/m,],
                   [0, 1-Tsample/tau]])
    Bd = np.array([[0], [Tsample/tau]])
    
    # LQR Controller weigths
    Q = 1*np.array([[1, 0],
                   [0, 1]])
    R = 200.1
        
    ctrlLQR = LQRcontrol(Ad, Bd, Q, R)
    
    # ***** Simulation params *****       
    x0 = np.array([0, 0]) # initial state [v, Force]
    Tsim    = 30     # [s] simulation time
    Nsimsteps = round(Tsim/Tsample); # number of simulation steps

    # ***** REFERENCE VELOCITY *****        
    vavg_Ref = 10 # [m/s] average reference velocity
    omega_v = 0.0 # [rad/s] frequency of oscillations of reference velocity 
    (v_ref, v_ref_d, v_ref_dd)=model.build_v_ref(Nsimsteps, Tsample, vavg_Ref, omega_v)

    # ***** SIMULATE RESULTS *****            
    (x,u)=model.simulate_control_euler( x0, ctrlLQR.control, v_ref, v_ref_d, v_ref_dd, Tsample) # simulate
    
    # ***** PLOT RESULTS *****    
    model.plotResults(x, v_ref, u,  Tsample)
    