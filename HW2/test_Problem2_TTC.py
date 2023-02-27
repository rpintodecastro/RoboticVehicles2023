# -*- coding: utf-8 -*-

"""
ME290: Robotic Vehicles

Template for Homework #2 
Problem 2: Trajectory Tracking Controller

"""

import numpy as np
import matplotlib.pyplot as plt
import MyODEsim # Numerical solver for differential equation

import KSTM     # Kinematic Vehicle Model
import TTC      # Trajectory Tracking Controller
import RefGeneration # Trajectory Reference Generation

# ****** BUILD Kinematic Vehicle Model and Trajectory Controller**********
kstm = KSTM.KSTModel()
ttc = TTC.TrajectoryTracking()

# ******Define parameters of vehicle ********
L = 1.22+1.18;   # wheel base [m]
lx = 1;          # distance of reference point    
kstm.L=L
kstm.lx=lx
ttc.L = L
ttc.lx = lx

# ********** initial position and orientation   *******
x0_pos= np.array( [0, 5, 0]);    # initial state [X, Y, psi]

# ********** simulation parameters ************
Tsample = 0.01 # [s] sampling time for discretization of vehicle model   
Tsim   =  200  # [s] simulation time
Nsimsteps = round(Tsim/Tsample); # number of simulation steps

# create reference trajectory
ref=RefGeneration.RefTrajector()
(p_ref,p_ref_dot) = ref.constant_point(X_ref=10, Y_ref=0, Nsimsteps=Nsimsteps, Tsample=Tsample)
#(p_ref,p_ref_dot) = ref.straight_line(X_ref=10, Y_ref=0, Nsimsteps=Nsimsteps, Tsample=Tsample)
#(p_ref,p_ref_dot) = ref.slalom(X_ref=0, Y_ref=0, Nsimsteps=Nsimsteps, Tsample=Tsample)
#(p_ref,p_ref_dot) = ref.circle(X_ref=0, Y_ref=0, Nsimsteps=Nsimsteps, Tsample=Tsample)

# Simulate KSTM and Trajectory Tracking Controller
sim = MyODEsim.Sim()
(x_kstm, u_kstm)=sim.simulate_control_euler(kstm.f_dot, x0_pos, 
                    ttc.control_io_steering, p_ref, p_ref_dot, Tsample) # simulate

# ******** PLOT RESULTS **********
plt.figure(dpi=300)
ttc.myplotXY(x_kstm, p_ref,'-') # plot X vs Y
ttc.myplot3(x_kstm,p_ref,'-', Tsample) # plot X,Y, psi
