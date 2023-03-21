# -*- coding: utf-8 -*-
"""
Created on Sun Feb 26 11:18:07 2023

Linear Quadratic Regulator (Example)
@author: ricardo de castro
"""

import matplotlib.pyplot as plt   # MATLAB plotting functions
import control as ct  # MATLAB-like functions (# MATLAB-like functions (https://python-control.readthedocs.io/))

import numpy as np
# Parameters defining the system
Ts = 0.1    # [s] sample time
m = 1.0           # system mass

# System matrices
Ad = np.array([[1, Ts], [0, 1]])
Bd = np.array([[0], [Ts/m]])
nstates=2
Cd = np.eye(nstates)
discsys = ct.ss(Ad, Bd, Cd, 0, dt=Ts)
print(discsys)
#sys = ss(A, B, C, 0)

# LQR feedback gain
Q  = np.eye(nstates)
R  = 00.1
P_ricc, L_eign, K = ct.dare(Ad, Bd, Q, R)
print(K)

discsys_cl = ct.ss(Ad-Bd@K, [[0],[0]], Cd, 0, dt=Ts)

#res = ct.step_response(discsys)
Tsim =15
X0 = [10, 0]
res = ct.initial_response(discsys_cl, Tsim, X0)

plt.figure(1)
#plt.plot(Tx.T, Yx.T)
plt.subplot(311)
plt.plot(res.outputs[0].T)
plt.ylabel('x1 [m]')
plt.ylim(top=11, bottom=-3)

plt.grid()
plt.subplot(312)
plt.plot(res.outputs[1].T)
plt.ylabel('x2 [m/s]')
plt.ylim(top=1, bottom=-6)
plt.grid()
plt.subplot(313)
u_LQR = -K@res.outputs
plt.plot(u_LQR.T)
plt.ylabel('u [N]')
plt.xlabel('time [s]')
plt.ylim(top=4, bottom=-6)
plt.grid()
#plt.plot(T.T, yout.T)
#plt.show(block=False)



plt.show()