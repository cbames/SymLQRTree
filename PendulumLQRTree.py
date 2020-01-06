""" 

This builds the LQRTree for the pendulum and the runs the resulting controller. 

Authors: Barrett Ames and Jiayue Fan 
Last Updated: Jan 2020 
"""

from pydrake.examples.pendulum import PendulumPlant
import numpy as np 



def cost(x,u): 
	dx = x 
	dx[0] = x[0] -np.pi 

	g = dx.T*Q*dx + u.T*R*u 
	dgdt = 0 
	dgdx = 2*dx.T*Q
	dgdu = 2*u.T*R
    dg = [dgdt, dgdx, dgdu]

    return [g,dg]