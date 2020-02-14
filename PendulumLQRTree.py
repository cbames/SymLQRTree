""" 

This builds the LQRTree for the pendulum and the runs the resulting controller. 

Authors: Barrett Ames and Jiayue Fan 
Last Updated: Jan 2020 
"""

from pydrake.examples.pendulum import (PendulumPlant, PendulumState)
import numpy as np 
from LQRTree import LQRTree
from util.utils import PendulumItem
import collections


def cost(x,u): 
	dx = x 
	dx[0] = x[0] -np.pi 

	g = dx.T*Q*dx + u.T*R*u 
	dgdt = 0 
	dgdx = 2*dx.T*Q
	dgdu = 2*u.T*R
    dg = [dgdt, dgdx, dgdu]

    return [g,dg]


if __name__ == '__main__':
	plant = PendulumPlant()
	goal_point=np.array([2,0]) # TODO: This doesn't seem right 
	dt=.05
	bounds=np.array([0,1])# TODO: This doesn't seem right 
	Point= PendulumItem
	acceptableError = 180
	tree=LQRTree(Point,1,goal_point,dt,plant,bounds,bounds,acceptableError )