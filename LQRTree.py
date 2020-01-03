""" 

Here we're implementing LQR-Trees using sampling based analysis for Region of Attraction. 
We'll also experiment with exploiting symmetry in the value function and/or the dynamics 
We should make this a dynamics agnostic solution


Authors: Barrett Ames and Jiayue Fan 
Last Updated: Jan 2020 
"""

import math
import warnings

import numpy as np

from pydrake.trajectories import PiecewisePolynomial
from pydrake.solvers import mathematicalprogram as mp
from pydrake.systems.framework import InputPortSelection
from pydrake.systems.primitives import LinearSystem
from pydrake.systems.trajectory_optimization import (
    AddDirectCollocationConstraint, DirectCollocation,
    DirectCollocationConstraint, DirectTranscription,
    TimeStep,
)

class LQRTree(Object): 



	def __init__(self):
	"""
	
	There are going to be  a lot of parameters that get set here. 

	dynamics: 		the plant that the tree will be constructed for 
	input_bounds:   upper and lower bounds on the control inputs 
	state_bounds:   bounds on the state space that we care about 
	num_nearest_neighbors: The number of nearest neighbors to consider 
	time_delta: the time descretization for trajectory 
	Q: state cost matrix 
	R: action cost matrix 
	V_err: amount of error we're willing to tolerate in the value function 
	goal_point: the goal of the LQR tree 

	"""


	def constructTree(): 
	"""
	1) build Time Invariant LQR solution around linearized goal point 
	2) Set bound on acceptable value function error 
	3) Randomly sample new point
	4) Attempt to connect new point to goal point using treeConnect()
		4.a) if successful construct funnel around trajectory 
		4.b) if not succesful go back to 3 
	5) Probabilistically test all of the funnels,
	if there is a high probability that a random sample will land 
	in a valid funnel, stop go to 2 and set a lower bound. repeat process until desired lower bound is achieved. 
	
	"""




	def treeConnect(self, start_pt, end_pt): 
	"""
	Use trajectory optimization to connect a sampled point to a nearest neighbor on the tree. 
	"""




	def funnelConstruction(): 
	"""
	Use TVLQR to construct a Region of Attraction around the trajectory
	"""


	def ActionValue():
	"""
	This returns the action value of a point based on a particular funnel.  
	"""


	def saveLQRTree(): 
	"""
	Save the LQR Tree using pickle so that it can be used later. 
	"""

	def loadLQRTree(): 
	""" 
	load a pickled tree so that tree construction can be bypassed
	"""


	def runLQRTreeController(): 
    """ 
    Must have a loaded tree from either the construct tree or LoadLQRTree functions. 
    """



