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
    DirectCollocationConstraint, 
    TimeStep,
)

class LQRTree(Object): 



	def __init__(self, cost_fun, plant):
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

	self.plant = plant 
	self.context =  self.plant.CreateDefaultContext()
	self.cost_fun = cost_fun


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

	# Setup direct collocation trajectory opitmization 
	dircol = DirectCollocation(
            self.plant, self.context, num_time_samples=21, minimum_timestep=0.2,
            maximum_timestep=0.5,
            input_port_index=InputPortSelection.kUseFirstInputIfItExists,
            assume_non_continuous_states_are_fixed=False)

	dircol.AddDurationBounds(lower_bound=0.5, upper_bound=3.0)
	
	#make start state equal to start point 
	dircol.AddLinearConstraint(dircol.initial_state() == start_pt)
	
	#make end state equal to end pt 
	dircol.AddLinearConstraint(dircol.final_state()   == end_pt)

	#construct collocation constraints
	constraint = DirectCollocationConstraint(plant, context)	
	AddDirectCollocationConstraint(constraint, dircol.timestep(0),
	                                      dircol.state(0), dircol.state(1),
	                                      dircol.input(0), dircol.input(1),
	                                      dircol)



	#Trajectory optimization requires an initial guess, even if it's not very good. 
    initial_u = PiecewisePolynomial.ZeroOrderHold([0, .3*21],
                                                   np.zeros((1, 2)))
    initial_x = PiecewisePolynomial()
    
    dircol.SetInitialTrajectory(traj_init_u=initial_u,
                                    traj_init_x=initial_x)


    dircol.AddRunningCost(self.cost_fun)


    result = mp.Solve(dircol)


    input_traj = dircol.ReconstructInputTrajectory(result=result)
    
    state_traj = dircol.ReconstructStateTrajectory(result=result)


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



