""" 

Here we're implementing LQR-Trees using sampling based analysis for Region of Attraction. 
We'll also experiment with exploiting symmetry in the value function and/or the dynamics 
We should make this a dynamics agnostic solution


Authors: Barrett Ames, Jiayue Fan, Yao Yuan
Last Updated: Jan 2020 
"""

import math
import warnings

import numpy as np

from pydrake.trajectories import PiecewisePolynomial
from pydrake.solvers import mathematicalprogram as mp
from pydrake.systems.framework import InputPortSelection, OutputPortSelection, Context
from pydrake.systems.primitives import LinearSystem, FirstOrderTaylorApproximation
from pydrake.systems.trajectory_optimization import (
    AddDirectCollocationConstraint, DirectCollocation,
    DirectCollocationConstraint, 
    TimeStep
)
import networkx as nx

from pydrake.systems.controllers import LinearQuadraticRegulator

class LQRTree(Object): 
"""
       function epsH = findGoalBasinSim()
           disp('Approximating goal basin of attraction...')
           tgoal = tic;
           epsH = Inf;
           % determine success limit
           termCond = ceil(log(goal.basin.termCond.alpha)...
               /log(goal.basin.termCond.pAlpha));
           % determine eigenvalues and eigenvectors of goal.Stilqr
           [L,ev] = eig(goal.Stilqr);
           ev = diag(ev);
           % init state that controls process of first estimating epsH and
           % then estimating the success rate, given epsH
           estimate = 2;
           
           while(estimate > 0)
               % if estimate = 2, get initial estimate epsH
               if(estimate > 1)
                   maxIter = options.maxIter;
               else
                   % if estimate = 1, estimate rate of samples for which
                   % Lyapunov condition holds. Use nEvals samples.
                   maxIter = goal.basin.nEvals;
                   % make sure term cond does not kick in
                   termCond = goal.basin.nEvals; 
                   disp(['Found epsH = ',num2str(epsH),' in ',...
                       num2str(toc(tgoal)),'s.']);
                   disp(['Now evaluating success rate with ',...
                       int2str(maxIter),' samples']);
               end
               nGoalSuccess = 0;
               iter = 0;
               while(nGoalSuccess < termCond && iter < maxIter)
                   iter = iter + 1;
                   % get samples until a sample is inside the ellipse
                   inside = 0;
                   while(~inside)
                       if(isinf(epsH))
                           x = getRandSample(xRange);
                           dx = (x-goal.x);
                           ji = dx'*goal.Stilqr*dx;
                           inside = ji < epsH;
                       else
                           lim = sqrt(epsH./ev);
                           X = -lim + rand(nX,1).*2.*lim;
                           inside = sum(X.^2.*ev) < epsH;
                           if(inside)
                               % calculate x
                               x = L*X + goal.x;
                               % calculate dx and check if x is inside of ellipse
                               % with radius epsH
                               dx = x - goal.x;
                               ji = dx'*goal.Stilqr*dx;
                               inside = ji < epsH;
                           end
                       end
                   end

                   if(testConstraints(x))
                       % get control input for a single step
                       u = goal.u - tK(1,:)*dx;
                       % apply input constraints
                       if(~isempty(options.uLimit))
                           u = min([u,options.uLimit(:,2)],[],2);
                           u = max([u,options.uLimit(:,1)],[],2);
                       end
                       % simulate system
                       [~,xtape] = ode45(@(t,x)PLANTFUN(t,x,u),[0,tDt(1)],x);
                       x = xtape(end,:)';
                       % evaluate Lyapunov function    
                       if((x-goal.x)'*goal.Stilqr*(x-goal.x) - ji < 0)
                           % decreasing, increase success count
                           nGoalSuccess = nGoalSuccess + 1;
                       else
                           % increasing, shrink epsH; do this also while
                           % estimating success rate, since it's good to have
                           % a conservative estimate
                           epsH = ji;
                           disp(['Lyapunov increasing by ',num2str((x-goal.x)'*goal.Stilqr*(x-goal.x) - ji),', shrunk goal basin epsH to ',num2str(epsH)])
                           % only reset success counter during initial estim.
                           nGoalSuccess = (estimate < 2) * nGoalSuccess;
                       end
                   else
                       % if state constraints are violated:
                       epsH = ji;
                       disp(['State constraints violated, shrunk goal basin epsH to ',num2str(epsH)])
                       % only reset success counter during initial estim.
                       nGoalSuccess = (estimate < 2) * nGoalSuccess;
                   end
               end
               % go from initial estimate to success rate estimate to done
               estimate = estimate - 1;
           end
           % make sure the goal is not almost just a point:
           assert( epsH > eps,...
               'Goal ellipse shrunk to zero, double check parameters');
           disp(['Approximated a goal set with epsH = ',...
               num2str(epsH)])
           % calculate success rate to display:
           [p,pci] = binofit(nGoalSuccess, maxIter);
           disp(['With ', int2str(maxIter),' samples from Goal basin, ',...
               'the success rate is at least ',num2str(p),...
               ' with confidence int. ', num2str(pci)]);
           if(options.promptUserAfterGoalApprox)
               input('Press the Return key to start generating the tree policy');        
           end
       end"""


    def findGoalBasinSim(): 
    """
    Determines the goal set, G, by approximating the invariant
    set of the closed-loop goal-state dynamics, see Sec. 5.3 of Reist
    """ 
        epsH = np.inf 


        return epsH

    def addNode(self, K, S, A, B, epsH, Vx, Qval, X0, T, U0)
    """
    the following code add a node with index/label = node_cnt,
    along with attributes for this node, like K,S,epsH
    """
        self.G.add_node(self.node_cnt,
                        K=K,
                        S=S,
                        A=A,
                        B=B,
                        epsH=epsH,
                        Vx=Vx, #TODO: Do we need to store Vx here?
                        Qval=Qval, 
                        X0=X0,
                        T=T,
                        U0=U0)
        # always add 1 to node cnt when adding nodes
        self.node_cnt=self.node_cnt+1


    def __init__(self, state_dim, action_dim, cost_fun, plant, input_bounds, state_bounds, acceptable_error):
    """
    
    There are going to be  a lot of parameters that get set here. 

    dynamics:       the plant that the tree will be constructed for 
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
    self.goal_point = goal_point 
    self.InputLimit = InputLimit
    self.acceptable_error = acceptable_error

    self.nX=state_dim
    self.nU=action_dim
    self.plant = plant 
    self.context =  self.plant.CreateDefaultContext()

    self.goal_point = goal_point 
    self.input_bounds=input_bounds
    self.state_bounds=state_bounds
    self.dt=dt
    
    self.node_cnt=0# stores how many nodes has been added to the graph,
                    # like the idx in the previous code
                    # the next node to add has index = node_cnt
    
    
    # init other variables
    self.G=nx.DiGraph()# node index serves as the key to access node from graph
    self.Q=np.eye(nX)*2#2 is user defined
    self.R=np.eye(nU)*5#5 is user defined
    self.nNodesInit = 10000


    # Construct LQR around goal point 
    context_goal=self.plant.CreateDefaultContext()
    context_goal.SetContinuousState(self.goal_point)#what's set discrete state
    self.plant.get_input_port(0).FixValue(context_goal, 0)#if input port not set, it doesn't seem the taylor approx can run, the number of ports vary plants by plants
    affine_sys_goal=FirstOrderTaylorApproximation(self.plant,context_goal)
    K_goal,S_goal = LinearQuadraticRegulator(affine_sys_goal.A(), affine_sys_goal.B(), self.Q, self.R)
    
    #Bound region around goal point 
    epsH = findGoalBasinSim()

    #Add Goal to 
    addNode(K_goal[0], S_goal[0], affine_sys_goal.A(), affine_sys_goal.B(), epsH,0,0,self.goal_point, 0,0)

    def test_goal(self, x_sample):
    """
    Function to test whether a point is in the goal set
    """
        dx = x - goal
        value = dx.T*self.G.nodes[0].S*dx
        inside = value < self.G.nodes[0].epsH
        return inside 


    def test_sample(self, x_sample):
    """
        Function to test whether a random sample is contained within 
        a funnel that we've already defined 
    """

        inGoal = 
            %check if sample is already in final basin:
            [inGoal] = testGoal(xSample);
            if(inGoal)
                init = [];   % return empty motion planning init trajectory
                flag = 3;    % if xSample in goal, return 3
                return
            end
            
            % initialize success state and test result flag
            success = -1;   % needed in case all nodes whose funnels contain
                            % the sample are tested (simAllNodes in options)
            flag = -1;
            
            % check if sample is within any funnel in the tree, and also get
            % closest node to xSample in tree if it's not inside
            [inside, closest] = inBasin(xSample, tX0, tS, tEpsH, tIdx);
            
            % if xSample is in any funnel, start simulation(s)
            if(inside)
                % get distances from sample to all tree node states:
                ji = getC2G(xSample, tX0, tS, tIdx);
                % ignore goal node
                ji = ji(2:end);
                % check distances to find what node funnels the sample is in
                drji = tEpsH(2:tIdx) - ji; % drji > 0 if inside
                % init node index array
                inds = 2:tIdx;
                % get indices of nodes where the sample is within funnel desc.
                lind = drji > 0;
                % get relevant arrays of distances and indices
                inds = inds(lind);
                ji = ji(lind);
                % sort by distances, then start simulating in ascending
                % order (lowest distance first (= N* in paper)).
                [~,ind] = sort(ji,'ascend');
                jbest = inf; % init variable to keep track best init traj.
                i = 1;  % init index to go through all possible node policies
                while(i <= length(ind)) % go through all nodes
                    % if node is outside funnel due to earlier sim, continue
                    if((ji(ind(i)) - tEpsH(inds(ind(i)))) >= 0)
                        i = i + 1;
                        continue;
                    end
                    
                    %simulate using the policy of current N*
                    [xtape,utape,tsim] = simTree(inds(ind(i)));
                    
                    %check if simulation is successful
                    stateConstraintsOK = true;
                    [inGoal] = testGoal(xtape(:,end));
                    %if the sample ended up in the goal,
                    % check if state constraints were violated:
                    if(inGoal)
                        stateConstraintsOK = testConstraints(xtape);
                    end
                    
                    if(inGoal && stateConstraintsOK)
                        % only set flag the first time a successful sim is
                        % encountered (needed in case simAllNodes is set)
                        if(success < 0)
                            %we're good, set success
                            success = 1 + (i == 1);
                            % return 2 if the first node was successful
                            % and 1 if it was a lter node
                            if(~options.simAllNodes)
                                % return if not all nodes whose funnel contains
                                % the sample should be tested
                                flag = success;
                                init = [];
                                return
                            end
                        end
                    else
                        % Failure, adjust epsH
                        adjustFunnels(xtape,inds(ind(i)));  % adjust funnels
                        
                        % store the trajectory for initializing motion-planning
                        cost = getCost(xtape,utape,tsim);   % get cost of traj.
                        % if the failed trajectory has the lowest cost
                        % heuristic so far, store it
                        if(cost < jbest)
                            init.xtape = xtape;
                            init.utape = utape;
                            init.tsim = tsim;
                            jbest = cost;
                        end
                    end
                    i = i+1;
                end
            else
                % if the sample is not in any funnel, obtain an initialization
                % trajectory using the closest node's feedback policy
                flag = -2;
                % simulate from closest node to get initial condition for
                % dircol, only if tree has more than one node (=goal), and
                % closest node is not goal node
                if(tIdx > 1 && closest > 1)
                    [xsim,usim,tsim] = simTree(closest);
                    init.xtape = xsim;
                    init.utape = usim;
                    init.tsim = tsim;
                else
                    init = [];   % return empty motion planning init trajectory
                end
            end
            % in case that simAllNodes is set, make sure flag is set before
            % function returns
            if(success > 0)
                flag = success;
                init = [];
            end
        end


        return flag
    
    def valueApprox(self, x_sample):
        value_estimate = float('inf')
        for node in range(self.Idx):
            dx0 = x_sample - X0[:,node]
            J = np.matmul(np.matmul(dx0,self.Q),dx0)+np.matmul(np.matmul(uk,self.R),uk)
            if (J+Qval[node]) < value_estimate:
                value_estimate = J+Qval[node]
        return value_estimate
        
    def getRandSample(self):
        x_random = zeros(self.nX)
        for i in range(self.nX):
            x_random[i] = random.uniform(self.input_bounds[i,0],self.input_bounds[i,1])
        return x_random


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

    #remeber to add goal node first
    #add_goal_node(G,goal_node)

        num_rl_iter=15
        nSuccess=0
        nSuccTerm=459
        Q_max=8000

        # for loop, number of full coverage iterations
        for rl_step in range(num_rl_iter):


            # while loop, if there are enough coverage or too many iterations
            while nSuccess < nSuccTerm and Q_max > self.acceptable_error:
                # generate a random sample in the range and 
                x_sample=getRandSample()

                # test if this sample is covered (Barrett)
                flag=test_sample(x_sample)

                # if covered, continue
                if flag==1:
                    nSuccess=nSuccess+1
                    continue

                # else, add a new trajectory from this random sample to existing tree
                add_traj_from_sample(x_sample)
        
            # end while loop
            Q_max = Q_max*0.8#redundant with for loop?



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

        #Input Constraint 
        u = dircol.input();
        dircol.AddConstraintToAllKnotPoints(-self.InputLimit <= u(0));
        dircol.AddConstraintToAllKnotPoints(u(0) <= self.InputLimit);


        #Trajectory optimization requires an initial guess, even if it's not very good. 
        initial_u = PiecewisePolynomial.ZeroOrderHold([0, .3*21],
                                                       np.zeros((1, 2)))
        initial_x = PiecewisePolynomial()
        
        dircol.SetInitialTrajectory(traj_init_u=initial_u,
                                        traj_init_x=initial_x)


        dircol.AddRunningCost(self.cost_fun)

        # Solve the trajectory optimization 
        result = mp.Solve(dircol)


        #Retrieve the input and state trajectories 
        input_traj = dircol.ReconstructInputTrajectory(result=result)
        state_traj = dircol.ReconstructStateTrajectory(result=result)

        return (input_traj, state_traj)

    def add_traj_from_sample(self, x_sample):

        # first find a (good) goal node in the existing tree to conect to 
        # use value_approx to find all (good) nodes
        context=self.plant.CreateDefaultContext()
        context.SetContinuousState(x_sample)#what's set discrete state
        self.plant.get_input_port(0).FixValue(context, 0)#if input port not set, it doesn't seem the taylor approx can run, the number of ports vary plants by plants
        affine_sys=FirstOrderTaylorApproximation(self.plant,context)
        
        K,S_rand= LinearQuadraticRegulator(affine_sys.A(), affine_sys.B(), self.Q, self.R)
        lqrdist=lambda x1,x2: np.matmul(np.matmul((x2-x1),S_rand),(x2-x1))#might need reshape

#         #for 1/3 case, search and find node with lowest lqrdist
#         #for the rest, randomly pick one from ok neighbours
#         goal_node=1
         if np.random.randint(2)==1:
             goal_indices = np.where(epsH(0:self.Idx)>0.05)[0] #Question1:Idx or Idx+1??
             distances=[]
             distances=[np.sqrt(np.square(X0(indice)-x_sample)) for indice in goal_indices]
             nearest=np.argsort(distances)
             goal_node=goal_indices(nearest(0))
         elif rl_step > 1:
             ok_neighbors = np.where(Qval(0:self.Idx)<valueApprox(x_sample))[0]
             ok_neighbors = np.where(epsH(ok_neighbors)>0.05)[0]
             ok_ind = np.random.randint(len(ok_neighbors))
             goal_node=ok_neighbors(ok_ind)
         else
             ok_neighbors = list(range(0,self.Idx)) #Question2:Idx or Idx+1??
             ok_neighbors = np.where(epsH(ok_neighbors)>0.05)[0]
             ok_ind = np.random.randint(len(ok_neighbors))
             goal_node=ok_neighbors(ok_ind)

         # use snopt to find a new trajectory
         connect, x_traj_pts, u_traj_edges = treeConnect(x_sample, X0(goal_node))
         # if not connect continue next while loop
         if connect == 0:
             print('Not enough neighbors!')
             continue
         # else, then add breakpoints of the trajectory to the graph
         else
                 add_to_graph(x_traj_pts, u_traj_edges, goal_node) #Question3: how to add Idx to each node?



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



