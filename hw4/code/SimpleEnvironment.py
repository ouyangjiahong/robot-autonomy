import numpy, openravepy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment
import pdb

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        lower_limits, upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)

        self.resolution = resolution
        self.ConstructActions()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        # Initialize the footprint
        config = start_config.copy()
        footprint = [numpy.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance
                
            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*numpy.array([xdot, ydot, tdot])
            if config[2] > numpy.pi:
                config[2] -= 2.*numpy.pi
            if config[2] < -numpy.pi:
                config[2] += 2.*numpy.pi

            footprint_config = config.copy()
            footprint_config[:2] -= start_config[:2]
            footprint.append(footprint_config)

            timecount += stepsize
            
        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)

        return footprint

    def PlotActionFootprints(self, idx):

        actions = self.actions[idx]
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        
        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            pl.plot(xpoints, ypoints, 'k')
                     
        pl.ion()
        pl.show()

        

    def ConstructActions(self):

        # Actions is a dictionary that maps orientation of the robot to
        #  an action set
        self.actions = dict()
              
        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)
        w = 1
        dt = 0.5
        primitives = [Control(w,w,dt),Control(w,-w,dt),Control(w,0,dt),
                    Control(0,w,dt),Control(-w,-w,dt),Control(w,w,dt/3),Control(w,-w,dt/3),Control(w,0,dt/3),
                    Control(0,w,dt/3),Control(-w,-w,dt/3)]
        #primitives = [Control(w,w,dt),Control(w,-w,dt),Control(w,w,dt/3),Control(w,-w,dt/3)]
        # Iterate through each possible starting orientation
        for idx in range(int(self.discrete_env.num_cells[2])):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = numpy.array(self.discrete_env.GridCoordToConfiguration(grid_coordinate))
            
            # TODO: Here you will construct a set of actions
            #  to be used during the planning process
            #
            for prim in primitives:
                self.actions[idx].append(Action(prim,
                    self.GenerateFootprintFromControl(start_config,prim)))
            
            

    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes

        current_config = self.herb.GetCurrentConfiguration()
        node_coord = self.discrete_env.NodeIdToGridCoord(node_id)
        node_config = self.discrete_env.NodeIdToConfiguration(node_id)
        #print "Finding Successor for: ",node_id,node_coord,node_config

        actions = self.actions[node_coord[2]]
        for action in actions:
            collide = False
            for fconfig in action.footprint:
                new_config = numpy.array(fconfig).copy()
                new_config[:2] += node_config[:2]
                if not self.CheckCollisionAndLimits(new_config):
                    collide = True
                    break
            if not collide:
                final_config = numpy.array(action.footprint[-1]).copy()
                final_config[:2] += node_config[:2]
                final_id = self.discrete_env.ConfigurationToNodeId(final_config)
                successors.append({"id":final_id,"control":action.control})
        self.herb.SetCurrentConfiguration(current_config)
        return successors

    def CheckCollisionAndLimits(self,config):
        #print "checking config: ",config
        old_config = self.herb.GetCurrentConfiguration()
        self.herb.SetCurrentConfiguration(config)
        is_not_colliding = not (self.herb.robot.GetEnv().CheckCollision(self.herb.robot) or self.herb.robot.CheckSelfCollision())
        lb = (config >= (numpy.array(self.boundary_limits[0])-numpy.array(self.resolution))).all()
        ub = (config <= (numpy.array(self.boundary_limits[1])-numpy.array(self.resolution))).all()
        return is_not_colliding and lb and ub


    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)
        dist_config = numpy.array(start_config)-numpy.array(end_config)
        #dist = dist_config[0]**2 + dist_config[1]**2 + 0.1*(dist_config[2]**2)
        dist = numpy.linalg.norm(dist_config)
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        cost = self.ComputeDistance(start_id,goal_id)
        
        return cost

