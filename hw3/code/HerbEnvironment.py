import numpy
from DiscreteEnvironment import DiscreteEnvironment

class HerbEnvironment(object):

    def __init__(self, herb, resolution):

        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')

        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7],
                                  [-1, 0,  0, 0],
                                  [ 0, 1,  0, 0],
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)

    def GetSuccessors(self, node_id):
        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes

        # Number of dimensions in the space we're working in
        dimensionality = self.discrete_env.dimension
        # Position in the configuration space
        config = self.discrete_env.NodeIdToConfiguration(node_id)

        # For each dimension
        for i in range(dimensionality):
            # Check if the successor is valid in the "forward" direction
            forward_config = config[:] # Copy each value in the config
            forward_config[i] += self.discrete_env.resolution # Buffer
            # If it's a valid successor add it to the list
            if (self.not_colliding_and_in_bounds(forward_config, i)):
                successors.append(self.discrete_env.ConfigurationToNodeId(forward_config))

            # Check if the successor is valid in the "backwards" direction
            backwards_config = config[:] # Copy each value in the config
            backwards_config[i] -= self.discrete_env.resolution # Buffer
            # If it's a valid successor add it to the list
            if (self.not_colliding_and_in_bounds(backwards_config, i)):
                successors.append(self.discrete_env.ConfigurationToNodeId(backwards_config))
            # variable = raw_input('Any key to continue: ')
        return successors

    def not_colliding_and_in_bounds(self, cfg, index):
        # Check for collisions with environment and self
        with self.robot:
            self.robot.SetActiveDOFValues(numpy.array(cfg))
            is_not_colliding = not (self.robot.GetEnv().CheckCollision(self.robot) or self.robot.CheckSelfCollision())
        # Ensure in bounds
        lower_bound = cfg[index] >= self.lower_limits[index] - self.discrete_env.resolution
        upper_bound = cfg[index] <= self.upper_limits[index] - self.discrete_env.resolution
        is_in_bounds = lower_bound and upper_bound
        return is_not_colliding and is_in_bounds

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that
        # computes the distance between the configurations given
        # by the two node ids
        # Start position in the configuration space
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)
        # End position in the configuration space
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)
        # The change in each dimension in the configuration space
        dist_config = numpy.array(start_config) - numpy.array(end_config)
        # The total distance
        dist = numpy.linalg.norm(dist_config)
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):

        cost = 0

        # TODO: Here you will implement a function that
        # computes the heuristic cost between the configurations
        # given by the two node ids
        cost = self.ComputeDistance(start_id,goal_id)

        return cost
    def ComputePathLength(self, path):
        path_length = 0
        for milestone in range(1,len(path)):
            dist =  numpy.linalg.norm(numpy.array(path[milestone-1])-numpy.array(path[milestone]))
            path_length = path_length + dist
        return path_length

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())

        #
        # TODO: Generate and return a random configuration
        #
    # Brad: Generate and return a random, collision-free, configuration
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        collisionFlag = True
        while collisionFlag is True:
            for dof in range(len(self.robot.GetActiveDOFIndices())):
                config[dof] = lower_limits[dof] + (upper_limits[dof] - lower_limits[dof]) * numpy.random.random_sample()
                self.robot.SetActiveDOFValues(numpy.array(config))
            if(self.robot.GetEnv().CheckCollision(self.robot)) is False:
                if(self.robot.CheckSelfCollision()) is False:
                    collisionFlag = False
            #else:
                #print "Self Collision detected in random configuration"
        #else:
            #print "Collision Detected in random configuration"
        return numpy.array(config)
    def Extend(self, start_config, end_config):

        #
        # TODO: Implement a function which attempts to extend from
        #   a start configuration to a goal configuration
        #
    # Brad: Extend from start to end configuration and return sooner if collision or limits exceeded
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        resolution = 100

    #Calculate incremental configuration changes
        config_inc = [0] * len(self.robot.GetActiveDOFIndices())
        for dof in range(len(self.robot.GetActiveDOFIndices())):
            config_inc[dof] = (end_config[dof] - start_config[dof]) / float(resolution)

    #Set initial config state to None to return if start_config violates conditions
        config = None

    #Move from start_config to end_config
        for step in range(resolution+1):
            prev_config = config
            config = [0] * len(self.robot.GetActiveDOFIndices())
            for dof in range(len(self.robot.GetActiveDOFIndices())):
            #Calculate new config
                config[dof] = start_config[dof] + config_inc[dof]*float(step)

            #Check joint limits
                if config[dof] > upper_limits[dof]:
                    print "Upper joint limit exceeded"
                    return prev_config
                if config[dof] < lower_limits[dof]:
                    print "Lower joint limit exceeded"
                    return prev_config

        #Set config and check for collision
        #CHECK: Lock environment?
            self.robot.SetActiveDOFValues(numpy.array(config))
            if(self.robot.GetEnv().CheckCollision(self.robot)) is True:
            #print "Collision Detected in extend"
                return prev_config
            if(self.robot.CheckSelfCollision()) is True:
            #print "Self Collision Detected in extend"
                return prev_config
        return end_config
