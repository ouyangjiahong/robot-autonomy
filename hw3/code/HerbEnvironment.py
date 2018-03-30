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
        # goal sampling probability
        self.p = 0.0

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

    def ComputeDistanceConfig(self, start_id, end_id):

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
        cost = self.ComputeDistanceConfig(start_id,goal_id)

        return cost

    ###
    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p


    def GenerateRandomConfiguration(self):
        config = [0] * self.robot.GetActiveDOF()
        lower_limits,upper_limits = self.robot.GetActiveDOFLimits()
        #
        # TODO: Generate and return a random configuration
        #
        while True:
            config = [numpy.random.uniform(low=lower_limits[i],high=upper_limits[i])
                         for i in range(self.robot.GetActiveDOF())]
            flag = self._CheckConfigCollision(config)
            if not flag:
                break
        return numpy.array(config)

    def _CheckConfigCollision(self,config):
        """ Checks whether the robot moved to config will collide with anything in the envt
            Returns: flag is true if there is a collision

        """
        tr = self.robot.GetActiveDOFValues()
        self.robot.SetActiveDOFValues(config)
        flag = self.robot.GetEnv().CheckCollision(self.robot) or self.robot.CheckSelfCollision()
        self.robot.SetActiveDOFValues(tr)
        return flag

    def ComputeDistanceConfig(self, start_config, end_config):

        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        return numpy.linalg.norm(start_config-end_config)


    def Extend(self, start_config, end_config, step=0.05):

        #
        # TODO: Implement a function which attempts to extend from
        #   a start configuration to a goal configuration
        #
        reached = False
        delta = end_config - start_config
        for i in numpy.arange(0.0,1.0+step,step):
            new_config = start_config + i*delta
            flag = self._CheckConfigCollision(new_config)
            if flag:
                break
        if i==0:
            return reached,None
        else:
            if i==1.0:
                reached = True
            return reached,start_config + (i-step)*delta

    def ShortenPath(self, path, timeout=5.0):

        #
        # TODO: Implement a function which performs path shortening
        #  on the given path.  Terminate the shortening after the
        #  given timout (in seconds).
        #
        start = end = time.time()
        path = list(path)
        short_path = []
        # print path
        print 'original path length: '
        print len(path)
        # print path
        while not numpy.array_equal(path, short_path):
            short_path = copy.deepcopy(path)
            for i in xrange(len(short_path)):
                print len(short_path)
                if len(short_path) == 3:
                    return short_path
                if time.time() - start > timeout:
                    return short_path
                two_points = numpy.random.choice(len(short_path)-1, size=2, replace=False)

                if two_points[0] > two_points[1]:
                    a = two_points[1] + 1
                    b = two_points[0] + 1
                else:
                    a = two_points[0] + 1
                    b = two_points[1] + 1
                if a+1 == b:
                    continue
                point_one = path[a]
                point_two = path[b]
                point_mid = path[a+1]
                diff = (point_mid - point_one)/100
                for i in xrange(100):
                    reached, end_point = self.Extend(point_one + i*diff, point_two)
                    if reached:
                        short_path[a+1] = point_one + i * diff
                        short_path = short_path[:a+2] + short_path[b:]
                        break
                # print short_path
            path = short_path
            print 'length after shortening: '
            print len(short_path)

        # print short_path

        return short_path
