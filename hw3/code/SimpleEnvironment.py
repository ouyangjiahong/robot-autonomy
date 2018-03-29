import numpy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class SimpleEnvironment(object):

    def __init__(self, herb, resolution):
        self.robot = herb.robot
        self.lower_limits = [-5., -5.]
        self.upper_limits = [5., 5.]
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)
        self.boundary_limits = [[-5., -5.], [5., 5.]]
        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.5],
                                  [-1, 0,  0, 0],
                                  [ 0, 1,  0, 0],
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        #####
        self.p = 0.0

    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        # Change to True to generate all 8 points instead of just 4
        all_eight_points = False
        # X or Y distance to move in a single step
        delta = self.discrete_env.resolution
        # Get the limits
        top_limit = self.upper_limits[1] - delta
        bottom_limit = self.lower_limits[1] + delta
        left_limit = self.lower_limits[0] + delta
        right_limit = self.upper_limits[0] - delta
        # Coordinates of the node
        coord = self.discrete_env.NodeIdToConfiguration(node_id)
        # The four coordinates to consider moving to
        top_coord    = [coord[0], coord[1] + delta]
        bottom_coord = [coord[0], coord[1] - delta]
        left_coord   = [coord[0] - delta, coord[1]]
        right_coord  = [coord[0] + delta, coord[1]]
        # The other four coordinates you can optionally consider
        if all_eight_points:
            top_left_coord     = [coord[0] - delta, coord[1] + delta]
            top_right_coord    = [coord[0] + delta, coord[1] + delta]
            bottom_left_coord  = [coord[0] - delta, coord[1] - delta]
            bottom_right_coord = [coord[0] + delta, coord[1] - delta]
        # Get the IDs of the four coordinates
        top_id          = self.discrete_env.ConfigurationToNodeId(top_coord)
        bottom_id       = self.discrete_env.ConfigurationToNodeId(bottom_coord)
        left_id         = self.discrete_env.ConfigurationToNodeId(left_coord)
        right_id        = self.discrete_env.ConfigurationToNodeId(right_coord)
        # Get the IDs of the optional four
        if all_eight_points:
            top_left_id     = self.discrete_env.ConfigurationToNodeId(top_left_coord)
            top_right_id    = self.discrete_env.ConfigurationToNodeId(top_right_coord)
            bottom_left_id  = self.discrete_env.ConfigurationToNodeId(bottom_left_coord)
            bottom_right_id = self.discrete_env.ConfigurationToNodeId(bottom_right_coord)
        # If it's below the top limit, add the top node
        if top_coord[1] < top_limit and not self.is_colliding(top_coord):
            successors.append(top_id)
            # Check and optionally add the two optional top nodes
            if all_eight_points:
                if left_coord[0] > left_limit:
                    successors.append(top_left_id)
                if right_coord[0] < right_limit:
                    successors.append(top_right_id)
        # If it's to the right of the left limit add the left node
        if left_coord[0] > left_limit and not self.is_colliding(left_coord):
            successors.append(left_id)
        # If it's to the left of the right limit add the right node
        if right_coord[0] < right_limit and not self.is_colliding(right_coord):
            successors.append(right_id)
        # If it's above the bottom limit add the bottom node
        if bottom_coord[1] > bottom_limit and not self.is_colliding(bottom_coord):
            successors.append(bottom_id)
            # Check and optionally add the two optional bottom nodes
            if all_eight_points:
                if left_coord[0] > left_limit:
                    successors.append(bottom_left_id)
                if right_coord[0] < right_limit:
                    successors.append(bottom_right_id)
        # variable = raw_input('Any key to continue: ')
        return successors

    def is_colliding(self,cfg):
        trans = self.robot.GetTransform()
        trans[0][3] = cfg[0]
        trans[1][3] = cfg[1]
        self.robot.SetTransform(trans)
        return self.robot.GetEnv().CheckCollision(self.robot)

    def ComputeDistance(self, start_id, end_id):
        # TODO: Here you will implement a function that
        # computes the distance between the configurations given
        # by the two node ids

        # Calculate distance using pythagorean theorem, may need abstracted for higher dimensional spaces
        # a^2 = b^2 + c^2
        startX = self.discrete_env.NodeIdToConfiguration(start_id)[0]
        startY = self.discrete_env.NodeIdToConfiguration(start_id)[1]
        endX = self.discrete_env.NodeIdToConfiguration(end_id)[0]
        endY = self.discrete_env.NodeIdToConfiguration(end_id)[1]
        # a^2
        deltaXSquared = (endX - startX) * (endX - startX)
        # b^2
        deltaYSquared = (endY - startY) * (endY - startY)

        # Square root of a^2 + b^2 gives us the distancd
        return numpy.sqrt(deltaXSquared + deltaYSquared)

    def ComputeHeuristicCost(self, start_id, goal_id):
        # TODO: Here you will implement a function that
        # computes the heuristic cost between the configurations
        # given by the two node ids

        cost_sum = 0;
        for x in range(self.discrete_env.dimension):
            cost_sum = cost_sum + abs(self.discrete_env.NodeIdToConfiguration(start_id)[x] - self.discrete_env.NodeIdToConfiguration(goal_id)[x])

        return cost_sum

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        pl.xlim([self.lower_limits[0], self.upper_limits[0]])
        pl.ylim([self.lower_limits[1], self.upper_limits[1]])
        pl.plot(goal_config[0], goal_config[1], 'gx')

        # Show all obstacles in environment
        for b in self.robot.GetEnv().GetBodies():
            if b.GetName() == self.robot.GetName():
                continue
            bb = b.ComputeAABB()
            pl.plot([bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] + bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0],
                     bb.pos()[0] - bb.extents()[0]],
                    [bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] + bb.extents()[1],
                     bb.pos()[1] - bb.extents()[1]], 'r')


        pl.ion()
        pl.show()

    def PlotEdge(self, sconfig, econfig, color='k.-'):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                color, linewidth=2.5)
        pl.draw()
        pl.pause(0.01)

    ###############################

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        
    def GenerateRandomConfiguration(self):
        config = [0] * 2;
        lower_limits, upper_limits = self.boundary_limits
        #
        # TODO: Generate and return a random configuration (DONE)
        #
        while True:
            config = [numpy.random.uniform(low=self.boundary_limits[0][i],high=self.boundary_limits[1][i]) for i in range(2)]
            flag = self._CheckConfigCollision(config)
            if not flag:
                break
        return numpy.array(config)
    
    def _CheckConfigCollision(self,config):
        """ Checks whether the robot moved to config will collide with anything in the envt
            Returns: flag is true if there is a collision

        """
        tr = self.robot.GetTransform()
        trnew = tr.copy()
        trnew[0][3] = config[0]
        trnew[1][3] = config[1]
        self.robot.SetTransform(trnew)
        flag = self.robot.GetEnv().CheckCollision(self.robot)
        self.robot.SetTransform(tr)
        return flag
    
    def ComputeDistanceConfig(self, start_config, end_config):
        #
        # TODO: Implement a function which computes the distance between
        # two configurations (DONE)
        #
        return numpy.linalg.norm(start_config-end_config)

    def Extend(self, start_config, end_config, step=0.05):
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration (DONE)
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
        return path
