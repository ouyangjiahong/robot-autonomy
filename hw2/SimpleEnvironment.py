import numpy
import matplotlib.pyplot as pl
import time
import copy

class SimpleEnvironment(object):

    def __init__(self, herb):
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5.], [5., 5.]]

        # add an obstacle
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 1.0],
                                  [-1, 0,  0, 0],
                                  [ 0, 1,  0, 0],
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)

        # goal sampling probability
        self.p = 0.0

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
        # print "tr, config:", trnew.shape, len(config)
        self.robot.SetTransform(trnew)
        flag = self.robot.GetEnv().CheckCollision(self.robot)
        self.robot.SetTransform(tr)
        return flag


    def ComputeDistance(self, start_config, end_config):
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
        xp = numpy.array([start_config[0], end_config[0]])
        # if not np.all(np.diff(xp) > 0):
        num_points = 100
        x_coord = numpy.linspace(start_config[0], end_config[0], num_points)
        y_coord = numpy.linspace(start_config[1], end_config[1], num_points)

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
        # return path
        start = end = time.time()
        path = list(path)
        short_path = []
        # print path
        # print len(path)
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
            # print len(short_path)
        # print short_path

        return short_path
        # return path


    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
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

    def PlotEdge(self, sconfig, econfig):
        pl.plot([sconfig[0], econfig[0]],
                [sconfig[1], econfig[1]],
                'k.-', linewidth=2.5)
        pl.draw()
        pl.pause(0.2)
