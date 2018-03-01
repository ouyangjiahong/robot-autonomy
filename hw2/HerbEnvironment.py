import numpy
import matplotlib.pyplot as pl
import time
import copy

class HerbEnvironment(object):

    def __init__(self, herb):
        self.robot = herb.robot

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.6],
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

    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p

    def _CheckConfigCollision(self,config):
        """ Checks whether the robot moved to config will collide with anything in the envt
            Returns: flag is true if there is a collision

        """
        # tr = self.robot.GetCurrentConfiguration()
        trnew = config
        tr = self.robot.GetActiveDOFValues()
        self.robot.SetActiveDOFValues(trnew)
        flag = self.robot.GetEnv().CheckCollision(self.robot)
        self.robot.SetActiveDOFValues(tr)
        # self.robot.SetTransform(tr)
        return flag


    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())

        #
        # TODO: Generate and return a random configuration
        #
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        # print(lower_limits.shape)
        while True:
            config = [numpy.random.uniform(low=lower_limits[i],high=upper_limits[i]) for i in range(len(self.robot.GetActiveDOFIndices()))]
            flag = self._CheckConfigCollision(config)
            print(flag)
            if not flag:
                break
        return numpy.array(config)



    def ComputeDistance(self, start_config, end_config):

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
        return path
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

    def InitializePlot(self, goal_config):
        self.fig = pl.figure()
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        pl.zlim([lower_limits[2], upper_limits[2]])
        pl.plot(goal_config[0], goal_config[1], goal_config[2], 'gx')

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
