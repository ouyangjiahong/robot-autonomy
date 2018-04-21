import numpy
import random
import time
import copy

class HerbEnvironment(object):

    def __init__(self, herb):
        self.robot = herb.robot
        self.herb = herb

        # add a table and move the robot into place
        # table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        # self.robot.GetEnv().Add(table)
        #
        # table_pose = numpy.array([[ 0, 0, -1, 0.6],
        #                           [-1, 0,  0, 0],
        #                           [ 0, 1,  0, 0],
        #                           [ 0, 0,  0, 1]])
        # table.SetTransform(table_pose)

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

    def CheckCollision(self, config):
        # transform = numpy.identity(4)
        # transform[0, 3] = config[0]
        # transform[1, 3] = config[1]
        # self.robot.SetTransform(transform)
        # prev_conf = self.robot.GetActiveDOFValues()
        with self.robot:
            self.robot.SetActiveDOFValues(config)
            collide = self.robot.GetEnv().CheckCollision(self.robot)
        # self.robot.SetActiveDOFValues(prev_conf)
        # transform[0, 3] = -config[0]
        # transform[1, 3] = -config[1]
        # self.robot.SetTransform(transform)
        return collide

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())
        lower_limits, upper_limits = self.robot.GetActiveDOFLimits()
        # print self.robot.GetActiveDOFLimits()
        #print self.robot.GetActiveDOFLimits()
        #
        # TODO: Generate and return a random configuration
        #
        collide = True
        while collide:
            config[0] = random.random() * abs(lower_limits[0] - upper_limits[0]) + lower_limits[0]
            config[1] = random.random() * abs(lower_limits[1] - upper_limits[1]) + lower_limits[1]
            config[2] = random.random() * abs(lower_limits[2] - upper_limits[2]) + lower_limits[2]
            config[3] = random.random() * abs(lower_limits[3] - upper_limits[3]) + lower_limits[3]
            config[4] = random.random() * abs(lower_limits[4] - upper_limits[4]) + lower_limits[4]
            config[5] = random.random() * abs(lower_limits[5] - upper_limits[5]) + lower_limits[5]
            config[6] = random.random() * abs(lower_limits[6] - upper_limits[6]) + lower_limits[6]
            collide = self.CheckCollision(config)

        return numpy.array(config)



    def ComputeDistance(self, start_config, end_config):

        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        distance = numpy.sqrt(numpy.sum(numpy.square(start_config - end_config)))

        return distance


    def Extend(self, start_config, end_config):

        #
        # TODO: Implement a function which attempts to extend from
        #   a start configuration to a goal configuration
        #
        # y_coord = numpy.interp(x_coord, [start_config[0], end_config[0]], [start_config[1], end_config[1]])
        # number of interpolate points
        dof = len(self.robot.GetActiveDOFIndices())
        num = 1000
        positions = numpy.zeros((dof,num))
        # positions[0,:] = numpy.linspace(start_config[0], end_config[0], num)
        for i in range(dof):
            positions[i,:] = numpy.linspace(start_config[i], end_config[i], num)

        config = numpy.copy(start_config)
        for i in range(num):
            if not self.CheckCollision(positions[:,i]):
                config = numpy.copy(positions[:,i])
            else:
                break
        return config

    def ShortenPath(self, path, timeout=5.0):
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
                    end_point = self.Extend(point_one + i*diff, point_two)
                    if numpy.array_equal(end_point, point_two):
                        short_path[a+1] = point_one + i * diff
                        short_path = short_path[:a+2] + short_path[b:]
                        break
                # print short_path
            path = short_path
            # print len(short_path)
        print short_path

        return short_path
    	# startTime = time.time()
    	# #if path has less than three points, it's already as short as possible
    	# if(len(path) < 3):
        #     return path
    	# while(time.time() - startTime < timeout):
        #     pathLength = len(path)
        #     seg1_start = 0
        #     seg2_start = 0
        #     while not(seg1_start < seg2_start):
        #         seg1_start = random.randint(0, pathLength-2)
        #         seg2_start = random.randint(0, pathLength-2)
        #     seg1_start_config = path[seg1_start]
        #     seg1_end_config = path[seg1_start+1]
        #     seg2_start_config = path[seg2_start]
        #     seg2_end_config = path[seg2_start+1]
        #     dof = 7
        #     num = 500
        #     seg1_positions = numpy.zeros((dof,num))
        #     seg2_positions = numpy.zeros((dof,num))
        #
        #     seg1_positions[0,:] = numpy.linspace(seg1_start_config[0], seg1_end_config[0], num)
        #     seg2_positions[0,:] = numpy.linspace(seg2_start_config[0], seg2_end_config[0], num)
        #     for i in range(1,dof):
        #         seg1_positions[i,:] = numpy.linspace(seg1_start_config[i], seg1_end_config[i], num)
        #         seg2_positions[i,:] = numpy.linspace(seg2_start_config[i], seg2_end_config[i], num)
        #
    	# 	randA = random.randint(0, num - 1)
    	# 	randB = random.randint(0, num - 1)
        #
    	# 	posA = seg1_positions[:, randA]
    	# 	posB = seg2_positions[:, randB]
        #
    	# 	if (numpy.array_equal(posB, self.Extend(posA, posB))):
    	# 		newPath = path[0:(seg1_start+1)]
    	# 		newPath.append(posA)
    	# 		newPath.append(posB)
    	# 		newPath.extend(path[(seg2_start+1):len(path)])
        #         print("new path")
        #         print(newPath)
        #         oldDistance = 0
        #         for i in range(0, len(path)-1):
        #             ithDistance = self.ComputeDistance(path[i], path[i+1])
        #             oldDistance = oldDistance + ithDistance
        #
        #         newDistance = 0
        #         for i in range(0, len(newPath)-1):
        #             ithDistance = self.ComputeDistance(newPath[i], newPath[i+1])
        #             newDistance = newDistance + ithDistance
        #
        #         if(oldDistance > newDistance):
        #             path = newPath
    	# return path
