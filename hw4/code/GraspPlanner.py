import logging, numpy, openravepy, time, math, pdb
import os.path

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner
        self.env = self.robot.GetEnv()
        self.manip = self.robot.GetActiveManipulator()

    def GetBasePoseForObjectGrasp(self, obj):
        initial_pose = self.robot.GetTransform()
        initial_config = self.robot.GetActiveDOFValues()

        # Load grasp database
        self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
        if not self.gmodel.load():
            self.gmodel.autogenerate()
        self.graspindices = self.gmodel.graspindices
        self.grasps = self.gmodel.grasps
        self.order_grasps()
        # self.grasps_ordered=numpy.load("grasps_ordered.npy")

        Tgrasp = self.gmodel.getGlobalGraspTransform(self.grasps_ordered[-34],collisionfree=True)
        print Tgrasp
        print type(Tgrasp)
        self.irmodel = openravepy.databases.inversereachability.InverseReachabilityModel(robot=self.robot)
        if not self.irmodel.load():
            self.irmodel.autogenerate()
        densityfn,samplerfn,bounds = self.irmodel.computeBaseDistribution(Tgrasp)

        goals = []
        numfailures = 0
        starttime = time.time()
        timeout = 999999
        N = 4
        with self.robot:
            while len(goals) < N:
                if time.time()-starttime > timeout:
                    break
                poses,jointstate = samplerfn(N-len(goals))
                for pose in poses:

                    #convert pose to pose we want
                    our_pose = self.getOurPose(pose)
                    node_id = self.base_planner.planning_env.discrete_env.ConfigurationToNodeId(our_pose)
                    discrete_pose = self.base_planner.planning_env.discrete_env.NodeIdToConfiguration(node_id)
                    pose = self.getTheirPose(discrete_pose)
                    #convert pose to node id
                    #convert node id to pose
                    #convert pose we want to pose they want

                    self.robot.SetTransform(pose)
                    obstacles=self.robot.GetEnv().GetBodies()
                    if not self.robot.GetEnv().CheckCollision(self.robot,obstacles[1]):
                        self.robot.SetDOFValues(*jointstate)
                        # validate that base is not in collision
                        if not self.manip.CheckIndependentCollision(openravepy.CollisionReport()):
                            q = self.manip.FindIKSolution(Tgrasp,filteroptions=openravepy.IkFilterOptions.CheckEnvCollisions)
                            print("q")
                            print(q)
                            if q is not None:
                                values = self.robot.GetDOFValues()
                                print("DOF values")
                                print(values)
                                values[self.manip.GetArmIndices()] = q
                                print("Arm Indices")
                                print(self.manip.GetArmIndices())
                                print("values")
                                print(values)
                                arm_config = q
                                goals.append((Tgrasp,pose,arm_config))
                            elif self.manip.FindIKSolution(Tgrasp,0) is None:
                                numfailures += 1
        minDistance = 9999999999
        minInd = 0
        current_pos = initial_config[4:6]
        for i, g in enumerate(goals):
            Tgrasp, goal_pose, arm_config = goals[0]
            goal_pos = goal_pose[4:6]
            distance = math.sqrt((goal_pos[0] - current_pos[0])**2+(goal_pos[1] - current_pos[1])**2)
            if distance < minDistance:
                minDistance = distance
                minInd = i

        Tgrasp, pose, arm_config = goals[minInd]
        base_pose = numpy.array(pose)
        quaternion = base_pose[0:4]
        position = base_pose[4:6]
        theta = openravepy.axisAngleFromQuat(quaternion)
        base_pose = numpy.array([position[0],position[1],theta[2]])
        values = self.robot.GetDOFValues()
        values[self.manip.GetArmIndices()] = arm_config
        #return robot to original starting position
        self.robot.SetTransform(pose)
        self.robot.SetDOFValues(values)

        print(base_pose)
        print(arm_config)
        raw_input("Goal ")

        self.robot.SetTransform(initial_pose)
        self.robot.SetActiveDOFValues(initial_config)
        raw_input("Start config")

        return base_pose, arm_config

    '''
    def GetBasePoseForObjectGrasp(self, obj):

        # Load grasp database
        gmodel = openravepy.databases.grasping.GraspingModel(self.robot, obj)
        if not gmodel.load():
            gmodel.autogenerate()

        base_pose = None
        grasp_config = None

        ###################################################################
        # TODO: Here you will fill in the function to compute
        #  a base pose and associated grasp config for the
        #  grasping the bottle
        ###################################################################
        self.gmodel = gmodel
        self.graspindices = gmodel.graspindices
        self.manip = self.robot.GetActiveManipulator()
        self.grasps = gmodel.grasps
        self.order_grasps()
        self.irmodel = openravepy.databases.inversereachability.InverseReachabilityModel(self.robot)
        if not self.irmodel.load():
            print("fail to load irmodel")

        # base_config = self.base_planner.planning_env.herb.GetCurrentConfiguration()
        # print(base_config)
        # base_id = self.base_planner.planning_env.discrete_env.ConfigurationToNodeId(base_config)
        # base_init = self.base_planner.planning_env.discrete_env.NodeIdToConfiguration(base_id)
        # print(base_init)
        base_init = self.robot.GetTransform()
        grasp_init = self.robot.GetActiveDOFValues()

        print("searching for valid base pose and grasp")
        # http://openrave.org/docs/0.8.0/openravepy/examples.tutorial_inversereachability/
        for grasp in self.grasps_ordered:
            grasp = self.grasps_ordered[7]
            print('checking a new grasp')
            Tgrasp = gmodel.getGlobalGraspTransform(grasp, collisionfree=True)
            densityfn, samplerfn, bounds = self.irmodel.computeBaseDistribution(Tgrasp)
            goals = []
            numfailures = 0
            N = 1
            starttime = time.time()
            timeout = 10
            with self.robot:
                while len(goals) < N:
                    # print(time.time()-starttime)
                    if time.time()-starttime > timeout:
                        print('time out')
                        print(len(goals))
                        break
                    poses, jointstate = samplerfn(N-len(goals))
                    # print(len(poses))
                    print('search every pose')
                    for pose in poses:
                        # print(pose)
                        our_pose = self.getOurPose(pose)
                        node_id = self.base_planner.planning_env.discrete_env.ConfigurationToNodeId(our_pose)
                        discrete_pose = self.base_planner.planning_env.discrete_env.NodeIdToConfiguration(node_id)
                        pose = self.getTheirPose(discrete_pose)

                        self.robot.SetTransform(pose)
                        self.robot.SetDOFValues(*jointstate)
                        # validate that base is not in collision
                        if self.robot.GetEnv().CheckCollision(self.robot):
                            continue
                        if not self.manip.CheckIndependentCollision(openravepy.CollisionReport()):
                            q = self.manip.FindIKSolution(Tgrasp,filteroptions=openravepy.IkFilterOptions.CheckEnvCollisions.IgnoreEndEffectorCollisions)
                            # print(q)
                            if q is not None:
                                values = self.robot.GetDOFValues()
                                values[self.manip.GetArmIndices()] = q
                                # goals.append((Tgrasp,pose,values))
                                goals.append((Tgrasp, pose, q))
                            elif self.manip.FindIKSolution(Tgrasp,0) is None:
                                numfailures += 1
            if len(goals) > 0:
                print('get valid base poses')
                break
        # TODO: decide to use which?
        # for ind, goal in enumerate(goals):
        #     raw_input('press ENTER to show goal %d' % ind)
        #     Tgrasp, pose, values = goal
        #     self.robot.SetTransform(pose)
        #     self.robot.SetDOFValues(values)
        idx = 0
        Tgrasp, pose, grasp_config = goals[idx]
        base_pose = numpy.array(pose)
        quaternion = base_pose[0:4]
        position = base_pose[4:6]
        theta = openravepy.axisAngleFromQuat(quaternion)
        base_pose = numpy.array([position[0],position[1],theta[2]])
        values = self.robot.GetDOFValues()
        values[self.manip.GetArmIndices()] = grasp_config

        self.robot.SetTransform(base_init)
        self.robot.SetActiveDOFValues(grasp_init)
        # self.robot.SetActiveDOFValues(grasp_init)

        #
        #
        # self.robot.SetTransform(pose)
        # self.robot.SetDOFValues(values)
        # # base_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        # base_config = self.base_planner.planning_env.herb.GetCurrentConfiguration()
        # print(base_config)
        # base_id = self.base_planner.planning_env.discrete_env.ConfigurationToNodeId(base_config)
        # base_pose = self.base_planner.planning_env.discrete_env.NodeIdToConfiguration(base_id)
        # print(base_pose)
        #
        # grasp_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        #
        # self.base_planner.planning_env.herb.SetCurrentConfiguration(base_init)
        # self.robot.SetActiveDOFValues(grasp_init)

        print('find base_pose and grasp config')
        return base_pose, grasp_config
        '''

    def order_grasps(self):
        fname = 'order_grasp.npy'
        if os.path.isfile(fname):
            self.grasps_ordered = numpy.load(fname)
            print('load order grasp from npy')
            return
        print('start oder grasp')
        self.grasps_ordered = self.grasps.copy() #you should change the order of self.grasps_ordered
        count = 0
        print(len(self.grasps_ordered))
        for grasp in self.grasps_ordered:
            #print count
            count += 1
            grasp[self.graspindices.get('performance')] = self.eval_grasp(grasp)
            print(count)

        # sort!
        #print "sort"
        order = numpy.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
        order = order[::-1]
        self.grasps_ordered = self.grasps_ordered[order]
        print('end of order grasp')
        numpy.save(fname, self.grasps_ordered)
        # for grasp in self.grasps_ordered[:4]:
        #     self.show_grasp(grasp, delay=10)

    def eval_grasp(self, grasp):
        print "call into evaluation function"
        with self.robot:
        # contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
            try:
                contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)
                obj_position = self.gmodel.target.GetTransform()[0:3,3]
                #     # for each contact
                G = numpy.empty([2, 0])
                for c in contacts:
                    pos = c[0:3] - obj_position
                    dir = -c[3:] #this is already a unit vector
                    cross = numpy.cross(pos, dir)
                    g = numpy.vstack((dir, cross))
                    #print g.shape
                    G = numpy.hstack((G, g))
                det = numpy.linalg.det(numpy.dot(G, G.T))
                return math.sqrt(det)
            except openravepy.planning_error,e:
                return 10000

    #displays the grasp
    def show_grasp(self, grasp, delay=1.5):
        with openravepy.RobotStateSaver(self.gmodel.robot):
            with self.gmodel.GripperVisibility(self.gmodel.manip):
                time.sleep(0.1) # let viewer update?
            try:
                with self.env:
                    contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=True)

                    contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
                    self.gmodel.robot.GetController().Reset(0)
                    self.gmodel.robot.SetDOFValues(finalconfig[0])
                    self.gmodel.robot.SetTransform(finalconfig[1])
                    self.env.UpdatePublishedBodies()
                    time.sleep(delay)
            except openravepy.planning_error,e:
                print 'bad grasp!',e

    def PlanToGrasp(self, obj):
        print('get into plan to grasp~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~')

        # Next select a pose for the base and an associated ik for the arm
        base_pose, grasp_config = self.GetBasePoseForObjectGrasp(obj)

        if base_pose is None or grasp_config is None:
            print 'Failed to find solution'
            exit()

        # Now plan to the base pose
        start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        # pdb.set_trace()
        print('start plan for base')
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

        # Now plan the arm to the grasp configuration
        print('start plan for grasp')
        start_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        print('after plan')
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        # pdb.set_trace()
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipulation.CloseFingers()

    def getOurPose(self, pose):
        our_pose = numpy.array(pose)
        quaternion = our_pose[0:4]
        position = our_pose[4:6]
        theta = openravepy.axisAngleFromQuat(quaternion)
        our_pose = numpy.array([position[0],position[1],theta[2]])
        return our_pose

    def getTheirPose(self, pose):
        quat = openravepy.quatFromAxisAngle(numpy.array([0,0,pose[2]]))
        their_pose = [quat[0], quat[1], quat[2], quat[3], pose[0], pose[1], 0]
        return their_pose
