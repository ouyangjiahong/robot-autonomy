import logging, numpy, openravepy, time, math, pdb
import os.path

class GraspPlanner(object):

    def __init__(self, robot, base_planner, arm_planner):
        self.robot = robot
        self.base_planner = base_planner
        self.arm_planner = arm_planner


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

        base_init = self.base_planner.planning_env.herb.GetCurrentConfiguration()
        grasp_init = self.robot.GetActiveDOFValues()

        print("searching for valid base pose and grasp")
        # http://openrave.org/docs/0.8.0/openravepy/examples.tutorial_inversereachability/
        for grasp in self.grasps_ordered:
            print('checking a new grasp')
            Tgrasp = gmodel.getGlobalGraspTransform(grasp, collisionfree=True)
            densityfn, samplerfn, bounds = self.irmodel.computeBaseDistribution(Tgrasp)
            goals = []
            numfailures = 0
            N = 100
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
                        self.robot.SetTransform(pose)
                        self.robot.SetDOFValues(*jointstate)
                        # validate that base is not in collision
                        if not self.manip.CheckIndependentCollision(openravepy.CollisionReport()):
                            q = self.manip.FindIKSolution(Tgrasp,filteroptions=openravepy.IkFilterOptions.CheckEnvCollisions.IgnoreEndEffectorCollisions)
                            # print(q)
                            if q is not None:
                                values = self.robot.GetDOFValues()
                                values[self.manip.GetArmIndices()] = q
                                goals.append((Tgrasp,pose,values))
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
        Tgrasp, pose, values = goals[idx]
        self.robot.SetTransform(pose)
        self.robot.SetDOFValues(values)
        base_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        grasp_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())

        self.base_planner.planning_env.herb.SetCurrentConfiguration(base_init)
        self.robot.SetActiveDOFValues(grasp_init)

        print('find base_pose and grasp config')
        return base_pose, grasp_config

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
        pdb.set_trace()

        if base_pose is None or grasp_config is None:
            print 'Failed to find solution'
            exit()

        # Now plan to the base pose
        start_pose = numpy.array(self.base_planner.planning_env.herb.GetCurrentConfiguration())
        pdb.set_trace()
        base_plan = self.base_planner.Plan(start_pose, base_pose)
        base_traj = self.base_planner.planning_env.herb.ConvertPlanToTrajectory(base_plan)

        print 'Executing base trajectory'
        self.base_planner.planning_env.herb.ExecuteTrajectory(base_traj)

        # Now plan the arm to the grasp configuration
        start_config = numpy.array(self.arm_planner.planning_env.herb.GetCurrentConfiguration())
        arm_plan = self.arm_planner.Plan(start_config, grasp_config)
        arm_traj = self.arm_planner.planning_env.herb.ConvertPlanToTrajectory(arm_plan)

        print 'Executing arm trajectory'
        self.arm_planner.planning_env.herb.ExecuteTrajectory(arm_traj)

        # Grasp the bottle
        task_manipulation = openravepy.interfaces.TaskManipulation(self.robot)
        task_manipultion.CloseFingers()
