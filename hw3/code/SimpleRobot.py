import numpy, openravepy

class SimpleRobot(object):

    def __init__(self, env):
        self.name = 'simple'
        self.robot = env.ReadRobotXMLFile('models/robots/pr2-beta-sim.robot.xml')
        env.Add(self.robot)

        right_relaxed = [ 0., 0., 0., -2.3, 0., 0., 0.]
        left_relaxed = [ 0., 0., 0., -2.3, 0., 0., 0.]
        
        right_manip = self.robot.GetManipulator('rightarm')
        self.robot.SetActiveDOFs(right_manip.GetArmIndices())
        self.robot.SetActiveDOFValues(right_relaxed)
        
        left_manip = self.robot.GetManipulator('leftarm')
        self.robot.SetActiveDOFs(left_manip.GetArmIndices())
        self.robot.SetActiveDOFValues(left_relaxed)

        self.robot.controller = openravepy.RaveCreateController(self.robot.GetEnv(), 'IdealController')

    def GetCurrentConfiguration(self):
        t = self.robot.GetTransform()
        pose = t[:2,3]
        return pose

    def ConvertPlanToTrajectory(self, plan):
        # Create a trajectory and insert all points
        traj = openravepy.RaveCreateTrajectory(self.robot.GetEnv(), 'GenericTrajectory')

        doft = openravepy.DOFAffine.X | openravepy.DOFAffine.Y 
        config_spec = openravepy.RaveGetAffineConfigurationSpecification(doft, self.robot)
        traj.Init(config_spec)

        idx = 0
        for pt in plan:
            traj.Insert(idx, pt)
            idx = idx + 1
        return traj

    def ExecuteTrajectory(self, traj):
        
        # Send the trajectory to the controller and wait for execution to complete
        max_vel = self.robot.GetAffineTranslationMaxVels()[:2]
        max_accel = 3 * max_vel
        openravepy.planningutils.RetimeAffineTrajectory(traj, max_vel,
                                                        max_accel, False)

        self.robot.GetController().SetPath(traj)
        self.robot.WaitForController(0)
