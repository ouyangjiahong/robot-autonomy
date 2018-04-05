import openravepy

class HerbRobot(object):

    def __init__(self, env, robot, manip):
        self.name = 'herb'
        self.robot = robot

        if manip == 'right':
            self.manip = self.robot.GetManipulator('right_wam')
            self.robot.SetActiveManipulator('right_wam')
        else:
            self.manip = self.robot.GetManipulator('left_wam')
            self.robot.SetActiveManipulator('left_wam')

        self.robot.SetActiveDOFs(self.manip.GetArmIndices())

    def GetCurrentConfiguration(self):
        return self.robot.GetActiveDOFValues()

    def ConvertPlanToTrajectory(self, plan):
        
        # Create a trajectory
        traj = openravepy.RaveCreateTrajectory(self.robot.GetEnv(), 'GenericTrajectory')
        config_spec = self.robot.GetActiveConfigurationSpecification()
        traj.Init(config_spec)

        idx = 0
        for pt in plan:
            traj.Insert(idx, pt)
            idx = idx + 1

        openravepy.planningutils.RetimeActiveDOFTrajectory(traj, self.robot, maxvelmult=1, maxaccelmult=1, hastimestamps=False, plannername='ParabolicTrajectoryRetimer')

        return traj

    def ExecuteTrajectory(self, traj):

        # Send the trajectory to the controller and wait for execution to complete
        self.robot.GetController().SetPath(traj)
        self.robot.WaitForController(0)
