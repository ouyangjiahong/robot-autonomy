import openravepy

class HerbRobot(object):

    def __init__(self, env, manip):
        self.name = 'herb'
        self.robot = env.ReadRobotXMLFile('models/robots/herb2_padded.robot.xml')
        env.Add(self.robot)

        right_relaxed = [ 5.65, -1.76, -0.26,  1.96, -1.15 , 0.87, -1.43 ]
        left_relaxed = [ 0.64, -1.76,  0.26,  1.96,  1.16,  0.87,  1.43 ]
        
        right_manip = self.robot.GetManipulator('right_wam')
        self.robot.SetActiveDOFs(right_manip.GetArmIndices())
        self.robot.SetActiveDOFValues(right_relaxed)
        
        left_manip = self.robot.GetManipulator('left_wam')
        self.robot.SetActiveDOFs(left_manip.GetArmIndices())
        self.robot.SetActiveDOFValues(left_relaxed)

        if manip == 'right':
            self.manip = right_manip
            self.robot.SetActiveManipulator('right_wam')
        else:
            self.manip = left_manip
            self.robot.SetActiveManipulator('left_wam')

        self.robot.SetActiveDOFs(self.manip.GetArmIndices())
    
        self.robot.controller = openravepy.RaveCreateController(self.robot.GetEnv(), 'IdealController')

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
