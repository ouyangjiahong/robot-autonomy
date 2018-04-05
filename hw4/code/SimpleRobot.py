import numpy, openravepy, time

class SimpleRobot(object):

    def __init__(self, env, robot):
        self.name = 'simple'
        self.robot = robot
        self.wheel_radius = 0.25
        self.wheel_distance = 0.5
        self.max_wheel_velocity = 1.0

    def GetCurrentConfiguration(self):
        t = self.robot.GetTransform()
        aa = openravepy.axisAngleFromRotationMatrix(t)
        pose = [t[0,3], t[1,3], aa[2]]
        return numpy.array(pose)

    def SetCurrentConfiguration(self, config):
        
        transform = [[numpy.cos(config[2]), -numpy.sin(config[2]), 0, config[0]],
                     [numpy.sin(config[2]),  numpy.cos(config[2]), 0, config[1]],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]]
        self.robot.SetTransform(transform)

    def ConvertPlanToTrajectory(self, plan):
        # Create a trajectory and insert all points
        return plan

    def ExecuteTrajectory(self, traj, stepsize = 0.01):
        
        # Send the trajectory to the controller and wait for execution to complete
        offset = None
        for action in traj:
            config = self.GetCurrentConfiguration()

            for fconfig in action.footprint:
                new_config = fconfig.copy()
                new_config[:2] += config[:2]
                self.SetCurrentConfiguration(new_config)
                time.sleep(0.001)

