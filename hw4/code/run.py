#!/usr/bin/env python

import argparse, numpy, openravepy, time

from HerbRobot import HerbRobot
from HerbEnvironment import HerbEnvironment
from SimpleRobot import SimpleRobot
from SimpleEnvironment import SimpleEnvironment
from GraspPlanner import GraspPlanner
from AStarPlanner import AStarPlanner
# TODO: Import the applicable RRTPlanner

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='script for testing planners')
    parser.add_argument('--test', type=int, default=1,
                        help='The test to run')
    parser.add_argument('--hres', type=float, default=0.1,
                        help='xy resolution')
    parser.add_argument('--tres', type=float, default=numpy.pi/8.,
                        help='angular resolution')
    parser.add_argument('-m', '--manip', type=str,
                        help='The manipulator to grasp the bottle with (right or left)')
    parser.add_argument('-d', '--debug', action='store_true',
                        help='Enable debug logging')
    
    args = parser.parse_args()
    
    openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    openravepy.misc.InitOpenRAVELogging()

    if args.debug:
        openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug)

    # Create an environment
    env = openravepy.Environment()
    env.SetViewer('qtcoin')
    env.GetViewer().SetName('Homework 4 Viewer')

    # Load HERB into it
    robot = env.ReadRobotXMLFile('models/robots/herb2_padded.robot.xml')
    env.Add(robot)
        
    theta = -numpy.pi/4.
    robot_pose = numpy.array([[numpy.cos(theta), -numpy.sin(theta), 0, -1.25],
                              [numpy.sin(theta),  numpy.cos(theta), 0,  0.82],
                              [0.              ,  0.              , 1,  0.  ],
                              [0.              ,  0.              , 0,  1.  ]])
    robot.SetTransform(robot_pose)

    right_relaxed = [ 5.65, -1.76, -0.26,  1.96, -1.15 , 0.87, -1.43 ]
    left_relaxed = [ 0.64, -1.76,  0.26,  1.96,  1.16,  0.87,  1.43 ]
    right_manip = robot.GetManipulator('right_wam')
    robot.SetActiveDOFs(right_manip.GetArmIndices())
    robot.SetActiveDOFValues(right_relaxed)
        
    left_manip = robot.GetManipulator('left_wam')
    robot.SetActiveDOFs(left_manip.GetArmIndices())
    robot.SetActiveDOFValues(left_relaxed)

    if args.manip == 'right':
        robot.SetActiveManipulator('right_wam')
    else:
        robot.SetActiveManipulator('left_wam')

    robot.controller = openravepy.RaveCreateController(robot.GetEnv(), 'IdealController')
    robot.ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(robot, iktype=openravepy.IkParameterization.Type.Transform6D)
    if not robot.ikmodel.load():
        robot.ikmodel.autogenerate()

    # Create environments for planning the arm and base
    resolution = [args.hres, args.hres, args.tres]
    herb = HerbRobot(env, robot, args.manip)
    arm_env = HerbEnvironment(herb)
    herb_base = SimpleRobot(env, robot)
    base_env = SimpleEnvironment(herb_base, resolution)

    base_planner = AStarPlanner(base_env, visualize = False)
    arm_planner = None
    # TODO: Here initialize your arm planner
  
    # add a table and move the robot into place
    table = env.ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
    env.Add(table)
    
    table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                              [-1, 0,  0, 0], 
                              [ 0, 1,  0, 0], 
                              [ 0, 0,  0, 1]])
    table.SetTransform(table_pose)

    # set a bottle on the table
    bottle = herb.robot.GetEnv().ReadKinBodyXMLFile('models/objects/fuze_bottle.kinbody.xml')
    herb.robot.GetEnv().Add(bottle)
    table_aabb = table.ComputeAABB()
    bottle_transform = bottle.GetTransform()
    bottle_transform[2,3] = table_aabb.pos()[2] + table_aabb.extents()[2]

    if args.test == 1:
        bottle_transform[:2,3] = table_aabb.pos()[:2]
    elif args.test == 2:
        bottle_transform[0,3] = table_aabb.pos()[0] - 0.5*table_aabb.extents()[0]
        bottle_transform[1,3] = table_aabb.pos()[1] - 0.5*table_aabb.extents()[1]
    elif args.test == 3:
        bottle_transform[0,3] = table_aabb.pos()[0] + 0.5*table_aabb.extents()[0]
        bottle_transform[1,3] = table_aabb.pos()[1] + 0.5*table_aabb.extents()[1]

    bottle.SetTransform(bottle_transform)
 
    planner = GraspPlanner(herb.robot, base_planner, arm_planner)
    planner.PlanToGrasp(bottle)

    import IPython
    IPython.embed()


        
    
