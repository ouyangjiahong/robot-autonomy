#!/usr/bin/env python

import argparse, numpy, openravepy, time

from HerbRobot import HerbRobot
from SimpleRobot import SimpleRobot
from HerbEnvironment import HerbEnvironment
from SimpleEnvironment import SimpleEnvironment

from RRTPlanner import RRTPlanner
from RRTConnectPlanner import RRTConnectPlanner

def main(robot, planning_env, planner):

    raw_input('Press any key to begin planning')

    start_config = numpy.array(robot.GetCurrentConfiguration())
    if robot.name == 'herb':
        goal_config = numpy.array([ 3.68, -1.90,  0.00,  2.20,  0.00,  0.00,  0.00 ])
    else:
        goal_config = numpy.array([4.0, 4.0])
    t1 = time.time()
    plan = planner.Plan(start_config, goal_config)
    t2 = time.time()
    print("Time taken for planning: ",t2-t1)
    print(plan)
    plan_short = planning_env.ShortenPath(plan)
    traj = robot.ConvertPlanToTrajectory(plan_short)
    robot.ExecuteTrajectory(traj)

if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='script for testing planners')
    
    parser.add_argument('-r', '--robot', type=str, default='simple',
                        help='The robot to load (herb or simple)')
    parser.add_argument('-p', '--planner', type=str, default='rrt',
                        help='The planner to run (rrt or rrtconnect)')
    parser.add_argument('-m', '--manip', type=str, default='right',
                        help='The manipulator to plan with (right or left) - only applicable if robot is of type herb')
    parser.add_argument('-v', '--visualize', action='store_true',
                        help='Enable visualization of tree growth (only applicable for simple robot)')
    parser.add_argument('-d', '--debug', action='store_true',
                        help='Enable debug logging')
    

    args = parser.parse_args()
    
    openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Info)
    openravepy.misc.InitOpenRAVELogging()

    if args.debug:
        openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug)

    env = openravepy.Environment()
    env.SetViewer('qtcoin')
    env.GetViewer().SetName('Homework 2 Viewer')

    # First setup the environment and the robot
    visualize = args.visualize
    if args.robot == 'herb':
        robot = HerbRobot(env, args.manip)
        planning_env = HerbEnvironment(robot)
        visualize = False
    elif args.robot == 'simple':
        robot = SimpleRobot(env)
        planning_env = SimpleEnvironment(robot)
    else:
        print 'Unknown robot option: %s' % args.robot
        exit(0)

    # Next setup the planner
    if args.planner == 'rrt':
        planner = RRTPlanner(planning_env, visualize=visualize)
    elif args.planner == 'rrtconnect':
        planner = RRTConnectPlanner(planning_env, visualize=visualize)
    else:
        print 'Unknown planner option: %s' % args.planner
        exit(0)

    main(robot, planning_env, planner)

    import IPython
    IPython.embed()

        
    
