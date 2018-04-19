#!/usr/bin/env python

PACKAGE_NAME = 'hw1'

# Standard Python Imports
import os
import copy
import time
import math
import numpy as np
np.random.seed(0)
import scipy

# OpenRAVE
import openravepy
#openravepy.RaveInitialize(True, openravepy.DebugLevel.Debug)


curr_path = os.getcwd()
relative_ordata = '/models'
ordata_path_thispack = curr_path + relative_ordata


#this sets up the OPENRAVE_DATA environment variable to include the files we're using
openrave_data_path = os.getenv('OPENRAVE_DATA', '')
openrave_data_paths = openrave_data_path.split(':')
if ordata_path_thispack not in openrave_data_paths:
  if openrave_data_path == '':
      os.environ['OPENRAVE_DATA'] = ordata_path_thispack
  else:
      datastr = str('%s:%s'%(ordata_path_thispack, openrave_data_path))
      os.environ['OPENRAVE_DATA'] = datastr

#set database file to be in this folder only
relative_ordatabase = '/database'
ordatabase_path_thispack = curr_path + relative_ordatabase
os.environ['OPENRAVE_DATABASE'] = ordatabase_path_thispack

#get rid of warnings
openravepy.RaveInitialize(True, openravepy.DebugLevel.Fatal)
openravepy.misc.InitOpenRAVELogging()



class RoboHandler:
  def __init__(self):
    self.openrave_init()
    self.problem_init()

    #order grasps based on your own scoring metric
    #self.order_grasps()

    #order grasps with noise
    self.order_grasps_noisy()


  # the usual initialization for openrave
  def openrave_init(self):
    self.env = openravepy.Environment()
    self.env.SetViewer('qtcoin')
    self.env.GetViewer().SetName('HW1 Viewer')
    self.env.Load('models/%s.env.xml' %PACKAGE_NAME)
    # time.sleep(3) # wait for viewer to initialize. May be helpful to uncomment
    self.robot = self.env.GetRobots()[0]
    self.manip = self.robot.GetActiveManipulator()
    self.end_effector = self.manip.GetEndEffector()

  # problem specific initialization - load target and grasp module
  def problem_init(self):
    self.target_kinbody = self.env.ReadKinBodyURI('models/objects/champagne.iv')
    #self.target_kinbody = self.env.ReadKinBodyURI('models/objects/winegoblet.iv')
    #self.target_kinbody = self.env.ReadKinBodyURI('models/objects/black_plastic_mug.iv')

    #change the location so it's not under the robot
    T = self.target_kinbody.GetTransform()
    T[0:3,3] += np.array([0.5, 0.5, 0.5])
    self.target_kinbody.SetTransform(T)
    self.env.AddKinBody(self.target_kinbody)

    # create a grasping module
    self.gmodel = openravepy.databases.grasping.GraspingModel(self.robot, self.target_kinbody)

    # if you want to set options, e.g. friction
    options = openravepy.options
    options.friction = 0.1
    if not self.gmodel.load():
      self.gmodel.autogenerate(options)

    self.graspindices = self.gmodel.graspindices
    self.grasps = self.gmodel.grasps


  def order_grasps(self):
    self.grasps_ordered = self.grasps.copy() #you should change the order of self.grasps_ordered
    count = 0
    for grasp in self.grasps_ordered:
        #print count
        count += 1
        grasp[self.graspindices.get('performance')] = self.eval_grasp(grasp)

    # sort!
    #print "sort"
    order = np.argsort(self.grasps_ordered[:,self.graspindices.get('performance')[0]])
    order = order[::-1]
    self.grasps_ordered = self.grasps_ordered[order]
    #print "show grasp"
    # self.show_grasp(self.grasps_ordered[0], delay=10)
    for grasp in self.grasps_ordered[:4]:
      self.show_grasp(grasp, delay=10)



  # order the grasps - but instead of evaluating the grasp, evaluate random perturbations of the grasp
  def order_grasps_noisy(self):
    print "Order Grasps Noisy Reached!"
    # print "call into noisy"
    self.grasps_ordered_noisy = self.grasps.copy() #you should change the order of self.grasps_ordered_noisy
    sample_count = 5
    n = 0
    for grasp in self.grasps_ordered_noisy:
      # Get a couple of noisy samples
      n+=1
      #print (n)
      samples = []
      for x in range(sample_count):
        # Get a random grasp from our function below
        noisy_grasp = self.sample_random_grasp(grasp)
        # Evaluate it with our evaluation metric and append this to our sampled array
        samples.append(self.eval_grasp(noisy_grasp))
      # Average our scores from those samples and set the score of that grasp
      grasp[self.graspindices.get('performance')] = sum(samples) / sample_count
    # Sort the grasps based on their score
    #print order
    order = np.argsort(self.grasps_ordered_noisy[:,self.graspindices.get('performance')[0]])
    order = order[::-1]
    self.grasps_ordered_noisy = self.grasps_ordered_noisy[order]
    # self.grasps_ordered_noisy = self.grasps_ordered_noisy[np.argsort(self.grasps_ordered_noisy[:,self.graspindices.get('performance')[0]], kind='quicksort').reverse()
    #print "show grasp"
    # self.show_grasp(self.grasps_ordered[0], delay=10)
    for grasp in self.grasps_ordered_noisy[:4]:
      self.show_grasp(grasp, delay=10)

    #print "Order Grasps Noisy Finished!"

  # function to evaluate grasps
  # returns a score, which is some metric of the grasp
  # higher score should be a better grasp
  def eval_grasp(self, grasp):
    # print "call into evaluation function"
    with self.robot:
      #contacts is a 2d array, where contacts[i,0-2] are the positions of contact i and contacts[i,3-5] is the direction
      try:
        contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=False)

        obj_position = self.gmodel.target.GetTransform()[0:3,3]
        # for each contact
        #G = np.array([]) #the wrench matrix
        G = np.empty([2, 0])
        for c in contacts:
          pos = c[0:3] - obj_position
          dir = -c[3:] #this is already a unit vector

          cross = np.cross(pos, dir)
          g = np.vstack((dir, cross))
          #print g.shape
          G = np.hstack((G, g))

        # w, v = np.linalg.eig(np.dot(G, G.T))
        # w = sorted(w)
        # print w
        # return math.sqrt(w[0]) #change this
        # return 0
        #another metric
        det = np.linalg.det(np.dot(G, G.T))
        return math.sqrt(det)

      except openravepy.planning_error,e:
        #you get here if there is a failure in planning
        #example: if the hand is already intersecting the object at the initial position/orientation
        return  100000

      #heres an interface in case you want to manipulate things more specifically
      #NOTE for this assignment, your solutions cannot make use of graspingnoise
#      self.robot.SetTransform(np.eye(4)) # have to reset transform in order to remove randomness
#      self.robot.SetDOFValues(grasp[self.graspindices.get('igrasppreshape')], self.manip.GetGripperIndices())
#      self.robot.SetActiveDOFs(self.manip.GetGripperIndices(), self.robot.DOFAffine.X + self.robot.DOFAffine.Y + self.robot.DOFAffine.Z)
#      self.gmodel.grasper = openravepy.interfaces.Grasper(self.robot, friction=self.gmodel.grasper.friction, avoidlinks=[], plannername=None)
#      contacts, finalconfig, mindist, volume = self.gmodel.grasper.Grasp( \
#            direction             = grasp[self.graspindices.get('igraspdir')], \
#            roll                  = grasp[self.graspindices.get('igrasproll')], \
#            position              = grasp[self.graspindices.get('igrasppos')], \
#            standoff              = grasp[self.graspindices.get('igraspstandoff')], \
#            manipulatordirection  = grasp[self.graspindices.get('imanipulatordirection')], \
#            target                = self.target_kinbody, \
#            graspingnoise         = 0.0, \
#            forceclosure          = True, \
#            execute               = False, \
#            outputfinal           = True, \
#            translationstepmult   = None, \
#            finestep              = None )



  # given grasp_in, create a new grasp which is altered randomly
  # you can see the current position and direction of the grasp by:
  # grasp[self.graspindices.get('igrasppos')]
  # grasp[self.graspindices.get('igraspdir')]
  def sample_random_grasp(self, grasp_in):
    grasp = grasp_in.copy()

    #sample random position
    RAND_DIST_SIGMA = 0.04
    pos_orig = grasp[self.graspindices['igrasppos']]
    noise_pos = np.random.normal(0,RAND_DIST_SIGMA,3)
    #print (pos_orig)

    #sample random orientation
    RAND_ANGLE_SIGMA = np.pi/20
    dir_orig = grasp[self.graspindices['igraspdir']]
    noise_dir = np.random.normal(0,RAND_ANGLE_SIGMA,3)

    roll_orig = grasp[self.graspindices['igrasproll']]
    noise_roll = np.random.normal(0,RAND_ANGLE_SIGMA,3)


    pos_noisy = pos_orig + noise_pos
    roll_noisy = roll_orig + noise_roll
    dir_noisy = dir_orig + noise_dir


    grasp[self.graspindices['igrasppos']] = pos_noisy
    grasp[self.graspindices['igraspdir']] = dir_noisy
    grasp[self.graspindices['igrasproll']] = roll_noisy

    return grasp
  #displays the grasp
  def show_grasp(self, grasp, delay=1.5):
    with openravepy.RobotStateSaver(self.gmodel.robot):
      with self.gmodel.GripperVisibility(self.gmodel.manip):
        time.sleep(0.1) # let viewer update?
        try:
          with self.env:
            contacts,finalconfig,mindist,volume = self.gmodel.testGrasp(grasp=grasp,translate=True,forceclosure=True)
            #if mindist == 0:
            #  print 'grasp is not in force closure!'
            contactgraph = self.gmodel.drawContacts(contacts) if len(contacts) > 0 else None
            self.gmodel.robot.GetController().Reset(0)
            self.gmodel.robot.SetDOFValues(finalconfig[0])
            self.gmodel.robot.SetTransform(finalconfig[1])
            self.env.UpdatePublishedBodies()
            time.sleep(delay)
        except openravepy.planning_error,e:
          print 'bad grasp!',e

if __name__ == '__main__':
  robo = RoboHandler()
  #time.sleep(10000) #to keep the openrave window open
