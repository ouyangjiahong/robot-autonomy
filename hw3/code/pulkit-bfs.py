import pdb
import numpy

class BreadthFirstPlanner(object): 

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

    def Plan(self, start_config, goal_config):
        # TODO: Here you will implement the depth first planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        plan = []
        explored = []
        queue = []
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)

        print "Printing start!!"
        #print start_config

        queue.append([list(start_config)])
        while queue:
            path = queue.pop(0)
            node = path[-1]
            #print "path:", path
            #print "node:", node
            if node not in explored:
                neighbours = self.planning_env.GetSuccessors(self.planning_env.discrete_env.ConfigurationToNodeId(node))
                for neighbour in neighbours:
                    neighbour = self.planning_env.discrete_env.NodeIdToConfiguration(neighbour)
                    plan = [list(path)]
                    plan.append(neighbour)
                    queue.append(plan)
                    #pdb.set_trace()
                    #print "Printing, Node", node
                    #print "Printing, Neighbour", neighbour
                    #print "Printing, All Neighbours", neighbours
                    self.planning_env.PlotEdge(node, neighbour)
                    if (numpy.array(neighbour) == goal_config).all():
                        print "Returning the Planned Path!!!"
                        return plan
                explored.append(node)
                #pdb.set_trace()
        print "Path Not found!!"
        plan = []
        plan.append(start_config)
        plan.append(goal_config)
        return plan
