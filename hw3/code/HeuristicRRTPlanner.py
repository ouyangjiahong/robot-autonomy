import numpy
from RRTTree import RRTTree

class HeuristicRRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        
    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        plot = self.visualize and hasattr(self.planning_env, 'InitializePlot')
        if plot:
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        c2c = {}
        copt = self.planning_env.ComputeHeuristicCost(start_id,goal_id)
        c2c[start_id] = copt
        cmax = copt
        self.planning_env.SetGoalParameters(goal_config)

        while True:
            while True:
                if numpy.random.rand()>self.planning_env.p:
                    qr = self.planning_env.GenerateRandomConfiguration()
                else:
                    qr = goal_config
                qn_id,qn = tree.GetNearestVertex(qr)
                qnN = self.planning_env.discrete_env.ConfigurationToNodeId(qn)
                mqn = 1.0 - (c2c[qnN]-copt+1e-5)/(cmax-copt+1e-5)
                if numpy.random.rand()>max(mqn,0.5):
                    break

            reached,qc = self.planning_env.Extend(qn,qr,step=epsilon)
            if qc is not None:
                qcN = self.planning_env.discrete_env.ConfigurationToNodeId(qc)
                c2c[qcN] = c2c[qnN] + self.planning_env.ComputeDistanceConfig(qc,qn)
                if c2c[qcN]>cmax:
                    cmax = c2c[qcN]
                qc_id = tree.AddVertex(qc)
                tree.AddEdge(qn_id,qc_id)
                if plot:
                    self.planning_env.PlotEdge(qn,qc)
                dist = self.planning_env.ComputeDistanceConfig(qc,goal_config)
                if (dist<=epsilon and reached):
                    goal_id = qc_id
                    break

        print("Number of vertices in tree: ",len(tree.vertices))

        plan.append(goal_config)
        v_id = goal_id
        while v_id != tree.GetRootId():
            next_id = tree.edges[v_id]
            plan.append(tree.vertices[next_id])
            v_id = next_id
        print("Path Length: ",len(plan))

        plan = plan[::-1]

        if plot:
            for j in range(len(plan)-1):
                self.planning_env.PlotEdge(plan[j],plan[j+1],color='r.-')

        return plan
