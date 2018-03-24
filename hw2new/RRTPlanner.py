import numpy
from RRTTree import RRTTree

class RRTPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        

    def Plan(self, start_config, goal_config, epsilon = 0.001):
        
        tree = RRTTree(self.planning_env, start_config)
        plan = []
        plotedge = False
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            plotedge = True
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        self.planning_env.SetGoalParameters(goal_config)
        
        while True:
            if numpy.random.rand()>self.planning_env.p:
                qr = self.planning_env.GenerateRandomConfiguration()
            else:
                qr = goal_config

            qn_id,qn = tree.GetNearestVertex(qr)
            reached,qc = self.planning_env.Extend(qn,qr,step=epsilon)
            if qc is not None:
                qc_id = tree.AddVertex(qc)
                tree.AddEdge(qn_id,qc_id)
                #plan.append(qc)
                if plotedge:
                    self.planning_env.PlotEdge(qn,qc)
		dist = self.planning_env.ComputeDistance(qc,goal_config)
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
        return numpy.array(plan[::-1])