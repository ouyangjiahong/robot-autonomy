import numpy, operator
from RRTPlanner import RRTTree

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize


    def Plan(self, start_config, goal_config, epsilon = 0.001):

        ftree = RRTTree(self.planning_env, start_config)
        rtree = RRTTree(self.planning_env, goal_config)
        plan = []
        plotedge = False
        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            plotedge = True
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        self.planning_env.SetGoalParameters(goal_config)

        while True:
            print("generating random config")
            qr = self.planning_env.GenerateRandomConfiguration()

            fqn_id,fqn = ftree.GetNearestVertex(qr)
            print("going into extend")
            # freached,fqc = self.planning_env.Extend(fqn,qr,step=epsilon)
            freached,fqc = self.planning_env.Extend(fqn,qr)

            if fqc is not None: # not Trapped
                fqc_id = ftree.AddVertex(fqc)
                ftree.AddEdge(fqn_id,fqc_id)
                if plotedge:
                    self.planning_env.PlotEdge(fqn,fqc)

                rqn_id,rqn = rtree.GetNearestVertex(fqc)
                print("going into extend")
                # rreached,rqc = self.planning_env.Extend(rqn,fqc,step=epsilon)
                rreached,rqc = self.planning_env.Extend(rqn,fqc)
                if rqc is not None:
                    rqc_id = rtree.AddVertex(rqc)
                    rtree.AddEdge(rqn_id,rqc_id)
                    if plotedge:
                        self.planning_env.PlotEdge(rqn,rqc)

                if (rreached): # work here is done
                    # dist = self.planning_env.ComputeDistance(rtree.vertices[0],goal_config)
                    #goal_id = qc_id
                    if (rtree.vertices[0]==goal_config).all():
                    # if dist < 0.01:
                        same = True
                        bridge_id_r = rqc_id
                        bridge_id_f = fqc_id
                    else:
                        same = False
                        actual_rtree = ftree
                        actual_ftree = rtree
                        bridge_id_r = fqc_id
                        bridge_id_r = rqc_id
                    break

            (ftree,rtree) = (rtree,ftree)

        if not same:
            (rtree,ftree) = (ftree,rtree)

        #print(ftree.vertices)
        #print(ftree.edges.items())

        #print(rtree.vertices)
        #print(rtree.edges.items())
	    print("Number of vertices in tree: ",len(ftree.vertices)+len(rtree.vertices))
        """
        plan.append(goal_config)
        v_id = goal_id
        while v_id != tree.GetRootId():
            next_id = tree.edges[v_id]
            plan.append(tree.vertices[next_id])
            v_id = next_id
        """
        plan.append(fqc)
        v_id = fqc_id
        while v_id != ftree.GetRootId():
            next_id = ftree.edges[v_id]
            plan.append(ftree.vertices[next_id])
            v_id = next_id

        plan = plan[::-1]

        v_id = rqc_id
        while v_id != rtree.GetRootId():
            next_id = rtree.edges[v_id]
            plan.append(rtree.vertices[next_id])
            v_id = next_id
	    print("Path Length: ",len(plan))
        return numpy.array(plan)
