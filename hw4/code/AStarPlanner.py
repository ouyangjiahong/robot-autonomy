from heapq import *
import time
from SimpleEnvironment import Action
import numpy as np

class AStarPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()


    def Plan(self, start_config, goal_config):
        t1 = time.time()
        plan = []

        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        plot = self.visualize and hasattr(self.planning_env, 'InitializePlot')
        print "start_id: ",start_id
        print "goal_id: ",goal_id
        # Heuristic weight
        k = 5
        if plot:
            self.planning_env.InitializePlot(goal_config)
        found = False
        tovisit = []
        visited = []
        parent = {}
        c2c = {}
        c2c[start_id] = 0
        numexpand = 0
        #tovisit.append(start_id)
        heappush(tovisit,(0+self.planning_env.ComputeHeuristicCost(start_id,goal_id),start_id))
        while True:
            #print("length of tovisit: ",len(tovisit))
            if len(tovisit)==0:
                print("No Path Found")
                break
            curr = heappop(tovisit)[1]
            print("curr: ",curr)
            numexpand = numexpand + 1
            if curr == goal_id:
                found=True
                print("Path Found")
                break
            visited.append(curr)
            succ = self.planning_env.GetSuccessors(curr)
            #print("successors: ",succ)
            for s in succ:
                sid = s["id"]
                sctrl = s["control"]
                if sid not in visited:
                    c2c_tentative = c2c[curr] + self.planning_env.ComputeDistance(sid,curr)
                    l = [h[1] for h in tovisit]
                    if sid in l: # already exists in tovisit, but new path might be better
                        idx = l.index(sid)
                        if c2c[sid] > c2c_tentative: # new path is better
                            parent[sid] = {"id":curr,"control":sctrl}
                            c2c[sid] = c2c_tentative
                            tovisit[idx] = (c2c[sid]+k*self.planning_env.ComputeHeuristicCost(sid,goal_id),sid)
                            heapify(tovisit)
                    else:
                        parent[sid] = {"id":curr,"control":sctrl}
                        c2c[sid] = c2c_tentative
                        heappush(tovisit,(c2c[sid]+k*self.planning_env.ComputeHeuristicCost(sid,goal_id),sid))

                    if plot:
                        pass
                        #c1 = self.planning_env.discrete_env.NodeIdToConfiguration(curr)
                        #c2 = self.planning_env.discrete_env.NodeIdToConfiguration(sid)
                        #self.planning_env.PlotEdge(c1,c2)
        # import IPython
        # IPython.embed()

        if found:
            n = goal_id
            while n in parent.keys():
                n,ctrl = parent[n]["id"],parent[n]["control"]
                ncfg = self.planning_env.discrete_env.NodeIdToConfiguration(n)
                action = Action(ctrl,self.planning_env.GenerateFootprintFromControl(np.array(ncfg),ctrl))
                plan.append(action)
            #plan.append(start_config)
        plan = plan[::-1]
        #print("Plan: ",plan)
        #if plot:
        #    for j in range(len(plan)-1):
        #        self.planning_env.PlotEdge(plan[j],plan[j+1],'r.-')

        t2 = time.time()
        print("Time Taken for Planning: ",t2-t1)
        print("Path Length: ",len(plan))
        print("Number of Nodes Expanded: ",numexpand)

        # import IPython
        # IPython.embed()

        return plan
