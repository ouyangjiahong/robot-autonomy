from heapq import *

class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()


    def Plan(self, start_config, goal_config):

        plan = []
        
        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        start_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        plot = self.visualize and hasattr(self.planning_env, 'InitializePlot')

        if plot:
            self.planning_env.InitializePlot(goal_config)
        found = False
        tovisit = []
        visited = []
        parent = {}
        c2c = {}
        c2c[start_id] = 0
        #tovisit.append(start_id)
        heappush(tovisit,(0+self.planning_env.ComputeHeuristicCost(start_id,goal_id),start_id))
        while True:
            if len(tovisit)==0:
                print("No Path Found")
                break
            curr = heappop(tovisit)[1]
            #print("Visiting Node: ",curr)
            if curr == goal_id:
                found=True
                print("Path Found")
                break
            visited.append(curr)
            succ = self.planning_env.GetSuccessors(curr)
            for s in succ:
                if s not in visited: # resolve both cases now
                    #tovisit.append(s)
                    parent[s] = curr
                    c2c_tentative = c2c[curr] + self.planning_env.ComputeDistance(s,curr)
                    l = [h[1] for h in tovisit]
                    if s in l:
                        idx = l.index(s)
                        if c2c[s] > c2c_tentative: # new path is better
                            c2c[s] = c2c_tentative
                            tovisit[idx] = (c2c[s]+self.planning_env.ComputeHeuristicCost(s,goal_id),s)
                            heapify(tovisit)
                    else:
                        c2c[s] = c2c_tentative
                        heappush(tovisit,(c2c[s]+self.planning_env.ComputeHeuristicCost(s,goal_id),s))

                    if plot:
                        c1 = self.planning_env.discrete_env.NodeIdToConfiguration(curr)
                        c2 = self.planning_env.discrete_env.NodeIdToConfiguration(s)
                        self.planning_env.PlotEdge(c1,c2)

        if found:
            n = goal_id
            while n in parent.keys():
                plan.append(self.planning_env.discrete_env.NodeIdToConfiguration(n))
                n = parent[n]
            plan.append(start_config)
        plan = plan[::-1]
        #print("Plan: ",plan)
        if plot:
            for j in range(len(plan)-1):
                self.planning_env.PlotEdge(plan[j],plan[j+1])

        return plan
