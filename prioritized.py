import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from copy import deepcopy



class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals, CPU_cutoff_time):
        """my_map       - list of lists specifying obstacle positions
        starts          - [(x1, y1), (x2, y2), ...] list of start locations
        goals           - [(x1, y1), (x2, y2), ...] list of goal locations
        CPU_cutoff_time - Cutoff time to prevent excessive long runs
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.CPU_time = 0
        self.CPU_cutoff_time = CPU_cutoff_time

        # Compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        
        # We use the pathfinding map to block out goal locations where other agents have settled.
        self.pathfinding_map = deepcopy(self.my_map)

        # Find path for each agent
        for i in range(self.num_of_agents): 
            path = a_star(self.pathfinding_map, self.starts[i], self.goals[i], self.heuristics[i],
                           i, constraints)
            
            # If there is no solution, the planning has failed.
            if path == None:
                return None
            
            # If the allowed time has elapsed, the simulation is aborted.
            if timer.time() - start_time > self.CPU_cutoff_time:
                return None

            # Add vertex and edge constraints for the path traversed by other agents.
            j = 0
            while j < (len(path)-1): 
                for other_agent in range(i+1, self.num_of_agents):
                    vertex_constraint = {'agent':  other_agent, 'loc': [path[j+1],], 'timestep' : j+1}
                    edge_constraint = {'agent':  other_agent, 'loc': [path[j+1],path[j]], 'timestep' : j+1}
                    constraints.append(vertex_constraint)
                    constraints.append(edge_constraint)
                    
              
                j += 1
            
            # Once an agent is at its goal location, that location is blocked out
            # on the pathfinding map for other agents.
            self.pathfinding_map[path[-1][0]][path[-1][1]] = True
            
            # The heuristics must then be recalculated.
            self.heuristics = []
            for goal in self.goals:
                self.heuristics.append(compute_heuristics(self.pathfinding_map, goal))
 
            result.append(path)
        

        #print("\n Found a solution! \n")
        #print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        # print(result)
        return result