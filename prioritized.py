import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from copy import deepcopy


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.M = 20
        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))
        # print("heuristics:", self.heuristics)

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        self.pathfinding_map = deepcopy(self.my_map)

        for i in range(self.num_of_agents):  # Find path for each agent
            path = a_star(self.pathfinding_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            #print("path", path)
            if path is None:
                raise BaseException('No solutions')
            
            j = 0
            while j < (len(path)-1): # A path for one agent was found using A*, this
                                     # This now gets appended as constraints for the next agents
                #print("Doing while loop")
                for other_agent in range(i+1, self.num_of_agents):
                    #additional = 0
                    #print("works")
                    vertex_constraint = {'agent':  other_agent, 'loc': [path[j+1],], 'timestep' : j+1}
                    edge_constraint = {'agent':  other_agent, 'loc': [path[j+1],path[j]], 'timestep' : j+1}
                    constraints.append(vertex_constraint)
                    constraints.append(edge_constraint)
                    
                    # for additional in range(self.M):
                    #     vertex_constraint = {'agent':  other_agent, 'loc': [path[j+1],], 'timestep' : j+2+additional}
                    #     constraints.append(vertex_constraint)
                    
                                    
                j = j + 1
                #print("constraints", constraints)
                #print('j', j)
            #print("path", path)
            self.pathfinding_map[path[-1][0]][path[-1][1]] = True
            self.heuristics = []
            for goal in self.goals:
                self.heuristics.append(compute_heuristics(self.pathfinding_map, goal))
            if len(path) > self.M:
                pass #raise BaseException('No solutions')
                
            result.append(path)
            
                       
            

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        # print(result)
        return result