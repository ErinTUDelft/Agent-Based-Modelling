import time as timer
import random
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


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

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []

        constraints = []
        predefined_constraints = [{'agent': 1,
                        'loc': [(7,2),(6,2)],
                        'timestep': 1}, {'agent': 1,
                                        'loc': [(7,2),(6,2)],
                                        'timestep': 2}, {'agent': 3,
                                                        'loc': [(7,2),(6,2)],
                                                        'timestep': 3},{'agent': 3,
                                                                        'loc': [(7,2),(6,2)],
                                                                        'timestep': 4}, {'agent': 3,
                                                                                        'loc': [(7,2),(6,2)],
                                                                                        'timestep': 5}]
                        
                       
                       
                       
                     
                                                                                                                                                 
        constraints = [] #predefined_constraints
        # constraints.append({'agent': 3,
        #                 'loc': [(6,2)],
        #                 'timestep': 3})
        print("constraints before: ", constraints)
        print("num of agents", self.num_of_agents)
      
        self.pathfinding_map = self.my_map.copy()

        for i in range(self.num_of_agents):  # Find path for each agent
            print(f"Now cycle {i} begins")
            path = a_star(self.pathfinding_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            
            # print("path:", path)
            # for time, loc in enumerate(path):
            # #     print(time)
            # #     print("Value: ", loc)
            #       constraint = {'agent': 1, 'loc': [loc], 'timestep': time+1 }
            #       constraints.append(constraint)
            print("starts: " ,self.starts)
            if path is None:
                raise BaseException('No solutions')
      
            j = 0
            print("path is: ", path)
            print(len(path))
            # for agentid in range (self.num_of_agents):
            #     if agentid == i:
            #         continue
                
            #     while j < (len(path)-1):
            #         constraint = {'agent':  agentid, 'loc': [path[j],path[j+1]], 'timestep' : j}
            #         j = j+1
            #         constraints.append(constraint)
            #         print("constraints after path: ", constraints, "\n")
            
            
            while j < (len(path)-1): # A path for one agent was found using A*, this
                                     # This now gets appended as constraints for the next agents
                constraint = {'agent':  i, 'loc': [path[j],path[j+1]], 'timestep' : j}
                j = j+1
                constraints.append(constraint)
                self.pathfinding_map[path[-1][0]][path[-1][1]] = True
            print("constraints after path: ", constraints, "\n")
            
            
                  
                  
            print("constraints: ", constraints)
            
            result.append(path)
            print("path:", path)


            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches


            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result