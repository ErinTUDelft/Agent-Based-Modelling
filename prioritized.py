import time as timer
import random
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from itertools import permutations
from single_agent_planner import get_sum_of_cost


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
        #print("constraints before: ", constraints)
        #print("num of agents", self.num_of_agents)
        #print("starts", self.starts) # permutaties van self.start
      # for start in starts:
      #     i = 0
      
        self.perm = permutations(range(self.num_of_agents))
        print("perm", self.perm)
        for startverzameling in list(self.perm):
            print(startverzameling)
            constraints = []
            Uitgang = 0
            
            for iterable in range(self.num_of_agents):  # Find path for each agent
                #print(f"Now cycle {i} begins")
                
                i = startverzameling[iterable]
                path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                              i, constraints)
    
                if path is None:
                    #raise BaseException('No solutions')
                    Uitgang = 1
                    break
                    
                    
                    
                j = 0
        
                
                while j < (len(path)-1): # A path for one agent was found using A*, this
                                         # This now gets appended as constraints for the next agents
                    constraint = {'agent':  i, 'loc': [path[j],path[j+1]], 'timestep' : j}
                    j = j+1
                    constraints.append(constraint)
                #print("constraints after path: ", constraints, "\n")
            
            
                  
            if Uitgang == 1:
                continue
            #print("constraints: ", constraints)
            
            result.append(path)
            cost = get_sum_of_cost(path)
            print("cost: ", cost)
            #print("path:", path)

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
