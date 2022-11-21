"""
This file contains a placeholder for the DistributedPlanningSolver class that can be used to implement distributed planning.
Code in this file is just provided as guidance, you are free to deviate from it.
"""

import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost
from aircraft import AircraftDistributed
from cbs import detect_collision, detect_collisions

class DistributedPlanningSolver(object):
    """A distributed planner"""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """
        self.CPU_time = 0
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))
        # T.B.D.
        
    def find_solution(self):
        """
        Finds paths for all agents from start to goal locations. 
        
        Returns:
            result (list): with a path [(s,t), .....] for each agent.
        """
        # Initialize constants       
        start_time = timer.time()
        result = []
        self.CPU_time = timer.time() - start_time
        
        agents = []
        
        #print(self.starts, self.goals, self.heuristics)
        
        # Create agent objects with AircraftDistributed class
        for i in range(self.num_of_agents):
            agents.append(AircraftDistributed(self.my_map, self.starts[i], 
                                              self.goals[i], self.heuristics[i], i))
        
        #print(agents)
        
        timestep = 0
        goals_reached = self.num_of_agents*[False]
        
        while timestep < 190:
            
            # Step 1: Look for agents in visible range
            # Step 2: If agents found, ask for path intent.
            # Step 3: If conflict in path, negotiate/solve conflict
            # Step 4: Stick to new path, check conflicts again
                ### TODO: Find way to solve conflicts / negotiate
            # Step 5: Walk path
            
            paths_unchanged = self.num_of_agents*[False]
            
            for agent in agents:
                agent.move(timestep)
                agent.path_unchanged = False
                if agent.location == agent.goal:
                    goals_reached[agent.id] = True
                    
            if all(goals_reached) == True:
                break
            
            
            while not all(paths_unchanged) == True:
                for agent in agents:
                    if agent.path_unchanged:
                        continue
                    
                    #print("Travelled path agent", agent.id, agent.travelled_path)
                    visible_agents = agent.run_agent_radar(agents)
                    
                    for visible_agent in visible_agents:
                        # while path van agent in conflict is met een visible_agent:
                            # Zoek nieuw path
                        collisions = agent.check_conflict(visible_agent) # Return conflict
                            # agent.ask_path(visible_agent)
                        
                        agent.path_unchanged = True
                            
                        while not collisions == []:
                            agent.negotiate_new_path(visible_agent,collisions)
                                #visible_agent.respond_to_negotiation()
                                #agent.update_path()
                            collisions = agent.check_conflict(visible_agent) # Return conflict
                            
                for agent in agents:
                    if agent.path_unchanged:
                        paths_unchanged[agent.id] = True
                    else:
                        paths_unchanged[agent.id] = False
                        
            
            
            
            timestep += 1
            
        for agent in agents:
            result.append(agent.travelled_path)
            print("Money for agent", agent.id, agent.money)
        
        
        # Print final output
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))  # Hint: think about how cost is defined in your implementation
        print(result)
     
        
        return result  # Hint: this should be the final result of the distributed planning (visualization is done after planning)