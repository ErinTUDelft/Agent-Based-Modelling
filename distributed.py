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

    def __init__(self, my_map, starts, goals, CPU_cutoff_time):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        CPU_cutoff_time - Cutoff time to prevent excessive long runs
        """
        
        # Some general constants are defined, as well as the import from the
        # map, giving the start and goal locations and the heuristics.
        self.CPU_time = 0
        self.CPU_cutoff_time = CPU_cutoff_time
        self.t_max = 190
        
        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))
        
    def find_solution(self):
        """
        Finds paths for all agents from start to goal locations. 
        
        Returns:
            result (list): with a path [(s,t), .....] for each agent.
        """
        # Initialize constants       
        start_time = timer.time()
        result = []
        agents = []
        
        # Create agent objects with AircraftDistributed class
        for i in range(self.num_of_agents):
            agents.append(AircraftDistributed(self.my_map, self.starts[i], 
                                              self.goals[i], self.heuristics[i], i))
        
        # Some initialisation is performed to set up the iteration loop.
        timestep = 0
        goals_reached = self.num_of_agents*[False]
        stop = False
        
        while timestep < self.t_max:
            
            # The general process of the distributed planning is as follows:
            # Step 0: We select an agent
            # Step 1: The agent looks for agents in visible range
            # Step 2: If agents found, agent asks visible agents for the intended path
            # Step 3: If conflict in path, negotiate/solve conflict
            # Step 4: Stick to new path, check conflicts again
            # Step 5: Walk path
            # Repeat            
            
            # If the cutoff time is met, we exit the loop.
            self.CPU_time = timer.time() - start_time
            if self.CPU_time > self.CPU_cutoff_time:
                return None
            
            # Variable to check if agent needs to check for conflicts.
            paths_unchanged = self.num_of_agents*[False]
            
            # Agents move and check if they have met their goal location.
            for agent in agents:
                agent.move()
                agent.path_unchanged = False
                if agent.location == agent.goal:
                    goals_reached[agent.id] = True
                    
            # If all goals reached, planning complete!
            if all(goals_reached) == True:
                break
            
            # As long as agents have changed paths, we must check for conflicts.
            while not all(paths_unchanged) == True:
                for agent in agents:
                    
                    # If path is unchanged, no need to check for conflicts.
                    if agent.path_unchanged:
                        continue
                    
                    # Check for visible agents.
                    visible_agents = agent.run_agent_radar(agents)
                    
                    # If there are no conflicts in sight, we don't need to change the path.
                    if visible_agents == []:
                        agent.path_unchanged = True
                        continue
                    
                    # As long as the agent's path is in conflict with a visible agent,
                    # it needs to negotiate for a new path.
                    
                    for visible_agent in visible_agents:
                        collisions = agent.check_conflict(visible_agent)
                        
                        # The agent's path has not changed, except if they must
                        # during the upcoming negotiation.
                        agent.path_unchanged = True
                        
                        # IF the cutoff time is met, we abort the simulation.
                        # Note that multiple of these if-statements are required,
                        # as the simulation can be looping in a vast number of locations.
                        self.CPU_time = timer.time() - start_time
                        if self.CPU_time > self.CPU_cutoff_time:
                            return None
                            
                        # As long as there are collisions, there must be negotiation
                        # going on.
                        while not collisions == []:
                            
                            stop = agent.negotiate_new_path(visible_agent,collisions, start_time, self.CPU_cutoff_time)
                            
                            # The negotiation process can also issue a stop command
                            # to abort the simulation.
                            if stop:
                                return None
                            collisions = agent.check_conflict(visible_agent) # Return conflict
                
                # We transfer the agent attributes to lists.
                for agent in agents:
                    if agent.path_unchanged:
                        paths_unchanged[agent.id] = True
                    else:
                        paths_unchanged[agent.id] = False
                    goals_reached[agent.id] = False
                        
            timestep += 1
            
        # Results are appended to the result list.    
        for agent in agents:
            result.append(agent.travelled_path)
        
        # Print final output
        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))  # Hint: think about how cost is defined in your implementation
        print(result)
     
        
        return result  # Hint: this should be the final result of the distributed planning (visualization is done after planning)