"""
This file contains the AircraftDistributed class that can be used to implement individual planning.
Code in this file is just provided as guidance, you are free to deviate from it.
"""

from single_agent_planner import a_star, get_location
from cbs import detect_collisions, standard_splitting
import random
from copy import deepcopy
from visualize import Animation
import time as timer

approach = "collision-based" # collision-based or prioritised-based

class AircraftDistributed(object):
    """Aircraft object to be used in the distributed planner."""

    visible_area_range = 4
    number_of_steps_to_plan = 5
    big_M = 10**10

    def __init__(self, my_map, start, goal, heuristics, agent_id):
        """
        my_map   - list of lists specifying obstacle positions
        starts      - (x1, y1) start location
        goals       - (x1, y1) goal location
        heuristics  - heuristic to goal location
        """

        self.my_map = my_map
        self.start = start
        self.goal = goal
        self.id = agent_id
        self.heuristics = heuristics
        self.location = self.start
        
        self.travelled_path = []
        self.constraints = []
        self.intended_path = []
        self.used_paths = []
        
        self.constraints_under_consideration = []
        self.path_under_consideration = []
        
        self.money = 10 
        self.utility_factor = 0.8
        
        self.has_reached_goal = False
        self.path_unchanged = True
        
        self.update_path(self.calculate_new_path())
        
        # Double check the problem is solvable.
        if self.intended_path == None:
            raise BaseException("Problem unsolvable, no inital path can be found.")
        
    def update_path(self, new_path):
        """ Updates the intended path and adds it to the used paths. 
        
        new_path    - New path to use
        """
        self.intended_path = deepcopy(new_path)
        self.used_paths.append(deepcopy(new_path))
        
    def get_current_time(self):
        """ Uses the travelled path to return the current timestep.
        """
        time = len(self.travelled_path) - 1
        
        return time
        
    def move(self):
        """ Moves the agent one step forward along the intended path.
        If the intended path is empty (e.g. agent is at goal location),
        the agent stays at its current location.
        """
        # If there is no intention to move, the agent should stay at the final
        # location of the travelled path.
        if self.intended_path == []:
            new_location = self.travelled_path[-1]
        else:
            new_location = self.intended_path.pop(0)
        
        # Set the new location and add the location to the travelled path.
        self.location = new_location
        self.travelled_path.append(new_location)
        
    def calculate_new_path(self, additional_constraints=[]):
        """ Uses A* to calculate a new possible path for an agent.
        
        additional_constraints  - Additional constraints to consider for
                                    determining the new path.
        """
        timestep = self.get_current_time()
        
        path = a_star(self.my_map, self.location, self.goal, 
                      self.heuristics, self.id, 
                      self.constraints + additional_constraints, 
                      timestep)
        
        # A* uses the current location as the start location. However,
        # the current location is already in the travelled path, and should
        # thus not be added to the intended path.
        # Except if this is the initialisation, when there is no travelled
        # path yet.
        #
        # was path == None was added to catch errors.
        if not self.travelled_path == [] and not path == None:
            path.pop(0)
        return path
    
    def run_agent_radar(self, agents):
        """ Returns the visible agents in range of the agent. 
        
        agents  - Agents whose location should be checked
        """
        
        # Determining the visible area. Currently the area is assumed square.
        visible_area = (range(self.location[0] - self.visible_area_range,
                              self.location[0] + self.visible_area_range), 
                        range(self.location[1] - self.visible_area_range,
                              self.location[1] + self.visible_area_range))
        
        visible_agents = []
        
        # We loop through the locations of the other agents.
        for agent in agents:
            # Of course the agent can see itself.
            if agent.id == self.id:
                continue
            
            # Check if the agent can see another agent.
            if agent.location[0] in visible_area[0] and \
                agent.location[1] in visible_area[1]:
                visible_agents.append(agent)
                
        return visible_agents
    
    # Conflict determination
    def share_intended_path(self, number_of_steps):
        """ Returns the agent's intended path. 
        
        number_of_steps     - Number of steps to share
        """
        intended_path = self.intended_path[0:number_of_steps]
        
        # If there is no intention to move, the final location of the travelled
        # path should be returned.
        if intended_path == []:
            intended_path = [self.travelled_path[-1]]
            
        return intended_path
    
    def share_path_under_consideration(self, number_of_steps):
        """ Returns the path the agent is considering. 
        
        number_of_steps     - Number of steps to share
        """
        intended_path = self.path_under_consideration[0:number_of_steps]
        
        # If there is no intention to move, the final location of the travelled
        # path should be returned.
        if intended_path == []:
            intended_path = [self.travelled_path[-1]]
            
        return intended_path
    
    def check_conflict(self, visible_agent):
        """ Returns the collisions with the visible_agent for the intended path.
        
        visible_agent   - Visible agent to check against for conflict.
        """
        # Obtain the paths of both agents. (Up to max. number of steps to plan)
        # Note that the current location must be added for sake of completeness
        own_path = [self.location] + self.share_intended_path(self.number_of_steps_to_plan)
        other_path = [visible_agent.location] + visible_agent.share_intended_path(self.number_of_steps_to_plan)
        
        # Get the current timestep (necessary for proper constraint definition)
        timestep = self.get_current_time()
        
        # Detect collisions.
        collisions = detect_collisions([own_path, other_path], timestep, 
                                       [self.id, visible_agent.id])
        
        return collisions
    
    def check_conflict_under_consideration(self, visible_agent):
        """ Returns the collisions with the visible_agent for the agent's new
        path it is considering.
        
        visible_agent   - Visible agent to check against for conflict.
        """
        # When the new path under consideration is longer than the intended path,
        # the agent needs to ensure those extra steps do not cause conflict.
        # See report for full explanation.
        extra_steps_to_check = len(self.path_under_consideration) - len(self.intended_path)
        number_of_steps = self.number_of_steps_to_plan + extra_steps_to_check
        
        # Obtain the paths of both agents. (Up to max. number of steps to plan)
        # Note that the current location must be added for sake of completeness
        own_path = [self.location] + self.share_path_under_consideration(number_of_steps)
        other_path = [visible_agent.location] + visible_agent.share_intended_path(number_of_steps)
        
        # Get the current timestep (necessary for proper constraint definition)
        timestep = self.get_current_time()
        
        # Detect collisions.
        collisions = detect_collisions([own_path, other_path], timestep, 
                                       [self.id, visible_agent.id])
        return collisions
    
    def consider_new_path(self, constraints):
        """ Lets an agent consider a new path, with the addition of certain constraints.
        Returns the added cost (multiplied by utility factor) of the path under consideration.
        
        constraints     - Constraints to add for the calculation of the new path.
        """
        #Finds the length of the original intended path
        length_original_path = len(self.intended_path) 
        
        # Add the extra constraints to the constraints under consideration.
        for constraint in constraints:
            self.constraints_under_consideration.append(constraint)
            
        # Calculate the new path.
        path_to_consider = self.calculate_new_path(self.constraints_under_consideration)

        # If A* finds no solution, return a significantly large cost. The agent
        # will then win the negotiation (in most cases).
        if path_to_consider == None:
            return self.big_M * self.utility_factor
        
        self.path_under_consideration = path_to_consider
        length_new_path = len(self.path_under_consideration)
        added_cost = length_new_path - length_original_path
        
        return added_cost*self.utility_factor
    
    def negotiate_new_path(self, visible_agent, collisions, start_time, cutoff_time):
        """ Method to let an agent negotiate with a visible agent to solve a
        conflict and thus avoid a collision.
        
        visible_agent   - Agent to negotiate with
        collisions      - Collisions with visible_agent
        start_time      - CPU start time of the simulation
        cutoff_time     - CPU cutoff time of the simulation        
        """
        
        # print("Negotation started by agent", self.id, "with agent", visible_agent.id)
        
        # Check collisions
        own_collisions = collisions
        opponents_collisions = visible_agent.check_conflict(self)
        agent_collisions = [own_collisions, opponents_collisions]
        
        # Set up paths under consideration to match the intended path.
        self.path_under_consideration = self.intended_path
        visible_agent.path_under_consideration = visible_agent.intended_path
        
        agents = [self, visible_agent]
        costs = [0, 0]
        
        # For both agents, determine a possible path that they could offer in
        # the bidding. These paths must be conflict-free. For that reason, they
        # loop until collisions == [].
        for i, agent in enumerate(agents):
            
            other_agent = agents[not i]
            
            while not agent_collisions[i] == []:
                
                # Cut off if iteration takes too long.
                CPU_time = timer.time() - start_time
                if CPU_time > cutoff_time:
                    return True
                
                constraints = []
                
                # When the new path under consideration is longer than the intended path,
                # the agent needs to ensure those extra steps do not cause conflict.
                # See report for full explanation.
                extra_steps_to_check = len(agent.path_under_consideration) - len(agent.intended_path)
                
                # It is possible to use two different approaches. One can either apply
                # constraints based on collisions (more a CBS approach) or apply constraints
                # based on the other agent's full path (more a prioritised planning approach).
                # In this case, the prioritised-based approach broke late in the process,
                # and still needs to be fixed. Preliminary results showed the collision-
                # based approach to be less computationally intensive and more optimal.                
                if approach == "collision-based":
                    for collision in agent_collisions[i]:
                        
                        # If the collision is outside of the range of number of steps
                        # to plan, we can disregard it.
                        if collision['timestep'] - agent.get_current_time() > \
                            agent.number_of_steps_to_plan + extra_steps_to_check:
                            continue
                        collision_split = standard_splitting(collision)
                        constraints.append(collision_split[0])
                        
                if approach == "prioritised-based":
                    
                    for i, location in enumerate(other_agent.intended_path[0:agent.number_of_steps_to_plan
                                                                             + extra_steps_to_check]):
                        constraints.append({
                        'agent': agent.id,
                        'loc': [location],
                        'timestep': agent.get_current_time() + i + 1})
                        constraints.append({
                        'agent': agent.id,
                        'loc': [location, get_location(other_agent.intended_path, i-1)],
                        'timestep': agent.get_current_time() + i + 1})
                
                # We determine the cost of the new path.
                costs[i] = agent.consider_new_path(constraints)
                
                # If the cost is massive, we are at the limits of what we can offer.
                # See consider_new_path() for more.
                if costs[i] == agent.big_M * agent.utility_factor:
                    break
                
                # We check again for collisions.
                agent_collisions[i] = agent.check_conflict_under_consideration(other_agent)
        
        # We extract the costs from the costs list.
        cost_of_own_new_path = costs[0]
        cost_of_opponents_new_path = costs[1]
        
        # Find whether the bid gets accepted or not
        accept = False
        if cost_of_own_new_path > cost_of_opponents_new_path:
            accept = True
            
        # If an agent has no money, it must accept the other agent's proposal.
        # Except if that agent is in even worse debt than the first agent.
        if self.money < 0 and self.money < visible_agent.money:
            accept = False
        elif visible_agent.money < 0 and visible_agent.money < self.money:
            accept = True
            
        # If the bids are the same, a coin toss determines who gets the bid.
        if abs(cost_of_own_new_path - cost_of_opponents_new_path) < 0.001: # Floating point correction
            flip = random.randint(0,1)
            if flip == 0:
                accept = True
            else:
                accept = False
        
        if accept:
            # Visible agent will use a new path
            visible_agent.constraints += visible_agent.constraints_under_consideration
            visible_agent.update_path(visible_agent.calculate_new_path())
            visible_agent.path_unchanged = False
            self.money -= cost_of_opponents_new_path
            visible_agent.money += cost_of_opponents_new_path
            
            # Double check the new constraints are feasible
            if visible_agent.intended_path == None:
                return True
            
            # print(f"Agent {visible_agent.id} accepts my (agent: {self.id}) proposal and will change course ".format())
        else:
            # Current agent will use a new path
            self.constraints += self.constraints_under_consideration
            self.update_path(self.calculate_new_path())
            self.path_unchanged = False
            visible_agent.money -= cost_of_own_new_path
            self.money += cost_of_own_new_path
            
            # Double check the new constraints are feasible
            if self.intended_path == None:
                return True
            # print(f"Agent {self.id} accepts my (agent: {visible_agent.id}) proposal and will change course ")
            
        self.constraints_under_consideration = []
        visible_agent.constraints_under_consideration = []