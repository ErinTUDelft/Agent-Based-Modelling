"""
This file contains the AircraftDistributed class that can be used to implement individual planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

from single_agent_planner import a_star, get_location
from cbs import detect_collisions, standard_splitting
import random

class AircraftDistributed(object):
    """Aircraft object to be used in the distributed planner."""

    visible_area_range = 3
    number_of_steps_to_plan = 4

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
        self.intended_path = []
        self.used_paths = []
        self.constraints = []
        self.money = 10 #in thousands, but completely arbitrary ofcourse
        
        self.update_path(self.calculate_new_path())
        
    def update_path(self, new_path):
        self.intended_path = new_path
        self.used_paths.append(new_path)
        

        
    def update_location(self, new_location):
        self.location = new_location
        
    def get_current_time(self):
        # Uses the travelled path to determine the current timestep.
        return len(self.travelled_path)-1
        
    def move(self, timestep):
        # If there is no intention to move, the agent should stay at the final
        # location of the travelled path.
        if self.intended_path == []:
            new_location = self.travelled_path[-1]
        else:
            new_location = self.intended_path.pop(0)
        
        # Set the new location and add the location to the travelled path.
        self.location = new_location
        self.travelled_path.append(new_location)
        
    def calculate_new_path(self):
        # Use A* to calculate a new possible path.
        path = a_star(self.my_map, self.location, self.goal, 
                      self.heuristics, self.id, self.constraints, 
                      self.get_current_time())
        return path
    
    def run_agent_radar(self, agents):
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
                print("Saw ya", agent.id)
                visible_agents.append(agent)
                
        return visible_agents
    
    # Conflict determination (Jelmer)
    def share_intended_path(self):
        # Function which returns the intended path of a variable.
        intended_path = self.intended_path
        
        # If there is no intention to move, the final location of the travelled
        # path should be returned.
        if intended_path == []:
            intended_path = [self.travelled_path[-1]]
            
        return intended_path
    
    def check_conflict(self, visible_agent):
        # Obtain the paths of both agents. (Up to max. number of steps to plan)
        own_path = self.share_intended_path()[0:self.number_of_steps_to_plan]
        other_path = visible_agent.share_intended_path()[0:self.number_of_steps_to_plan]
        
        # Get the current timestep (necessary for proper constraint definition)
        timestep = self.get_current_time()
        
        # Detect collisions.
        collisions = detect_collisions([own_path, other_path], timestep, 
                                       [self.id, visible_agent.id])
        return collisions
    
    
    # Negotiation (Erin)
    
    def proposal(self, visible_agent,option):
        length_original_path = len(self.intended_path) #finds the length of the original intended path
        
        #Adds the conflict of interest to the constraint table
        self.constraints.append(option)
        length_new_path = len(self.calculate_new_path())
        
        added_cost = length_new_path - length_original_path
        utility_factor = 0.80 #this can be changed to any arbitrary value, can also be agent dependant
        
        return added_cost*utility_factor
        
        
    def respond_to_negotiation(self):
        

        # What is known?
        # The location of the current agent
        # The location of the visible agent
        # The location of the conflict
        # The timestep of the conflict
        # The location of an alternative route
        # What the extra cost is of the alternative route
        pass #TODO
    
    def negotiate_new_path(self,visible_agent,collisions):
        #visible_agent.respond_to_negotiation()
        option = standard_splitting(collisions[0])
        
        #We are doing Vickrey bidding here: a monetary value is given to the other for being allowed to stay on course
        proposed_bid = self.proposal(visible_agent,option[0])
        opponents_bid = visible_agent.proposal(self,option[1])
        
        print("proposed",proposed_bid)
        print(opponents_bid)
        #Find whether the bid gets accepted or not
        accept = False
        if proposed_bid > opponents_bid:
            accept = True
        if abs(proposed_bid - opponents_bid) < 0.001: #within bounds to prevent floating point errors
            flip = random.randint(0,1) #if the bids are the same, a cointoss will decide who wins
            if flip == 0:
                accept = True
            else:
                accept = False

        if accept:
            #Visible agent will use a new path
            self.constraints.pop(-1)
            visible_agent.update_path(visible_agent.calculate_new_path())
            self.money -= proposed_bid
            visible_agent.money += proposed_bid
            #print(f"Agent {visible_agent.id} accepts my (agent: {AircraftDistributed.id}) proposal and will change course ")
        else:
            #current agent will use a new path
            visible_agent.constraints.pop(-1)
            self.update_path(self.calculate_new_path())
            visible_agent.money -= proposed_bid
            self.money += proposed_bid
            #print(f"Agent {visible_agent.id} accepts my (agent: {AircraftDistributed.id}) proposal and will change course ")
            
        
        pass #TODO