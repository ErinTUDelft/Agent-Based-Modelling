"""
This file contains the AircraftDistributed class that can be used to implement individual planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

from single_agent_planner import a_star
from cbs import detect_collisions
import random
visible_area_range = 3

class AircraftDistributed(object):
    """Aircraft object to be used in the distributed planner."""

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
        self.path = []
        self.used_paths = []
        self.constraints = []
        self.money = 10 #in thousands, but completely arbitrary ofcourse
        
    def update_path(self, new_path, new_location):
        self.used_paths.append(new_path)
        self.path.append(new_location)
        

        
    def update_location(self, new_location):
        self.location = new_location
        
    def calculate_new_path(self):
        path = a_star(self.my_map, self.location, self.goal, 
                      self.heuristics, self.id, self.constraints)
        return path
    
    def run_agent_radar(self, agents):
        # Determining the visible area. Currently the area is assumed square.
        visible_area = (range(self.location[0] - visible_area_range,
                              self.location[0] + visible_area_range), 
                        range(self.location[1] - visible_area_range,
                              self.location[1] + visible_area_range))
        
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
    def share_path(self):
        pass #TODO
    
    def check_conflict(self):
        
        #share_path()
        pass #TODO
    
    
    # Negotiation (Erin)
    
    def proposal(self):
        length_original_path = len(self.path) #finds the length of the original intended path
        
        conflict_location = self.check_conflict(self) #Adds the conflict of interest to the constraint table
        self.constraints.append(conflict_location)
        length_new_path = self.calculate_new_path(self)
        
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
    
    def negotiate_new_path(self,visible_agent):
        visible_agent.respond_to_negotiation()
        
        #We are doing Vickrey bidding here: a monetary value is given to the other for being allowed to stay on course
        proposed_bid = self.proposal(self)
        opponents_bid = self.proposal(visible_agent)
        
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

        if accept == True:
            #Visible agent will use a new path
            self.money -= proposed_bid
            visible_agent.money += proposed_bid
            print(f"Agent {visible_agent.id} accepts my (agent: {AircraftDistributed.id}) proposal and will change course ")
        else:
            #current agent will use a new path
            self.constraints.append(conflict_location,timestep)
            self.path = self.calculate_new_path(self)
            self.money += proposed_bid
            visible_agent.money -= proposed_bid
            
            print(f"I, agent {AircraftDistributed.id} accepts agent's {visible_agent.id} proposal and will change my course ")
            
        
        pass #TODO