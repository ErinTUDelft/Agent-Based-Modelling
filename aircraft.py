"""
This file contains the AircraftDistributed class that can be used to implement individual planning.

Code in this file is just provided as guidance, you are free to deviate from it.
"""

from single_agent_planner import a_star
from cbs import detect_collisions

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
        
    def update_path(self, new_path, new_location):
        self.used_paths.append(new_path)
        self.path.append(new_location)
        
    def update_location(self, new_location):
        self.location = new_location
        
    def calculate_new_path(self):
        path = a_star(self.my_map, self.start, self.goal, 
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
    def respond_to_negotiation(self):
        pass #TODO
    
    def negotiate_new_path(self):
        pass #TODO