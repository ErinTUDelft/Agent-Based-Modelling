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
    big_M = 10000
    constraint_consideration_limit = 10**7

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
        self.constraints_under_consideration = []
        self.path_under_consideration = []
        self.used_paths = []
        self.constraints = []
        self.money = 10 #in thousands, but completely arbitrary ofcourse
        self.utility_factor = 0.8
        self.has_reached_goal = False
        self.path_unchanged = True
        
        self.update_path(self.calculate_new_path())
        
    def update_path(self, new_path):
        self.intended_path = deepcopy(new_path)
        self.used_paths.append(deepcopy(new_path))

        
    def update_location(self, new_location):
        self.location = new_location
        
    def get_current_time(self):
        # Uses the travelled path to determine the current timestep.
        time = len(self.travelled_path) - 1
        
        return time
        
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
        
    def calculate_new_path(self, additional_constraints=[]):
        # Use A* to calculate a new possible path.
        
        timestep = self.get_current_time()
        
        path = a_star(self.my_map, self.location, self.goal, 
                      self.heuristics, self.id, 
                      self.constraints + additional_constraints, 
                      timestep)
        
        if not self.travelled_path == [] and not path == None:
            path.pop(0)
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
                visible_agents.append(agent)
                
        return visible_agents
    
    # Conflict determination
    def share_intended_path(self, number_of_steps, goal_big_M=False):
        # Function which returns the intended path of an agent.
        intended_path = self.intended_path[0:number_of_steps]
        
        # If there is no intention to move, the final location of the travelled
        # path should be returned.
        if intended_path == []:
            intended_path = [self.travelled_path[-1]]
            
        
        print("Intended path", self.id, intended_path)
            
        #if intended_path[-1] == self.goal and \
        #    len(self.path_under_consideration) <= number_of_steps:
        #    #print("BIG MMM", intended_path + [intended_path[-1]] * self.big_M)
        #    intended_path += [intended_path[-1]] * self.big_M
        #    print("Big M applied in share intended path")
            
        #while len(intended_path) < number_of_steps:
        #    intended_path.append(intended_path[-1])
            
        #print("Intended path", intended_path)
            
        return intended_path
    
    def share_path_under_consideration(self, number_of_steps, goal_big_M=False):
        # Function which returns the path under consideration of an agent.
        intended_path = self.path_under_consideration[0:number_of_steps]
        
        # If there is no intention to move, the final location of the travelled
        # path should be returned.
        if intended_path == []:
            intended_path = [self.travelled_path[-1]]
            
        print("Intended path", self.id, intended_path)
        
        #if intended_path[-1] == self.goal and \
        #    len(self.path_under_consideration) <= number_of_steps:
        #    #print("BIG MMM", intended_path + [intended_path[-1]] * self.big_M)
        #    intended_path += [intended_path[-1]] * self.big_M
        #    
        #    print("Big M applied in share path under consideration")
            
        #while len(intended_path) < number_of_steps:
        #    intended_path.append(intended_path[-1])
            
        return intended_path
    
    def check_conflict(self, visible_agent):
        
        # Obtain the paths of both agents. (Up to max. number of steps to plan)
        own_path = [self.location] + self.share_intended_path(self.number_of_steps_to_plan, True)
        other_path = [visible_agent.location] + visible_agent.share_intended_path(self.number_of_steps_to_plan, True)
        
        # Get the current timestep (necessary for proper constraint definition)
        timestep = self.get_current_time()
        
        # Detect collisions.
        collisions = detect_collisions([own_path, other_path], timestep, 
                                       [self.id, visible_agent.id])
        
        return collisions
    
    def check_conflict_under_consideration(self, visible_agent):
        extra_steps_to_check = len(self.path_under_consideration) - len(self.intended_path)
        
        number_of_steps = self.number_of_steps_to_plan + extra_steps_to_check
        
        own_path = [self.location] + self.share_path_under_consideration(number_of_steps, True)
        other_path = [visible_agent.location] + visible_agent.share_intended_path(number_of_steps, True)
        
        # Get the current timestep (necessary for proper constraint definition)
        timestep = self.get_current_time()
        
        # Detect collisions.
        collisions = detect_collisions([own_path, other_path], timestep, 
                                       [self.id, visible_agent.id])
        return collisions
    
    # Negotiation (Erin)
    
    def consider_new_path(self, constraints):
        length_original_path = len(self.intended_path) #finds the length of the original intended path
        
        for constraint in constraints:
            self.constraints_under_consideration.append(constraint)
        path_to_consider = self.calculate_new_path(self.constraints_under_consideration)
        #if self.get_current_time() > 0:
        #    self.path_under_consideration = deepcopy(self.path_under_consideration[1:])
        
        if path_to_consider == None:
            return self.big_M * self.utility_factor
        
        self.path_under_consideration = path_to_consider
        
        length_new_path = len(self.path_under_consideration)
        
        #print("My", self.id, " new path is:", self.path_under_consideration)
        
        added_cost = length_new_path - length_original_path
        
        return added_cost*self.utility_factor
        
        
    def respond_to_negotiation(self):
        

        # What is known?
        # The location of the current agent
        # The location of the visible agent
        # The location of the conflict
        # The timestep of the conflict
        # The location of an alternative route
        # What the extra cost is of the alternative route
        pass #TODO
    
    def negotiate_new_path(self, visible_agent, collisions):
        # visible_agent.respond_to_negotiation()
        
        print("Negotation started by agent", self.id, "with agent", visible_agent.id)
        
        own_collisions = collisions
        opponents_collisions = visible_agent.check_conflict(self)
        
        self.path_under_consideration = self.intended_path
        visible_agent.path_under_consideration = visible_agent.intended_path
        
        iteration = 0
        
        while not own_collisions == []:
            
            #print("Travelled path", self.travelled_path)
            #print("Other's travelled path", visible_agent.travelled_path)
            
            
            #print("Used paths", self.id, self.used_paths)
            #print("Used paths", visible_agent.id, visible_agent.used_paths)
            
            #print("Full path", self.id, self.travelled_path + self.path_under_consideration)
            #print("Full path", visible_agent.id, visible_agent.travelled_path + visible_agent.path_under_consideration)
            
            print("Own collisions:", own_collisions)
        
            own_constraints = []
            
            extra_steps_to_check = len(self.path_under_consideration) - len(self.intended_path)
            
            if approach == "collision-based":
                for collision in own_collisions:
                    if collision['timestep'] - self.get_current_time() > \
                        self.number_of_steps_to_plan + extra_steps_to_check:
                        continue
                    collision_split = standard_splitting(collision)
                    own_constraints.append(collision_split[0])
                
            #print(self.path_under_consideration)
            #print(self.path_under_consideration[0:self.number_of_steps_to_plan])
            #print(visible_agent.intended_path[0:visible_agent.number_of_steps_to_plan][-1])
            
            #if visible_agent.intended_path[0:visible_agent.number_of_steps_to_plan][-1] == visible_agent.goal and \
            #    len(visible_agent.intended_path) <= visible_agent.number_of_steps_to_plan:
            #        
            #    print("I ran woo")
            #    
            #    for i in range(self.big_M):
            #        own_constraints.append({
            #            'agent': self.id,
            #            'loc': [visible_agent.goal],
            #            'timestep': self.get_current_time() + len(visible_agent.intended_path) + i + 2})
            
            if approach == "prioritised-based":
                
                for i, location in enumerate(visible_agent.intended_path[0:self.number_of_steps_to_plan
                                                                         + extra_steps_to_check]):
                    own_constraints.append({
                    'agent': self.id,
                    'loc': [location],
                    'timestep': self.get_current_time() + i + 1})
                    own_constraints.append({
                    'agent': self.id,
                    'loc': [location, get_location(visible_agent.intended_path, i-1)],
                    'timestep': self.get_current_time() + i + 1})
            
            
                
            print("Own constraints", own_constraints)
            
            # We are doing Vickrey bidding here: a monetary value is given to the 
            # other for being allowed to stay on course
            cost_of_own_new_path = self.consider_new_path(own_constraints)
            
            if cost_of_own_new_path == self.big_M * self.utility_factor:
                break
            
            print("New path under consideration", self.id, self.path_under_consideration)
            
            own_collisions = self.check_conflict_under_consideration(visible_agent)
            
            if len(self.constraints_under_consideration) > self.constraint_consideration_limit:
                exit
                
            animations = dict()
            animate = False
            
            if animate:
                animations[iteration] = Animation(self.my_map, [self.start, visible_agent.start], 
                                                  [self.goal, visible_agent.goal],
                                                  [self.travelled_path+self.path_under_consideration, visible_agent.travelled_path+visible_agent.path_under_consideration])
                #animations[iteration].show()
                #timer.sleep(1)
                animations[iteration].show()
                iteration +=1
                start_time = timer.time()
                while timer.time() - start_time < 1:
                    pass
            
        while not opponents_collisions == []:
            
            print("Opponents collisions:", opponents_collisions)
            
            
            #print("Travelled path", self.travelled_path)
            #print("Other's travelled path", visible_agent.travelled_path)
            
            #print("Used paths", self.id, self.used_paths)
            #print("Used paths", visible_agent.id, visible_agent.used_paths)
        
            opponents_constraints = []
            
            
            extra_steps_to_check = len(visible_agent.path_under_consideration) - len(visible_agent.intended_path)
            
            if approach == "collision-based":
            
                for collision in opponents_collisions:
                    if collision['timestep'] - visible_agent.get_current_time() > \
                        visible_agent.number_of_steps_to_plan + extra_steps_to_check:
                        continue
                    collision_split = standard_splitting(collision)
                    opponents_constraints.append(collision_split[0])
                    
            if approach == "prioritised-based":
            
                for i, location in enumerate(self.intended_path[0:self.number_of_steps_to_plan
                                                                + extra_steps_to_check]):
                    opponents_constraints.append({
                    'agent': visible_agent.id,
                    'loc': [location],
                    'timestep': visible_agent.get_current_time() + i + 1})
                    opponents_constraints.append({
                    'agent': visible_agent.id,
                    'loc': [location, get_location(self.intended_path, i-1)],
                    'timestep': visible_agent.get_current_time() + i + 1})
                
            #if self.intended_path[0:self.number_of_steps_to_plan][-1] == self.goal and \
            #    len(self.intended_path) <= self.number_of_steps_to_plan:
            #        
            #    print("I ran woo")
            #    
            #    for i in range(visible_agent.big_M):
            #        own_constraints.append({
            #            'agent': visible_agent.id,
            #            'loc': [self.goal],
            #            'timestep': visible_agent.get_current_time() + len(self.intended_path) + i})
                    
            
            print("Opponents constraints", opponents_constraints)
            # We are doing Vickrey bidding here: a monetary value is given to the 
            # other for being allowed to stay on course
            cost_of_opponents_new_path = visible_agent.consider_new_path(opponents_constraints)
            
            if cost_of_own_new_path == visible_agent.big_M * visible_agent.utility_factor:
                break
            
            print("New path under consideration", visible_agent.id, visible_agent.path_under_consideration)
            
            opponents_collisions = visible_agent.check_conflict_under_consideration(self)
            
            if len(visible_agent.constraints_under_consideration) > self.constraint_consideration_limit:
                exit
        
        print("Timestep", self.get_current_time())
        print("Own location", self.location)
        print("Own possible path", self.id, self.location, self.path_under_consideration)
        print("Opponents intended path", visible_agent.location, visible_agent.id, visible_agent.intended_path)
        print("Opponents possible path", visible_agent.location, visible_agent.id, visible_agent.path_under_consideration)
        print("Own intended path", self.location, self.id, self.intended_path)
        
        own_remaining_money = self.money - cost_of_opponents_new_path
        opponents_remaining_money = visible_agent.money - cost_of_own_new_path
        
        #print("proposed",cost_of_own_new_path)
        #print(cost_of_opponents_new_path)
        # Find whether the bid gets accepted or not
        accept = False
        if cost_of_own_new_path > cost_of_opponents_new_path:
            accept = True
            
        # TODO: Solve edge case of both having negative money.
        if self.money < 0 and self.money < visible_agent.money:
            accept = False
        elif visible_agent.money < 0 and visible_agent.money < self.money:
            accept = True
        if abs(cost_of_own_new_path - cost_of_opponents_new_path) < 0.001: #within bounds to prevent floating point errors
            flip = random.randint(0,1) #if the bids are the same, a cointoss will decide who wins
            print("It's a coin flip.")
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
            print(f"Agent {visible_agent.id} accepts my (agent: {self.id}) proposal and will change course ".format())
        else:
            # Current agent will use a new path
            self.constraints += self.constraints_under_consideration
            self.update_path(self.calculate_new_path())
            self.path_unchanged = False
            visible_agent.money -= cost_of_own_new_path
            self.money += cost_of_own_new_path
            print(f"Agent {self.id} accepts my (agent: {visible_agent.id}) proposal and will change course ")
            
        self.constraints_under_consideration = []
        visible_agent.constraints_under_consideration = []
        
        print("New intended path agent", self.id, self.location, self.intended_path)
        print("New intended path agent", visible_agent.id, visible_agent.location, visible_agent.intended_path)