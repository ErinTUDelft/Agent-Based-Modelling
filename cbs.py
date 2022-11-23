import time as timer
import heapq
import random
from single_agent_planner import compute_heuristics, a_star, get_location, get_sum_of_cost
from visualize import Animation
from copy import deepcopy
#from treelib import Node, Tree


def detect_collision(path1, path2, debug=False):
    """ Returns the first collision that occurs between two robot paths (or None if there is no collision)

    path1    - First path to evaluate
    path2    - Second path to evaluate
    debug    - Debug parameter (Boolean) to generate additional print statements.
    """

    path = max([len(i) for i in [path1, path2]])
 
    for timestep in range(path):
        
        # Print statement for debug purposes
        if debug:
            print(get_location(path1, timestep), get_location(path2, timestep), 
                  get_location(path1, timestep) == get_location(path2, timestep),
                  get_location(path1, timestep) == get_location(path2, timestep + 1) and \
                      get_location(path2, timestep) == get_location(path1, timestep + 1))
        
        # Vertex collision
        if get_location(path1, timestep) == get_location(path2, timestep):
            return {'loc': [get_location(path1, timestep)], 'timestep': timestep}
        
        # Edge collision
        if get_location(path1, timestep) == get_location(path2, timestep + 1) and \
            get_location(path2, timestep) == get_location(path1, timestep + 1):
            return {'loc': [get_location(path1, timestep), get_location(path2, timestep)], 'timestep': timestep + 1}

    return None


def detect_collisions(paths, start_time=0, agents=[], debug=False):
    """ Returns a list of first collisions between all robot pairs.
    
    paths       - All robot paths
    start_time  - Start time of the first value of the paths variables
    agents      - List of agent IDs
    debug       - Debug parameter (Boolean) to generate additional print statements
    """
    ##############################
    # Task 3.1: Return a list of first collisions between all robot pairs.
    #           A collision can be represented as dictionary that contains the id of the two robots, the vertex or edge
    #           causing the collision, and the timestep at which the collision occurred.
    #           You should use your detect_collision function to find a collision between two robots.
    
    collisions = []
    
    # Print statement for debug purposes.
    if debug:
        print(paths)

    # We evaluate for each agent.
    for agent, path in enumerate(paths):
        
        # If agent ids are specified, we set the agent id.
        if not agents == []:
            agent_id = agents[agent]
        else:
            agent_id = agent
            
        # Print statement for debug purposes.
        if debug:
            print(agent_id, path)
        
        # We only need to evaluate the relation of an agent with other agents
        # with which it has not previously interacted. Hence, we start the range
        # at agent + 1 (it also should not evaluate interaction with itself.).
        for other_agent in range(agent + 1, len(paths)):
            
            # If agent ids are specified, we set the agent id.
            if not agents == []:
                other_agent_id = agents[other_agent]
            else:
                other_agent_id = other_agent
            
            # Print statement for debug purposes.
            if debug:
                print("Comparing agent", agent_id, "and", other_agent_id)
            
            # We detect if there is a collision.
            collision_detection = detect_collision(path, paths[other_agent], debug)
            
            # If this is the case, we add the collision to the collisions list.
            if not collision_detection == None:
                
                collision = {'a1': agent_id,
                             'a2': other_agent_id,
                             'loc': deepcopy(collision_detection['loc']),
                             'timestep': collision_detection['timestep'] + start_time}                
                collisions.append(collision)
    
    # Print statement for debug purposes.
    if debug:
        print(sorted(collisions, key=lambda collisions: collisions['timestep']))

    return sorted(collisions, key=lambda collisions: collisions['timestep'])


def standard_splitting(collision):
    """ Returns a list of (two) constraints to resolve the given collision.
    
    collision   - Collision to split.
    """
    
    constraints = []
    
    # Vertex constraints
    if len(collision['loc']) == 1:
        constraints.append({'agent': collision['a1'], 
                            'loc': deepcopy(collision['loc']), 
                            'timestep': collision['timestep']})
        constraints.append({'agent': collision['a2'], 
                            'loc': deepcopy(collision['loc']), 
                            'timestep': collision['timestep']})
        
    # Edge constraints
    elif len(collision['loc']) == 2:
        constraints.append({'agent': collision['a1'], 
                            'loc': [collision['loc'][0], collision['loc'][1]], 
                            'timestep': collision['timestep']})
        # For the second constraint, we must flip the edge!
        constraints.append({'agent': collision['a2'], 
                            'loc': [collision['loc'][1], collision['loc'][0]], 
                            'timestep': collision['timestep']})

    return constraints

class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, my_map, starts, goals, CPU_cutoff_time):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        CPU_cutoff_time - Cutoff time to prevent excessive long runs
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0
        self.CPU_cutoff_time = CPU_cutoff_time

        self.open_list = []

        # Compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def push_node(self, node):
        """ Inserts node into the open list in an ordered fashion using heappush. """
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        #print("Generate node {}".format(self.num_of_generated), "Cost:", node['cost'])
        self.num_of_generated += 1

    def pop_node(self):
        """ Extracts best node from the open list using heappop. """
        _, _, id, node = heapq.heappop(self.open_list)
        #print("Expand node {}".format(id), "Cost:", node['cost'])
        self.num_of_expanded += 1
        return node

    def find_solution(self, disjoint=True):
        """ Finds paths for all agents from their start locations to their goal locations

        disjoint    - use disjoint splitting or not
        """

        start_time = timer.time()
        
        # File to use for debugging purposes
        debug_file = open("debug.txt", "w", buffering=1)

        # Generate the root node
        # cost          - cost of solution presented in node
        # constraints   - list of constraints
        # paths         - list of paths, one for each agent
        #               [[(x11, y11), (x12, y12), ...], [(x21, y21), (x22, y22), ...], ...]
        # collisions    - list of collisions in paths
        # iteration     - iteration of node
        root = {'cost': 0,
                'constraints': [],
                'paths': [],
                'collisions': [],
                'iteration': 0}
        
        for i in range(self.num_of_agents):  # Find initial path for each agent
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, root['constraints'])
            if path is None:
                raise BaseException('No solutions')
            root['paths'].append(path)

        # Fill out the root node.
        root['cost'] = get_sum_of_cost(root['paths'])
        root['collisions'] = detect_collisions(root['paths'])
        self.push_node(root) # Add root node to open list.
        
        # Optional code to animate multiple iterations.
        animations = dict()
        # animation = Animation(self.my_map, self.starts, self.goals, root['paths'])
        # animation.save("output.mp4", 1.0) # install ffmpeg package to use this option
        # timer.sleep(0.5)
        # animation.show()
        # print("Root node")
        # print("Paths:", root['paths'])
        # print("Constraints", root['constraints'])
        # print("Collisions:", root['collisions'])
        
        iteration = 0

        while len(self.open_list) > 0:
            node = self.pop_node() # Obtain the current best node.
            
            # Check if the node is collision-free, and thus optimal.
            if len(node['collisions']) == 0:
                print(node)
                return node['paths']
            
            # Grab the first collision
            collision = node['collisions'][0]
            constraints = standard_splitting(collision)
            
            # For each constraint, attempt to generate a new child node.
            for constraint in constraints:
                
                # Cutoff point for statistical analysis
                if timer.time() - start_time > self.CPU_cutoff_time:
                    return None
                
                iteration += 1
                
                # Optional code for debugging purposes
                if False: # iteration >= 997:
                    debug = True
                else:
                    debug = False
                
                # Create new child node.
                new_node = dict()
                new_node['iteration'] = iteration
                
                # Optional code for debugging purposes.
                if debug:
                    pass
                    animate = False
                    visualised_branch = [759, 552, 549, 515, 484, 282, 270, 260, 215, 111, 83, 51, 15, 7, 4, 1, 0]
                    if iteration in visualised_branch:
                        print("VISUALISE")
                        animate = True
                
                # Fill out new child node with paths and constraints from its parent.
                new_node['constraints'] = deepcopy(node['constraints']) + [constraint]
                new_node['paths'] = deepcopy(node['paths'])
                
                # Find the new path for the agent, given the new constraints.
                agent = constraint['agent']
                path = a_star(self.my_map, self.starts[agent], self.goals[agent], 
                              self.heuristics[agent], agent, new_node['constraints'])
                
                # If there is no solution, this is not a feasible child node.
                if path == None:
                    continue
                
                # Fill out the new node with the new path.
                new_node['paths'][agent] = deepcopy(path)
                new_node['collisions'] = detect_collisions(new_node['paths'])
                new_node['cost'] = get_sum_of_cost(new_node['paths'])
                
                # Optional code to visualise an entire branch of the algorithm for debugging purposes.
                if debug:
                    pass
                    if animate:
                        animations[iteration] = Animation(self.my_map, self.starts, self.goals, new_node['paths'])
                        animations[iteration].show()
                        timer.sleep(1)
                    print("Node", iteration)
                    print("Paths:", new_node['paths'])
                    print("Constraints", new_node['constraints'])
                    print("Collisions:", new_node['collisions'])
                    
                    debug_file.write("Node {}\n".format(iteration))
                    debug_file.write("Parent node: {}\n".format(node['iteration']))
                    debug_file.write("Paths: {}\n".format(new_node['paths']))
                    debug_file.write("Constraints: {}\n".format(new_node['constraints']))
                    debug_file.write("Collisions: {}\n".format(new_node['collisions']))
                    
                    # tree.create_node(iteration,iteration, parent=node['iteration'])
                
                # Add the node to the open list.
                self.push_node(new_node)
                
        print("No solution found.")
        result = self.pop_node()
        print("Visualised node:", result)
        # tree.show()
        self.print_results(result)
        debug_file.close()
        return result['paths']


    def print_results(self, node):
        print("\n Found a solution! \n")
        CPU_time = timer.time() - self.start_time
        print("CPU time (s):    {:.2f}".format(CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(node['paths'])))
        print("Expanded nodes:  {}".format(self.num_of_expanded))
        print("Generated nodes: {}".format(self.num_of_generated))
