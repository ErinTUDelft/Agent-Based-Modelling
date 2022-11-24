"""
Main file to run experiments and show animation.

Note: To make the animation work in Spyder you should set graphics backend to 'Automatic' (Preferences > Graphics > Graphics Backend).
"""

#!/usr/bin/python
from random import randint
from random import shuffle
from math import inf
import matplotlib.pyplot as plt
import argparse
import glob
from pathlib import Path
from cbs import CBSSolver
from independent import IndependentSolver
from prioritized import PrioritizedPlanningSolver
from distributed import DistributedPlanningSolver # Placeholder for Distributed Planning
from visualize import Animation
from single_agent_planner import get_sum_of_cost
import pandas as pd
import time as timer
SOLVER = "Prioritized"



# Determine whether to perform statistical analysis or to perform a single simulation.
statistical_analysis = True

# Should values from the assignment.txt file be used or should start/goal locations
# be completely random? Note that statistical analysis is always fully random.
full_random = True

# CPU cut-off time
CPU_cutoff_time = 5

# Algorithms to consider for statistical analysis
simulations_per_agent = 50
agent_range = range(2,11)

Prioritized = True
PrioritizedPlusSimple = True
PrioritizedPlus = True
CBS =  True 
Distributed = True 

algorithms = ["Prioritized", "PrioritizedPlus", "PrioritizedPlusSimple", "CBS", "Distributed"]
algorithms_used = [Prioritized, PrioritizedPlus, PrioritizedPlusSimple, CBS, Distributed]
algorithm_colors = {"Prioritized": 'blue', 
                    "PrioritizedPlus": 'yellow', 
                    "PrioritizedPlusSimple": 'green', 
                    "CBS": 'orange', 
                    "Distributed": 'red'}

def print_mapf_instance(my_map, starts, goals):
    """
    Prints start location and goal location of all agents, using @ for an obstacle, . for an open cell, and 
    a number for the start location of each agent.
    
    Example:
        @ @ @ @ @ @ @ 
        @ 0 1 . . . @ 
        @ @ @ . @ @ @ 
        @ @ @ @ @ @ @ 
    """
    print('Start locations')
    print_locations(my_map, starts)
    print('Goal locations')
    print_locations(my_map, goals)


def print_locations(my_map, locations):
    """
    See docstring print_mapf_instance function above.
    """
    starts_map = [[-1 for _ in range(len(my_map[0]))] for _ in range(len(my_map))]
    for i in range(len(locations)):
        starts_map[locations[i][0]][locations[i][1]] = i
    to_print = ''
    for x in range(len(my_map)):
        for y in range(len(my_map[0])):
            if starts_map[x][y] >= 0:
                to_print += str(starts_map[x][y]) + ' '
            elif my_map[x][y]:
                to_print += '@ '
            else:
                to_print += '. '
        to_print += '\n'
    print(to_print)
    

def random_start(number_of_agents, start_locs=None, goal_locs=None):
    """
    This function spawns the agent randomly on the first two columns, and assigns their location on the opposite 
    side of the map.
    """


    if start_locs == None:
        start_locs = []
    if goal_locs == None:
        goal_locs = [] 
    for agent in range(number_of_agents):
        while True:
            y_start = randint(0,8)
            x_start = randint(0,1)
            valuepair_start = (y_start, x_start)
      
            y_goal = 8 - y_start
            x_goal = 21 - x_start
            valuepair_goal = (y_goal, x_goal)

            
            if valuepair_goal not in goal_locs and valuepair_start not in start_locs:
                goal_locs.append(valuepair_goal)
                start_locs.append(valuepair_start)
                break 
        # print(start_locs, goal_locs)
                
    return start_locs, goal_locs

def import_mapf_instance(filename):
    """
    Imports mapf instance from instances folder. Expects input as a .txt file in the following format:
        Line1: #rows #columns (number of rows and columns)
        Line2-X: Grid of @ and . symbols with format #rows * #columns. The @ indicates an obstacle, whereas . indicates free cell.
        Line X: #agents (number of agents)
        Line X+1: xCoordStart yCoordStart xCoordGoal yCoordGoal (xy coordinate start and goal for Agent 1)
        Line X+2: xCoordStart yCoordStart xCoordGoal yCoordGoal (xy coordinate start and goal for Agent 2)
        Line X+n: xCoordStart yCoordStart xCoordGoal yCoordGoal (xy coordinate start and goal for Agent n)
        
    Example:
        4 7             # grid with 4 rows and 7 columns
        @ @ @ @ @ @ @   # example row with obstacle in every column
        @ . . . . . @   # example row with 5 free cells in the middle
        @ @ @ . @ @ @
        @ @ @ @ @ @ @
        2               # 2 agents in this experiment
        1 1 1 5         # agent 1 starts at (1,1) and has (1,5) as goal
        1 2 1 4         # agent 2 starts at (1,2) and has (1,4) as goal
    """
    f = Path(filename)
    if not f.is_file():
        raise BaseException(filename + " does not exist.")
    f = open(filename, 'r')
    # first line: #rows #columns
    line = f.readline()
    rows, columns = [int(x) for x in line.split(' ')]
    rows = int(rows)
    columns = int(columns)
    # #rows lines with the map
    my_map = []
    for r in range(rows):
        line = f.readline()
        my_map.append([])
        for cell in line:
            if cell == '@':
                my_map[-1].append(True)
            elif cell == '.':
                my_map[-1].append(False)
    # #agents
    line = f.readline()
    num_agents = int(line)

    starts = []
    goals = []
    

    for a in range(num_agents):
        line = f.readline()
        
        # Any agents which do not have a start and goal location are given
        # these randomly.
        if not line:
            starts, goals = random_start(num_agents-a, starts, goals)
            break
        
        sx, sy, gx, gy = [int(x) for x in line.split(' ')]
        starts.append((sx, sy))
        goals.append((gx, gy))
    f.close()
    
    return my_map, starts, goals

def cost_plot(dfs):
    """"
    This function takes the large created dataframe, and categorizes it by grouping the 
    minimum, maximum and mean value for each number of agents. These values are then plotted. 
    """
    plt.figure()
    #plt.title('Total costs per number of agents ')
    plt.xlabel('Agents [-]')
    plt.ylabel('Average cost [-]')
    
    current_cost_max = 0
    current_cost_min = inf
    for df_key in dfs.keys():
        color = algorithm_colors[df_key]
        dfs[df_key].groupby('agents')['costs'].max().plot(color = color, label="", alpha=0.35)
        dfs[df_key].groupby('agents')['costs'].min().plot(color = color, label="", alpha=0.35)
        dfs[df_key].groupby('agents')['costs'].mean().plot(color = color, label=df_key, ylim = 0)
        #plt.legend(['maximum', 'minimum' , 'mean'])
        
        costs_max = dfs[df_key].groupby('agents')['costs'].max()
        costs_min = dfs[df_key].groupby('agents')['costs'].min()
        
        cost_max = costs_max.rename_axis(index=None).max()
        cost_min = costs_min.rename_axis(index=None).min()
        
        current_cost_max = max(cost_max, current_cost_max)
        current_cost_min = min(cost_min, current_cost_min)
        
        plt.fill_between(agent_range,costs_max, costs_min, color=color, alpha=0.25)
        

   
 
    plt.legend()
    plt.ylim(0, 1.05*current_cost_max)
   
    #plt.fill_between(costs_max,costs_min)
    #successrate = df.groupby('agents')['success'].mean()
    #finishtimes = df.groupby('agents')['finish times'].mean()

    plt.show()

def succes_plot(dfs):
    """"
    This function takes the large created dataframe, and categorizes it by grouping the 
    success rates for the different algorithms. As the successes are stored as either True or False Boolean's
    (Which are represented by 1's and 0's, the average can be taken. These values are then plotted. 
    """
    plt.figure()
    #plt.title('Success rate per number of agents')
    plt.xlabel('Agents [-]')
    plt.ylabel('Success rate [-]')
    #plt.ylim(0, 1.2)
    for df_key in dfs.keys():
        color = algorithm_colors[df_key]
        df = dfs[df_key]
        success_rate = df.groupby('agents')['success'].mean()
        success_rate.plot(label=df_key, color=color, ylim = [0,1.2])
        
    plt.legend()
    plt.show()
    

def finish_times_plot(dfs):
    """
    The finish times are defined as the time it takes for all the agents to reach their goal location. 
    This value is found by taking the maximum path length of each solution. 
    """


    plt.figure()
    #plt.title('Finish times per number of agents')
    plt.xlabel('Agents [-]')
    plt.ylabel('Finish time [-]')
    
    current_finish_time_max = 0
    current_finish_time_min = inf
    
    for df_key in dfs.keys():
    
        color = algorithm_colors[df_key]
        
        dfs[df_key].groupby('agents')['finish times'].max().plot(color=color, label="", alpha=0.35)
        dfs[df_key].groupby('agents')['finish times'].min().plot(color=color, label="", alpha=0.35)
        dfs[df_key].groupby('agents')['finish times'].mean().plot(color=color, label=df_key, ylim = 0)
        
        finish_times_max = dfs[df_key].groupby('agents')['finish times'].max()
        finish_times_min = dfs[df_key].groupby('agents')['finish times'].min()
        
        finish_time_max = finish_times_max.rename_axis(index=None).max()
        finish_time_min = finish_times_min.rename_axis(index=None).min()
        
        current_finish_time_max = max(finish_time_max, current_finish_time_max)
        current_finish_time_min = min(finish_time_min, current_finish_time_min)
    
        plt.fill_between(agent_range,finish_times_max, finish_times_min, color=color, alpha=0.25)
    plt.legend()
    plt.ylim(0, 1.05*current_finish_time_max)
    plt.show()
    
def cpu_times_plot(dfs):
    plt.figure() 

    plt.xlabel('Agents [-]')
    plt.ylabel('CPU time [s]')
    for df_key in dfs.keys():
        color = algorithm_colors[df_key]
        dfs[df_key].groupby('agents')['cpu_times'].mean().plot(color=color, label=df_key)
    plt.legend()
    plt.show()
    
def add_to_database(algorithm, paths, cpu_time):
    #cpu_time = timer.time() - start_time
    
    if not paths == None: 
        costs[algorithm].append(get_sum_of_cost(paths)/agents)
        finish_times[algorithm].append(len(max(paths)))
        success[algorithm].append(True)
        cpu_times[algorithm].append(cpu_time)
        
    if paths == None:
        costs[algorithm].append(None)
        finish_times[algorithm].append(None)
        success[algorithm].append(False)
        cpu_times[algorithm].append(None)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs various MAPF algorithms')
    parser.add_argument('--instance', type=str, default=None,
                        help='The name of the instance file(s)')
    parser.add_argument('--batch', action='store_true', default=False,
                        help='Use batch output instead of animation')
    parser.add_argument('--disjoint', action='store_true', default=False,
                        help='Use the disjoint splitting')
    parser.add_argument('--solver', type=str, default=SOLVER,
                        help='The solver to use (one of: {CBS,Independent,Prioritized}), defaults to ' + str(SOLVER))

    args = parser.parse_args()
    # Hint: Command line options can be added in Spyder by pressing CTRL + F6 > Command line options. 
    # In PyCharm, they can be added as parameters in the configuration.

    result_file = open("results.csv", "w", buffering=1)
    
    if not statistical_analysis:
        for file in sorted(glob.glob(args.instance)):

            print("***Import an instance***")
            my_map, starts, goals = import_mapf_instance(file)
            if full_random:
                starts, goals = random_start(len(starts))
            print_mapf_instance(my_map, starts, goals)
    
            if args.solver == "CBS":
                print("***Run CBS***")
                cbs = CBSSolver(my_map, starts, goals, CPU_cutoff_time)
                paths = cbs.find_solution(args.disjoint)
            elif args.solver == "Independent":
                print("***Run Independent***")
                solver = IndependentSolver(my_map, starts, goals)
                paths = solver.find_solution()
            elif args.solver == "Prioritized":
                print("***Run Prioritized***")
                solver = PrioritizedPlanningSolver(my_map, starts, goals, CPU_cutoff_time)
                paths = solver.find_solution()
            elif args.solver == "Distributed":  # Wrapper of distributed planning solver class
                print("***Run Distributed Planning***")
                solver = DistributedPlanningSolver(my_map, starts, goals, CPU_cutoff_time)
                paths = solver.find_solution()
            else: 
                raise RuntimeError("Unknown solver!")
    
            cost = get_sum_of_cost(paths)
            result_file.write("{},{}\n".format(file, cost))
    
    
            if not args.batch:
                print("***Test paths on a simulation***")
                animation = Animation(my_map, starts, goals, paths)
                # animation.save("output.mp4", 1.0) # install ffmpeg package to use this option
                animation.show()
                
            

    if statistical_analysis:

        for file in sorted(glob.glob(args.instance)):
    
            print("***Import an instance***")
            
            my_map, starts, goals = import_mapf_instance(file)
            print_mapf_instance(my_map, starts, goals)
            
            costs = dict()
            finish_times = dict()
            success = dict()
            cpu_times = dict()
            
            for i, algorithm in enumerate(algorithms):
                if not algorithms_used[i]:
                    continue
                
                #print(algorithm)
                
                costs[algorithm] = []
                finish_times[algorithm] = []
                success[algorithm] = []
                cpu_times[algorithm] = []
                

            nums_of_agents = []
            
            for agents in agent_range:

         
                for simulations in range(simulations_per_agent):
                    
                    print("Iteration", simulations, "for", agents, "agents. Total progress:", 
                          round(((agents-min(agent_range))*simulations_per_agent+simulations)/
                          (len(agent_range)*simulations_per_agent)*100,1), "%")
                    
                    nums_of_agents.append(agents)
                   
                    my_map, starts, goals = import_mapf_instance(file)
                    starts, goals = random_start(agents)
                    
                    start_time = timer.time()
                    
            
                    if CBS:
                        start_time = timer.time()
                        #print("***Run CBS***")
                        cbs = CBSSolver(my_map, starts, goals, CPU_cutoff_time)
                        paths = cbs.find_solution(args.disjoint)
                        cpu_time = timer.time() - start_time
                        add_to_database("CBS", paths, cpu_time)
                        

                            
                    if Prioritized:
                        start_time = timer.time()
                        solver = PrioritizedPlanningSolver(my_map, starts, goals, CPU_cutoff_time)
                        paths = solver.find_solution()
                        
                        cpu_time = timer.time() - start_time
                        add_to_database("Prioritized", paths,cpu_time)
                        
                        
                        #print(cpu_times['Prioritized'])
                        
                            
                    if PrioritizedPlusSimple:
                        start_time = timer.time()
                        paths = None
                        while paths == None:
            
                            solver = PrioritizedPlanningSolver(my_map, starts, goals, CPU_cutoff_time)
                            paths = solver.find_solution()
                            
                            # Reorder start-goal pairs.
                            temp = list(zip(starts, goals))
                            shuffle(temp)
                            res1, res2 = zip(*temp)
                            starts, goals = list(res1), list(res2)
                            
                        cpu_time = timer.time() - start_time
                        add_to_database("PrioritizedPlusSimple", paths, cpu_time)        
                            
    
 
                    if PrioritizedPlus:
                        start_time = timer.time()
      
                        solver = DistributedPlanningSolver(my_map, starts, goals, CPU_cutoff_time)
                        paths1 = solver.find_solution()
                     
        
                        paths2 = None
                        
                        if paths2 == None:
                            while paths2 == None:
                               
    
                                solver = PrioritizedPlanningSolver(my_map, starts, goals, CPU_cutoff_time)
                                paths2 = solver.find_solution()
                                
                                # Shuffle two lists with same order
                                # Using zip() + * operator + shuffle()
                                
                                temp = list(zip(starts, goals))
                                shuffle(temp)
                                res1, res2 = zip(*temp)
                                starts, goals = list(res1), list(res2)

                                
                        if paths1 == None and not paths2 == None:
                            paths = paths2
                        elif not paths1 == None and not paths2 == None:       
                            #paths = min(len(paths1), paths2) this was a faulty approach!
                            
                            if get_sum_of_cost(paths1) > get_sum_of_cost(paths2):
                                paths = paths2
                            else:
                                paths = paths1
                        #elif not paths1 == None and paths2 == None:
                            #paths = paths1
                        else: 
                            paths = None
                            
                        # if get_sum_of_cost(paths) > 400:
                        #     print(paths1)
                        #     print("paths2", paths2)
                        # Some simulations would be very inefficient, this condition visualizes those edge cases
                        # if not paths == None: 
                        #     if get_sum_of_cost(paths) > 400: 
                        #         animation = Animation(my_map, starts, goals, paths)
                        #         animation.show()
                                
                        cpu_time = timer.time() - start_time
                        add_to_database("PrioritizedPlus", paths, cpu_time)        
                        

    
                    if Distributed:
                        start_time = timer.time() 
                        #print("***Run Distributed Planning***")
                        solver = DistributedPlanningSolver(my_map, starts, goals, CPU_cutoff_time) 
                        paths = solver.find_solution()
                        cpu_time = timer.time() - start_time
                        add_to_database("Distributed", paths, cpu_time)
                
                        
   
                        
            #print(costs, finish_times, success, cpu_times, nums_of_agents)
            
            dfs = dict()
            
            for i, algorithm in enumerate(algorithms):
                if not algorithms_used[i]:
                    continue
                
                d = {'costs': costs[algorithm], 
                         'finish times': finish_times[algorithm],
                         'success': success[algorithm],
                         'cpu_times': cpu_times[algorithm],
                         'agents': nums_of_agents
                         }
                
                dfs[algorithm] = pd.DataFrame(data = d)
                dfs[algorithm].to_csv("analyses/L3-" + algorithm + ".csv")

            cost_plot(dfs)
            succes_plot(dfs)
            finish_times_plot(dfs)
            cpu_times_plot(dfs)


        animate = False
        if animate and not args.batch:
            print("***Test paths on a simulation***")
            animation = Animation(my_map, starts, goals, paths)
            # animation.save("output.mp4", 1.0) # install ffmpeg package to use this option
            animation.show()
            
    result_file.close()
    
