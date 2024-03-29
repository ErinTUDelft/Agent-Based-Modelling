"""
Main file to run experiments and show animation.

Note: To make the animation work in Spyder you should set graphics backend to 'Automatic' (Preferences > Graphics > Graphics Backend).
"""

import time as timer
#test made
#test2
#!/usr/bin/python
import numpy as np
from random import randint
import matplotlib.pyplot as plt
import statistics
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


SOLVER = "CBS"

min_num_agents = 4
max_num_agents = 8
iterations = 10

sum_costs = True 
CPU = True

CBS = True
Prioritized = True
Prioritized_plus = False
Distributed = False

Random = True
Plot = True
Animations = False
#number_of_collisions
#percentage of failure cpu_time constrainen naar 5 seconden?
#def number_of_colisions


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
    

def random_start(number_of_agents): #only works on a 9x22 grid, but that is all we need to do
    """
    Function for generating a random start. It takes as input the desired amount of agents, and it then generates
    a start and goal location for this agent. It then checks whether the goal location nor start location is already in use. If 
    this is not the case, the valuepair gets added to the respective goal and start lists 
    """
    starts = []
    goals = []
    for agents in range(number_of_agents):
        Condition = False 
        while Condition == False:
            y_start = randint(0,8)
            x_start = randint(0,1)
            valuepair_start = (y_start, x_start)
      
            y_goal = randint(0,8)
            x_goal = randint(20,21)
            valuepair_goal = (y_goal, x_goal)
            
            if valuepair_goal not in goals and valuepair_start not in starts:
                goals.append(valuepair_goal)
                starts.append(valuepair_start)
                Condition = True 
                
            else:
                continue
            
    return starts, goals

def plot(agents,cbs_cpu_list, cbs_cost_list, prioritized_cpu_list, prioritized_cost_list,x_axis):
    """
    This function will plot the important parameters around which the different solvers are compared. 
    It takes as input the agents, the cpu_times for the respective solvers, the costs of the respective solvers

    Parameters
    ----------
    agents : TYPE
        DESCRIPTION.
    cbs_cpu_list : TYPE
        DESCRIPTION.
    cbs_cost_list : TYPE
        DESCRIPTION.
    prioritized_cpu_list : TYPE
        DESCRIPTION.
    prioritized_cost_list : TYPE
        DESCRIPTION.
    x_axis : TYPE
        DESCRIPTION.

    Returns
    -------
    None.

    """
    figure, axis = plt.subplots(2,2)
    
    axis[0,0].plot(x_axis,cbs_cpu_list)
    axis[0,0].plot(x_axis,prioritized_cpu_list)
    axis[0,0].set_title("cpu times")
    axis[0,0].legend(['cbs', 'prioritized'])
    
    axis[0,1].plot(x_axis,cbs_cost_list)
    axis[0,1].plot(x_axis,prioritized_cost_list)
    axis[0,1].set_title("costs")
    axis[0,1].legend(['cbs', 'prioritized'])
    
    axis[1,0].plot(x_axis,cbs_cpu_list)
    axis[1,0].plot(x_axis,prioritized_cpu_list)
    axis[1,0].set_title("change of failure")
    
    axis[1,1].plot(x_axis,cbs_cost_list)
    axis[1,1].plot(x_axis,prioritized_cost_list)
    axis[1,1].set_title("undefined")
    axis[1,1].set_xlabel('Amount of agents [-]')
    axis[1,0].set_xlabel('Amount of agents [-]')
    axis[1,0].set_ylabel('CPU time [s]')
    plt.show()


def import_mapf_instance(filename):
    random = True
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
    
    if not line:
        random = True
    
    #XX work in progress
    starts = []
    goals = []
    

    if random == True:
        #starts, goals = random_start(number_of_agents)
        pass
        
    else:
        for a in range(num_agents):
            line = f.readline()
            sx, sy, gx, gy = [int(x) for x in line.split(' ')]
            starts.append((sx, sy))
            goals.append((gx, gy))
        f.close()
    return my_map, starts, goals


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

    for file in sorted(glob.glob(args.instance)):

        print("***Import an instance***")
        my_map, starts, goals = import_mapf_instance(file)
        
        print_mapf_instance(my_map, starts, goals)
        
        cbs_cpu_list = []
        cbs_cost_list = []
        
        prioritized_cpu_list = []
        prioritized_cost_list = []
        
        x_axis = []
        
        for agents in range(min_num_agents,max_num_agents):
            #for iteration in range(iterations):
            x_axis.append(agents) 
            
            for i in range(2):
                cbs_cpu = []
                cbs_cost = []
                prioritized_cost = []
                prioritized_cpu = []
            
                if Random == False:
                    my_map, starts, goals = import_mapf_instance(file)
                    
                if Random == True: #could have else, but wanted this to be explicit
                    my_map , starts, goals = import_mapf_instance(file)
                    starts, goals = random_start(agents)
                    
                if CBS == True:
                    print("***Run CBS***")
                    start_time = timer.time()
                    cbs = CBSSolver(my_map, starts, goals)
                    paths = cbs.find_solution(args.disjoint)
                    cbs_cpu = timer.time() - start_time
                    
                    if paths is False:
                        failure_cbs = 1
                    if paths is not False:
                        cbs_cost.append(get_sum_of_cost(paths))
                    

                    print("cbs cpu", cbs_cpu)
                    if Animations == True: 
                        print("***Test paths on a simulation***")
                        animation = Animation(my_map, starts, goals, paths)
                        # animation.save("output.mp4", 1.0) # install ffmpeg package to use this option
                        animation.show()
                        timer.sleep(2)
                    
                # if Prioritized_plus == True:
                #     print("***Run Prioritized+ ***")
                #     solver = IndependentSolver(my_map, starts, goals)
                #     paths = solver.find_solution()
                if Prioritized == True:
                    print("***Run Prioritized***")
                    start_time = timer.time()
                    solver = PrioritizedPlanningSolver(my_map, starts, goals)
                    paths = solver.find_solution()
                    prioritized_cpu = timer.time() - start_time
                    print("pathss", paths)
                    if paths is not False:
                        prioritized_cost.append(get_sum_of_cost(paths))
    

                    print("prio cpu:", prioritized_cpu)
                    if Animations == True:
                        print("***Test paths on a simulation***")
                        animation = Animation(my_map, starts, goals, paths)
                        # animation.save("output.mp4", 1.0) # install ffmpeg package to use this option
                        animation.show()
                        timer.sleep(2)
                    
                if Distributed == True:  # Wrapper of distributed planning solver class
                    print("***Run Distributed Planning***")
                    solver = DistributedPlanningSolver(my_map, starts, goals, ...) #!!!TODO: add your own distributed planning implementation here.
                    paths = solver.find_solution()
                    
            prioritized_cost = np.mean(prioritized_cost)
            prioritized_cpu = np.mean(prioritized_cpu)
            cbs_cpu = np.mean(cbs_cpu)
            cbs_cost = np.mean(cbs_cost)
            cbs_cpu_list.append(cbs_cpu)
            cbs_cost_list.append(cbs_cost)             
            prioritized_cost_list.append(prioritized_cost)
            prioritized_cpu_list.append(prioritized_cpu)                    
                
            # else: 
            #     raise RuntimeError("Unknown solver!")
                
            # if Animations == True:
            #     print("***Test paths on a simulation***")
            #     animation = Animation(my_map, starts, goals, paths)
            #     # animation.save("output.mp4", 1.0) # install ffmpeg package to use this option
            #     animation.show()
            #     timer.sleep(2)
    plot(agents,cbs_cpu_list, cbs_cost_list, prioritized_cpu_list, prioritized_cost_list,x_axis)    
    result_file.close()
    