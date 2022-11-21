"""
Main file to run experiments and show animation.

Note: To make the animation work in Spyder you should set graphics backend to 'Automatic' (Preferences > Graphics > Graphics Backend).
"""


#test made
#test2
#!/usr/bin/python
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
import time as timer
SOLVER = "Prioritized"
from random import randint

CBS = not True 
Prioritized = True
random = True
#agents = 5

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
    

def random_start(number_of_agents, starts, goals):
    for agents in range(number_of_agents):
        while True:
            y_start = randint(0,8)
            x_start = randint(0,1)
            valuepair_start = (y_start, x_start)
      
            y_goal = 8 - y_start
            x_goal = 21 - x_start
            valuepair_goal = (y_goal, x_goal)

            
            if valuepair_goal not in goals and valuepair_start not in starts:
                goals.append(valuepair_goal)
                starts.append(valuepair_start)
                break 
                
    return starts, goals

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
        #starts, goals = random_start(agents)
        print_mapf_instance(my_map, starts, goals)
        
        costs = []
        finishtimes = []
        success = []
        cpu_times = []
        agentsnumber = []
        
        
        
        for agents in range(2,11):
            #print("agents", agents)
            #agents = 5
        
            for simulations in range(50):
                agentsnumber.append(agents)
                
                my_map, starts, goals = import_mapf_instance(file)
                starts, goals = random_start(agents)
                
        
                if CBS == True:
                    #print("***Run CBS***")
                    start_time = timer.time()
                    cbs = CBSSolver(my_map, starts, goals)
                    paths = cbs.find_solution(args.disjoint)
                    cpu_time = timer.time() - start_time
                    
                    if paths is not False: 
                        costs.append(get_sum_of_cost(paths))
                        finishtimes.append(len(max(paths)))
                        success.append(True)
                        cpu_times.append(cpu_time)
                        
                    if paths is False:
                        costs.append(None)
                        finishtimes.append(None)
                        success.append(False)
                        cpu_times.append(None)
                elif args.solver == "Independent":
                    #print("***Run Independent***")
                    solver = IndependentSolver(my_map, starts, goals)
                    paths = solver.find_solution()
                elif Prioritized == True:
                    #print("***Run Prioritized***")
                    # my_map, starts, goals = import_mapf_instance(file)
                    # starts, goals = random_start(agents)

                    start_time = timer.time()
                    solver = PrioritizedPlanningSolver(my_map, starts, goals)
                    paths = solver.find_solution()
                    cpu_time = timer.time() - start_time
                    
                    #print(paths)
                    #cost = get_sum_of_cost(paths)
                    
                    if paths is not False: 
                        costs.append(get_sum_of_cost(paths))
                        finishtimes.append(len(max(paths)))
                        success.append(True)
                        cpu_times.append(cpu_time)
                        
                    if paths is False:
                        costs.append(None)
                        finishtimes.append(None)
                        success.append(False)
                        cpu_times.append(None)
                    

                elif args.solver == "Distributed":  # Wrapper of distributed planning solver class
                    print("***Run Distributed Planning***")
                    solver = DistributedPlanningSolver(my_map, starts, goals) #!!!TODO: add your own distributed planning implementation here.
                    paths = solver.find_solution()
                else: 
                    raise RuntimeError("Unknown solver!")
                    
        d = {'costs': costs, 
                 'finish times': finishtimes,
                 'success': success,
                 'cpu_times': cpu_times,
                 'agents': agentsnumber
                 }
        df = pd.DataFrame(data = d)
        # print(paths)
        # print(costs)
        # print(finishtimes)
        # print(df)

    #     if not args.batch:
    #         print("***Test paths on a simulation***")
    #         animation = Animation(my_map, starts, goals, paths)
    #         # animation.save("output.mp4", 1.0) # install ffmpeg package to use this option
    #         animation.show()
    # result_file.close()
    
def cost_plot(df):
    """"
    This function takes the large created dataframe, and categorizes it by grouping the 
    minimum, maximum and mean value for each number of agents. These values are then plotted. 
    """
    costs_max = df.groupby('agents')['costs'].max().plot(kind = 'line')
    costs_min = df.groupby('agents')['costs'].min().plot(kind = 'line')
    costs_average = df.groupby('agents')['costs'].mean().plot(kind = 'line', xlabel = 'agents [-]', ylabel = 'Total cost [-]', ylim = 0, title = 'Total costs for prioritized planning per number of agents ')
    plt.legend(['maximum', 'minimum' , 'mean'])

    manual = False
    if manual:
            if args.solver == "CBS":
                print("***Run CBS***")
                cbs = CBSSolver(my_map, starts, goals)
                paths = cbs.find_solution(args.disjoint)
            elif args.solver == "Independent":
                print("***Run Independent***")
                solver = IndependentSolver(my_map, starts, goals)
                paths = solver.find_solution()
            elif args.solver == "Prioritized":
                print("***Run Prioritized***")
                solver = PrioritizedPlanningSolver(my_map, starts, goals)
                paths = solver.find_solution()
            elif args.solver == "Distributed":  # Wrapper of distributed planning solver class
                print("***Run Distributed Planning***")
                solver = DistributedPlanningSolver(my_map, starts, goals) #!!!TODO: add your own distributed planning implementation here.
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
        result_file.close()
    
    costs_max = df.groupby('agents')['costs'].max()
    costs_min = df.groupby('agents')['costs'].min()
    
    costs_max = costs_max.rename_axis(index=None)
    costs_min = costs_min.rename_axis(index=None)
    
    plt.fill_between(range(2,11),costs_max, costs_min, color='grey', alpha=0.5)
    
   
   
    #plt.fill_between(costs_max,costs_min)
    successrate = df.groupby('agents')['success'].mean()
    finishtimes = df.groupby('agents')['finish times'].mean()

    plt.show()

def succes_plot(df):
    """"
    This function takes the large created dataframe, and categorizes it by grouping the 
    minimum, maximum and mean value for each number of agents. These values are then plotted. 
    """
    plt.figure()
    successrate = df.groupby('agents')['success'].mean().plot(kind = 'line', xlabel = 'agents [-]', ylabel = 'Success rate [-]', ylim = 0, title = 'Success rate for prioritized planning per number of agents ')
    plt.show()

def finishtimes_plot(df):
    plt.figure()
    finishtime_max = df.groupby('agents')['finish times'].max().plot(kind = 'line')
    finishtime_min = df.groupby('agents')['finish times'].min().plot(kind = 'line')
    finishtime_average = df.groupby('agents')['finish times'].mean().plot(kind = 'line', xlabel = 'agents [-]', ylabel = 'finish time [-]', ylim = 0, title = 'Finish times for prioritized planning per number of agents ')
    
    finishtime_max = df.groupby('agents')['finish times'].max()
    finishtime_min = df.groupby('agents')['finish times'].min()
    
    finishtime_max = finishtime_max.rename_axis(index=None)
    finishtime_min = finishtime_min.rename_axis(index=None)
    
    plt.fill_between(range(2,11),finishtime_max, finishtime_min, color='grey', alpha=0.5)
    
def cpu_times_plot(df):
    plt.figure() 
    costs_max = df.groupby('agents')['cpu_times'].mean().plot(kind = 'line', ylim = 0, title = 'Cpu times for prioritized planning per number of agents ', ylabel = 'cpu_time [s]', xlabel = 'agents [-]')
    

cost_plot(df)
succes_plot(df)
finishtimes_plot(df)
cpu_times_plot(df)



    