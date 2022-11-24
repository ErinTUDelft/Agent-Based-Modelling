# -*- coding: utf-8 -*-
"""
Created on Thu Nov 24 03:01:27 2022

@author: Jelmer
"""
import pandas as pd
from run_experiments import cost_plot, finish_times_plot, succes_plot, cpu_times_plot, agent_range

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

dfs = dict()

for i, algorithm in enumerate(algorithms):
    if not algorithms_used[i]:
        continue
    df = pd.read_csv("analyses/L2-" + algorithm + ".csv")
    dfs[algorithm] = df
    
cost_plot(dfs)
succes_plot(dfs)
finish_times_plot(dfs)
cpu_times_plot(dfs)