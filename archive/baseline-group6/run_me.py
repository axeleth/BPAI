"""
Run-me.py is the main file of the simulation. Run this file to run the simulation.

This code is part of the assignment of the course AE4422-20 Agent-based Modelling and Simulation in Air Transport (2021/22)

This is the work of group 6

Student names and student numbers: Florina Sirghi   (4648579)
                                   Vincent van Gorp (4777697)
"""

import os
from numpy import number
import pandas as pd
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import time as timer
import pygame as pg
from single_agent_planner import calc_heuristics, get_sum_of_cost
from visualization import map_initialization, map_running
from Aircraft import Aircraft
from independent import run_independent_planner
from prioritized import run_prioritized_planner
from cbs import CBSSolver
import random as rand
from individual_planner import Individual_Planner
import openpyxl as pxl

#%% SET SIMULATION PARAMETERS
#Input file names (used in import_layout) -> Do not change those unless you want to specify a new layout.
nodes_file = "nodes.xlsx" #xlsx file with for each node: id, x_pos, y_pos, type
edges_file = "edges.xlsx" #xlsx file with for each edge: from  (node), to (node), length

# Parameters that can be changed:
simulation_time = 30
number_of_aircraft = 10  # the number of aircraft that will be spawned
max_spawntime = 10  # the number of aircraft that will be spawned
planner = "CBS"  # choose which planner to use: "Independent", "Prioritized", "CBS" or "Individual"
number_of_simulations = 1  # how many simulations will be run in order to obtain data for the evaluation of model variants
                            # please set the number_of_simulations to 1 if you would like to visualise only one simulation



# Initialising the lists for the performance indicators that will be used to evaluate the model
runnmb = 0

runnmb_lst = []  # the list in which we will store all the numbers identifying the runs that have been performed,
                 # for evaluation purposes (starting from run 0, run 1 .. etc.)

mean_taxitime_lst = []  # the list in which we will store the mean taxiing time of all the agents, for each run
variance_taxitime_lst = []  # the list in which we will store the variance in the taxiing time of all the agents, for each run
variance_cost_lst = []  # the list in which we will store the variance in the replanning cost for the agents, for each run
cpu_time_lst = []  # the list in which we will store the CPU time for each run

coefvar_taxitime_lst_mean = [] # the list in which we will store the coefficient of varition for the mean taxiing time for each run
coefvar_taxitime_lst_variance = []  # the list in which we will store the coefficient of varition for the variance in taxiing time
                                    # for each run
coefvar_cost_lst = []  # the list in which we will store the coefficient of variation for the cost of the replanning of the aircraft
coefvar_cpu_lst = []  # the list in which we will store the coefficient of variation for the CPU time


for simulation in range(number_of_simulations):


    runnmb += 1

    #Visualization (can also be changed)
    plot_graph = False    # show graph representation in NetworkX
    visualization =  True  # pygame visualization # please set this back to True if you would like to visualise the solution
    make_excel_files = True  # please set this parameter to "False" if you want to visualise only one run and not save the
                             # data to the Excel files
    visualization_speed = 0.1  # set at 0.1 as default

    time_start = timer.time()

    #%%Function definitions
    def import_layout(nodes_file, edges_file):
        """
        Imports layout information from xlsx files and converts this into dictionaries.
        INPUT:
            - nodes_file = xlsx file with node input data
            - edges_file = xlsx file with edge input data
        RETURNS:
            - nodes_dict = dictionary with nodes and node properties
            - edges_dict = dictionary with edges annd edge properties
            - start_and_goal_locations = dictionary with node ids for arrival runways, departure runways and gates
        """
        gates_xy = []   #lst with (x,y) positions of gates
        rwy_dep_xy = [] #lst with (x,y) positions of entry points of departure runways
        rwy_arr_xy = [] #lst with (x,y) positions of exit points of arrival runways

        df_nodes = pd.read_excel(os.getcwd() + "/" + nodes_file, engine='openpyxl') # had to specify the engine for it to work on PyCharm
        df_edges = pd.read_excel(os.getcwd() + "/" + edges_file, engine='openpyxl')

        #Create nodes_dict from df_nodes
        nodes_dict = {}
        for i, row in df_nodes.iterrows():
            node_properties = {"id": row["id"],
                            "x_pos": row["x_pos"],
                            "y_pos": row["y_pos"],
                            "xy_pos": (row["x_pos"],row["y_pos"]),
                            "type": row["type"],
                            "neighbors": set()
                            }
            node_id = row["id"]
            nodes_dict[node_id] = node_properties

            #Add node type
            if row["type"] == "rwy_d":
                rwy_dep_xy.append((row["x_pos"],row["y_pos"]))
            elif row["type"] == "rwy_a":
                rwy_arr_xy.append((row["x_pos"],row["y_pos"]))
            elif row["type"] == "gate":
                gates_xy.append((row["x_pos"],row["y_pos"]))

        #Specify node ids of gates, departure runways and arrival runways in a dict
        start_and_goal_locations = {"gates": gates_xy,
                                    "dep_rwy": rwy_dep_xy,
                                    "arr_rwy": rwy_arr_xy}

        #Create edges_dict from df_edges
        edges_dict = {}
        for i, row in df_edges.iterrows():
            edge_id = (row["from"],row["to"])
            from_node =  edge_id[0]
            to_node = edge_id[1]
            start_end_pos = (nodes_dict[from_node]["xy_pos"], nodes_dict[to_node]["xy_pos"])
            edge_properties = {"id": edge_id,
                            "from": row["from"],
                            "to": row["to"],
                            "length": row["length"],
                            "weight": row["length"],
                            "start_end_pos": start_end_pos
                            }
            edges_dict[edge_id] = edge_properties

        #Add neighbor nodes to nodes_dict based on edges between nodes
        for edge in edges_dict:
            from_node = edge[0]
            to_node = edge[1]
            nodes_dict[from_node]["neighbors"].add(to_node)

        return nodes_dict, edges_dict, start_and_goal_locations

    def create_graph(nodes_dict, edges_dict, plot_graph = True):
        """
        Creates networkX graph based on nodes and edges and plots
        INPUT:
            - nodes_dict = dictionary with nodes and node properties
            - edges_dict = dictionary with edges annd edge properties
            - plot_graph = boolean (True/False) If True, function plots NetworkX graph. True by default.
        RETURNS:
            - graph = networkX graph object
        """

        graph = nx.DiGraph() #create directed graph in NetworkX

        #Add nodes and edges to networkX graph
        for node in nodes_dict.keys():
            graph.add_node(node,
                        node_id = nodes_dict[node]["id"],
                        xy_pos = nodes_dict[node]["xy_pos"],
                        node_type = nodes_dict[node]["type"])

        for edge in edges_dict.keys():
            graph.add_edge(edge[0], edge[1],
                        edge_id = edge,
                        from_node =  edges_dict[edge]["from"],
                        to_node = edges_dict[edge]["to"],
                        weight = edges_dict[edge]["length"])

        #Plot networkX graph
        if plot_graph:
            plt.figure()
            node_locations = nx.get_node_attributes(graph, 'xy_pos')
            nx.draw(graph, node_locations, with_labels=True, node_size=100, font_size=10)

        return graph

    #%% RUN SIMULATION
    # =============================================================================
    # 0. Initialization
    # =============================================================================
    nodes_dict, edges_dict, start_and_goal_locations = import_layout(nodes_file, edges_file)
    graph = create_graph(nodes_dict, edges_dict, plot_graph)
    heuristics = calc_heuristics(graph, nodes_dict)

    aircraft_lst = []   # Initialising the list that will eventually contain all aircraft agents

    if visualization:
        map_properties = map_initialization(nodes_dict, edges_dict) #visualization properties

    # =============================================================================
    # 1. While loop and visualization
    # =============================================================================

    #Start of while loop
    running=True
    escape_pressed = False
    time_end = simulation_time
    dt = 0.1  # should be factor of 0.5 (0.5/dt should be integer)
    t= 0

    # Making random timesteps for spawning the aircraft - deciding the spawning moments for all aircraft; these are not
    # changed anywhere else in the code
    timesteps = []
    for i in range(0, number_of_aircraft):
        random_timestep = rand.randint(1, max_spawntime)
        while timesteps.count(random_timestep) >= 6:  # Ensuring that a timestep cannot be repeated more than 6 times in the list because
                                               # a maximum of 6 aircraft (5 departing and 1 arriving) can be spawned at the same timestep
            random_timestep = rand.randint(1, max_spawntime)

        timesteps.append(random_timestep)  # this determines when the aircraft will spawn

    timesteps = sorted(timesteps)  # ensuring that the timesteps are in chronological order


    look_up_table_times_locations = []  # this list is used for preventing aircraft to spawn at the same time at the same location
    constraints = []  # initialising the list of constraints
    collisions = []  # initialising list to keep track of collisions

    suboptimal_solution_for_collision = []  # this list is specifically for the "Individual" planner and its role is to keep track of
                                            # whether a collision was solved by the algorithm in a suboptimal way (for example by making
                                            # the aircraft wait for too long at a certain position or even, very rarely, let a collision
                                            # happen if it doesn't find another solution)

    existent_paths = [[] for i in range(len(timesteps))]  # initialising a list of lists to keep track of the existent paths of each agent
                                                          # for the CBS and the Individual planner

    result = []  # here the final results (in the form of a list of paths for each agent) are going to be stored
    starts = []  # the starting node for each agent is stored here
    goals = []   # the goal node for each agent is stored here
    original_path_lst = []  # the length of the original path (calculated without constraints) for each agent is stored here

    current_time = timer.time()

    print("Simulation Started")
    while running:
        t = round(t, 2)

        CPUtime = current_time - time_start  # recalculating the CPU time

        # Check conditions for termination
        if t >= time_end or escape_pressed or (CPUtime>20 and visualization==False): # imposing that the run is terminated if the
                                                                                     # CPU time is larger than 20 seconds and the
                                                                                     # visualisation is switched off
                                                                                     # (for evaluation purposes only)
            running = False
            pg.quit()
            print("Simulation Stopped", runnmb)
            break

            # Visualization: Update map if visualization is true
        if visualization:
            current_states = {}  # Collect current states of all aircraft
            for ac in aircraft_lst:
                if ac.status == "taxiing":
                    current_states[ac.id] = {"ac_id": ac.id,
                                            "xy_pos": ac.position,
                                            "heading": ac.heading}
            escape_pressed = map_running(map_properties, current_states, t)
            timer.sleep(visualization_speed)


        for i in range(len(timesteps)):

            if t == timesteps[i]:  # if we have an aircraft that is spawned at that moment

                startpositionsD = [34, 35, 36, 97, 98]  # the gate nodes, which are the starting positions for departing aircraft
                goalpositionsD = [1, 2]  # the runway entry points, which are the goals for departing aircraft
                startpositionsA = [37, 38]  # the runway exit points, which are the starting positions for arriving aircraft
                goalpositionsA = [34, 35, 36, 97, 98]  # the gates, which are the goal positions for arriving aircraft

                switch = rand.randint(0, 1)  # randomising whether the aircraft that will be spawned will be an arriving or a departing
                                             # aircraft

                if [t, startpositionsD[0]] in look_up_table_times_locations and [t, startpositionsD[1]] in look_up_table_times_locations\
                        and [t, startpositionsD[2]] in look_up_table_times_locations:
                    switch = 1  # the aircraft will be an arriving one
                elif [t, startpositionsA[0]] in look_up_table_times_locations or [t, startpositionsA[1]] in look_up_table_times_locations:
                    # with the above statement, we ensure that aircraft do not spawn at the same time on the arrival runway
                    switch = 0  # the aircraft will be a departing one



                if switch == 0:
                    # the simulation will spawn a departing aircraft.
                    starting_positionD = startpositionsD[rand.randint(0, len(startpositionsD) - 1)]

                    while [t, starting_positionD] in look_up_table_times_locations:
                        starting_positionD = startpositionsD[rand.randint(0, len(startpositionsD) - 1)]  # we get a combination of
                                                           # spwaning timestep and starting position that has not been used so far

                    acD = Aircraft(i, 'D', starting_positionD,
                                goalpositionsD[rand.randint(0, len(goalpositionsD) - 1)], t, nodes_dict)  # we generate the aircraft object
                    aircraft_lst.append(acD)  # and then append it to the aircraft list
                    starts.append(acD.start)
                    goals.append(acD.goal)
                    look_up_table_times_locations.append([t, starting_positionD])  # we also append it to the lookup table to ensure
                                                            # that only one aircraft will be spawn at that location at that timestep

                if switch == 1:
                    # the simulation will spawn an arriving aircraft.
                    starting_positionA = startpositionsA[rand.randint(0, len(startpositionsA) - 1)]

                    while [t, starting_positionA] in look_up_table_times_locations:
                        starting_positionA = startpositionsA[rand.randint(0, len(startpositionsA) - 1)]
                    acA = Aircraft(i, 'A', starting_positionA,
                                goalpositionsA[rand.randint(0, len(goalpositionsA) - 1)], t, nodes_dict)
                    aircraft_lst.append(acA)
                    starts.append(acA.start)
                    goals.append(acA.goal)
                    look_up_table_times_locations.append([t, starting_positionA])

        # Do planning

        if planner == "Independent":
            for y in range(len(timesteps)):
                if t == timesteps[y]:  # (Hint: Think about the condition that triggers (re)planning)
                    run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t)

        elif planner == "Prioritized":
            # Running the prioritized planner first, then saving the final obtained paths as results
            run_prioritized_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, constraints, timesteps, result, original_path_lst)
            if run_prioritized_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, constraints, timesteps, result, original_path_lst) is not None:
                results=run_prioritized_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, constraints, timesteps, result, original_path_lst)

        elif planner == "CBS":
            # Running the CBS planner
                    if t%0.5==0: # at every timestep when an agent reaches a node (multiple of 0.5 sec)
                        current_node = 0
                        current_location_aircraft = [] # we create a list that stores all the current locations of the aircraft
                        for ac in aircraft_lst:
                            if ac.status == "taxiing":
                                id = ac.id
                                position = ac.position
                                for i in range(1, 109):
                                    if nodes_dict[i]['xy_pos'] == position:
                                        current_node = nodes_dict[i]['id']
                                current_location_aircraft.append((current_node, t, id)) # appending the current node, the time
                                                                      # and the id of the agents to the current locations list
                        # generating the CBSSolver object:
                        cbs = CBSSolver(nodes_dict, edges_dict, aircraft_lst, heuristics, t, starts, goals, current_location_aircraft)
                        paths = cbs.find_solution(constraints, existent_paths,current_location_aircraft, collisions, time_start, original_path_lst)

                        if paths is not None:
                            results = paths

        elif planner == "Individual":
            # Running the Individual planner
            for y in range(len(timesteps)):
                if t == timesteps[y]:
                    run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t)  # first, we would like to obtain the
                                                                                                  # original paths of all aircraft
            for ac in aircraft_lst:
                if ac.status=="taxiing" and ac.spawntime==t:
                    original_path = ac.path_to_goal
                    original_path_lst.append((len(original_path) + 1) / 2 - 0.5)  # then, we append the length of the original path
                                                                                  # (in seconds!) to the original_path_lst in order to
                                                                                  # use it for calculating the cost of path replanning
                                                                                  # later, for evaluation purposes
            current_time = timer.time()

            individual_planner = Individual_Planner(nodes_dict, edges_dict, heuristics, aircraft_lst, t, constraints)
            suboptimal_solution_for_collision.append(individual_planner.planner(existent_paths, time_start))

            for ac in aircraft_lst:
                if ac.status=="arrived":
                    if (ac.path_to_goal[-1][0], ac.path_to_goal[-1][1], ac.id) not in existent_paths[ac.id]:
                        existent_paths[ac.id].append((ac.path_to_goal[-1][0], ac.path_to_goal[-1][1], ac.id))  # adding the last element
                                       # in the path of each aircraft (the goal node, the arrival time, ac.id) to the existent paths lsit

            results = existent_paths

        else:
            raise Exception("Planner:", planner, "is not defined.")

        # Move the aircraft that are taxiing
        for ac in aircraft_lst:
            if ac.status == "taxiing":
                ac.move(dt, t)

        current_time = timer.time()
        t = t + dt


    # Please uncomment the following statement if you would like to print the list of paths for all aircraft
    # print("These are the paths for all aircraft", results)

    # =============================================================================
    # 2. Implement analysis of output data here
    # =============================================================================
    # what data do you want to show?

    if "True" in suboptimal_solution_for_collision:
        print("Suboptimal solution for collision encountered during this run")

    length_of_paths_per_agent = []  # initialising a list that will store the length (in seconds!) of the path of
                                    # each aircraft -> this is actually the taxiing time of each agent
    for path in results:
        length_of_paths_per_agent.append(len(path)/2 - 0.5)

    cost_replanning = np.array(length_of_paths_per_agent) - np.array(original_path_lst)  # here, we are calculating the
                                                                                         # "cost of replanning" per agent, defined as
                                                                                         # the time difference between the actual
                                                                                         # path the aircraft followed in the end,
                                                                                         # and its originally planned path
    current_time = timer.time()

    runnmb_lst.append(runnmb)

    ############# Output data for the mean and variance taxiing time ############

    mean_taxitime = np.round(np.mean(length_of_paths_per_agent), 3)
    mean_taxitime_lst.append(mean_taxitime)
    variance_taxitime = np.round(np.var(length_of_paths_per_agent), 3)
    variance_taxitime_lst.append(variance_taxitime)

    coefvar_taxitime_mean = np.sqrt(np.round(np.var(mean_taxitime_lst), 3)) / np.round(np.mean(mean_taxitime_lst), 3)
    coefvar_taxitime_lst_mean.append(coefvar_taxitime_mean)

    coefvar_taxitime_variance = np.sqrt(np.round(np.var(variance_taxitime_lst), 3)) / np.round(
        np.mean(variance_taxitime_lst), 3)
    coefvar_taxitime_lst_variance.append(coefvar_taxitime_variance)

    ############# Output data for the CPU time ############

    cpu_time = np.round(current_time- time_start, 3)
    cpu_time_lst.append(cpu_time)
    coefvar_cpu = np.sqrt(np.round(np.var(cpu_time_lst), 3)) / np.round(np.mean(cpu_time_lst), 3)
    coefvar_cpu_lst.append(coefvar_cpu)

    ############# Output data for the variance in the replanning cost (delays caused by replanning/ adhering to constraints) ############

    variance_cost = np.round(np.var(cost_replanning), 3)
    variance_cost_lst.append(variance_cost)
    coefvar_cost = np.sqrt(np.round(np.var(variance_cost_lst), 3)) / np.round(np.mean(variance_cost_lst), 3)
    coefvar_cost_lst.append(coefvar_cost)

############# Compiling the evaluation data dictionary, that will be used for evaluation #############

evaluation_data =  {'Run': runnmb_lst, 'Number of Aircraft': number_of_aircraft,
                    'Maximum Spawn Time': max_spawntime, 'Mean Taxitime': mean_taxitime_lst,
                    'Variance Taxitime': variance_taxitime_lst,
                    'Variance Cost': variance_cost_lst, 'CPU Time': cpu_time_lst}


df = pd.DataFrame(evaluation_data)
df = df.set_index('Run')

# Creating the plots for the coefficient of variation of the four chosen performance indicators, which will be displayed in the Excel files

fig, axes  = plt.subplots(2,2)
axes[0,0].plot(runnmb_lst, coefvar_taxitime_lst_mean)
axes[0,0].set(title = "$C_{v}$" ' mean taxi time',
        xlabel = 'Number of runs',
        ylabel = "$C_{v}$")
axes[0,1].plot(runnmb_lst, coefvar_taxitime_lst_variance, color='red')
axes[0,1].set(title = "$C_{v}$" ' variance taxi time',
        xlabel = 'Number of runs',
        ylabel = "$C_{v}$")
axes[1,0].plot(runnmb_lst, coefvar_cost_lst, color= 'green')
axes[1,0].set(title = "$C_{v}$" ' cost',
        xlabel = 'Number of runs',
        ylabel = "$C_{v}$")
axes[1,1].plot(runnmb_lst, coefvar_cpu_lst, color= 'black')
axes[1,1].set(title = "$C_{v}$" ' CPU time',
        xlabel = 'Number of runs',
        ylabel = "$C_{v}$")


fig.tight_layout(pad=2.0)

plt.savefig('coefvar.png')  # Saving the figure

############# Creating the Excel files #############

if make_excel_files == True:
    if planner == "Prioritized":
        plt.savefig('coefvar_prioritized.png')
        writer = pd.ExcelWriter('Data_Prioritized.xlsx', engine= 'xlsxwriter')
        df.to_excel(writer, sheet_name='Data Prioritized')
        worksheet = writer.sheets['Data Prioritized']
        worksheet.insert_image('M3', 'coefvar_prioritized.png')
        writer.save()

    if planner == "CBS":
        plt.savefig('coefvar_CBS.png')
        writer = pd.ExcelWriter('Data_CBS.xlsx', engine= 'xlsxwriter')
        df.to_excel(writer, sheet_name='Data CBS')
        worksheet = writer.sheets['Data CBS']
        worksheet.insert_image('M3', 'coefvar_CBS.png')
        writer.save()

    if planner == "Individual":
        plt.savefig('coefvar_individual.png')
        writer = pd.ExcelWriter('Data_Individual.xlsx', engine= 'xlsxwriter')
        df.to_excel(writer, sheet_name='Data Individual')
        worksheet = writer.sheets['Data Individual']
        worksheet.insert_image('M3', 'coefvar_individual.png')
        writer.save()
