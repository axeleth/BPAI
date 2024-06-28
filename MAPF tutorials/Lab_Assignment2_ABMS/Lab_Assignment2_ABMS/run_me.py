"""
Run-me.py is the main file of the simulation. Run this file to run the simulation.
"""

import os
import pandas as pd
import numpy as np
import statistics
import networkx as nx
import matplotlib.pyplot as plt
import time as timer
import pygame as pg
import random as rd
from single_agent_planner import calc_heuristics
from visualization import map_initialization, map_running
from Aircraft import Aircraft
from independent import run_independent_planner
from prioritized import run_prioritized_planner
from cbs import CBSSolver
from deviate import *

#%% SET SIMULATION PARAMETERS
#Input file names (used in import_layout) -> Do not change those unless you want to specify a new layout.
nodes_file = "nodes.xlsx" #xlsx file with for each node: id, x_pos, y_pos, type
edges_file = "edges.xlsx" #xlsx file with for each edge: from  (node), to (node), length

#Parameters that can be changed:
simulation_time = 35
planner = "CBS" #choose which planner to use (currently only Independent is implemented)

#Visualization (can also be changed)
plot_graph = False    #show graph representation in NetworkX
visualization = True        #pygame visualization
visualization_speed = 0.01 #set at 0.1 as default (0.1 is very slow now because there are a lot more steps!)



#evaluation metrics
cpu_time_list = []

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

    df_nodes = pd.read_excel(os.getcwd() + "/" + nodes_file)
    df_edges = pd.read_excel(os.getcwd() + "/" + edges_file)

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

number_of_aircraft = 20
runnumb = 1
while runnumb <= 5:


    #%% RUN SIMULATION
    # =============================================================================
    # 0. Initialization
    # =============================================================================
    nodes_dict, edges_dict, start_and_goal_locations = import_layout(nodes_file, edges_file)
    graph = create_graph(nodes_dict, edges_dict, plot_graph)
    heuristics = calc_heuristics(graph, nodes_dict)

    aircraft_lst = []   #List which can contain aircraft agents
    cpu_time_list = []  #List which can contain cpu times of the planner

    if visualization:
        map_properties = map_initialization(nodes_dict, edges_dict) #visualization properties

    # =============================================================================
    # 1. While loop and visualization
    # =============================================================================

    #Start of while loop
    running=True
    escape_pressed = False
    time_end = simulation_time
    dt = 0.05 #should be factor of 0.5 (0.5/dt should be integer)
    t= 0 

    max_spawntime = 20  # maximum spawntime for the aircraft
    # number_of_aircraft = 6  # number of aircraft to spawn
    timesteps = [] # timesteps at which the aircraft spawn

    for i in range(0, number_of_aircraft):
        random_timestep = rd.randint(1, max_spawntime)
        while timesteps.count(random_timestep) >= 6:  # Ensuring that a timestep cannot be repeated more than 6 times in the list because
                                                # a maximum of 6 aircraft (5 departing and 1 arriving) can be spawned at the same timestep
            random_timestep = rd.randint(1, max_spawntime)

        timesteps.append(random_timestep)  # this determines when the aircraft will spawn

    timesteps = sorted(timesteps)  # ensuring that the timesteps are in chronological order


    look_up_table_times_locations = []  # this list is used for preventing aircraft to spawn at the same time at the same location
    constraints = []  # constraint list init
    collisions = []  # collision list init

    existent_paths = [[] for i in range(len(timesteps))]    # initialising a list of lists to keep track of the existent paths of each agent
                                                            # for the CBS and the Individual planner


    # result = []  # here the final results (in the form of a list of paths for each agent) are going to be stored
    starts = []  # the starting node for each agent is stored here
    goals = []   # the goal node for each agent is stored here
    original_path_lst = []  # the length of the original path (calculated without constraints) for each agent is stored here



    # id = 0
    # new_aircraft = False

    print("Simulation Started")
    while running:
        t= round(t,2)
        if t%1 == 0:
            print("Time: ", t)

        #Check conditions for termination
        if t >= time_end or escape_pressed:
            running = False
            pg.quit()
            print("Simulation Stopped")
            break

        #Visualization: Update map if visualization is true
        if visualization:
            current_states = {} #Collect current states of all aircraft
            for ac in aircraft_lst:
                if ac.status == "taxiing":
                    current_states[ac.id] = {"ac_id": ac.id,
                                            "xy_pos": ac.position,
                                            "heading": ac.heading}
            escape_pressed = map_running(map_properties, current_states, t)
            timer.sleep(visualization_speed)
            # print(current_states)

    # =============================================================================
    # SPAWNING A SINGLE EXAMPLE AIRCRAFT
        # #Spawn aircraft for this timestep (use for example a random process)
        # if t == 1 :
        #     ac0 = Aircraft(0, 'D', 'cS', 98,37, timesteps[0], nodes_dict)
        #     aircraft_lst.append(ac0)
        #     starts.append(ac0.start)
        #     goals.append(ac0.goal)

    # =============================================================================

        for i in range(len(timesteps)):

                if t == timesteps[i]:  # if we have an aircraft that is spawned at that moment

                    startpositionsD = [34, 35, 36, 97, 98]  # the gate nodes, which are the starting positions for departing aircraft
                    goalpositionsD = [1, 2]  # the runway entry points, which are the goals for departing aircraft
                    startpositionsA = [37, 38]  # the runway exit points, which are the starting positions for arriving aircraft
                    goalpositionsA = [34, 35, 36, 97, 98]  # the gates, which are the goal positions for arriving aircraft

                    # startpositionsA = [1, 2]  # the runway entry points, which are the goals for departing aircraft
                    # goalpositionsD = [37, 38]  # the runway exit points, which are the starting positions for arriving aircraft

                    # Profiles
                    departing_profiles = ['fast', 'slow', 'cS'] #, 'cS'
                    arriving_profiles = ['fast', 'slow', 'cE', 'fS'] #, 'cE', 'fS'

                    switch = rd.randint(0, 1)  # randomising whether the aircraft that will be spawned will be an arriving or a departing
                                                # aircraft

                    if [t, startpositionsD[0]] in look_up_table_times_locations and [t, startpositionsD[1]] in look_up_table_times_locations\
                            and [t, startpositionsD[2]] in look_up_table_times_locations:
                        switch = 1  # the aircraft will be an arriving one
                    elif [t, startpositionsA[0]] in look_up_table_times_locations or [t, startpositionsA[1]] in look_up_table_times_locations:
                        # with the above statement, we ensure that aircraft do not spawn at the same time on the arrival runway
                        switch = 0  # the aircraft will be a departing one



                    if switch == 0:
                        # the simulation will spawn a departing aircraft.
                        starting_positionD = startpositionsD[rd.randint(0, len(startpositionsD) - 1)]

                        while [t, starting_positionD] in look_up_table_times_locations:
                            starting_positionD = startpositionsD[rd.randint(0, len(startpositionsD) - 1)]  # we get a combination of
                                                            # spwaning timestep and starting position that has not been used so far

                        profileD = departing_profiles[rd.randint(0, len(departing_profiles) - 1)]  # we get a random profile for the departing aircraft

                        acD = Aircraft(i, 'D', profileD, starting_positionD,
                                    goalpositionsD[rd.randint(0, len(goalpositionsD) - 1)], t, nodes_dict)  # we generate the aircraft object
                        aircraft_lst.append(acD)  # and then append it to the aircraft list
                        starts.append(acD.start)
                        goals.append(acD.goal)
                        look_up_table_times_locations.append([t, starting_positionD])  # we also append it to the lookup table to ensure
                                                                # that only one aircraft will be spawn at that location at that timestep

                    if switch == 1:
                        # the simulation will spawn an arriving aircraft.
                        starting_positionA = startpositionsA[rd.randint(0, len(startpositionsA) - 1)]

                        while [t, starting_positionA] in look_up_table_times_locations:
                            starting_positionA = startpositionsA[rd.randint(0, len(startpositionsA) - 1)]

                        profileA = arriving_profiles[rd.randint(0, len(arriving_profiles) - 1)]  # we get a random profile for the arriving aircraft
                        acA = Aircraft(i, 'A', profileA, starting_positionA,
                                    goalpositionsA[rd.randint(0, len(goalpositionsA) - 1)], t, nodes_dict)
                        aircraft_lst.append(acA)
                        starts.append(acA.start)
                        goals.append(acA.goal)
                        look_up_table_times_locations.append([t, starting_positionA])


        # Agent decides to change speed
        if t%0.5 == 0:
            for ac in aircraft_lst:
                check_deviation(ac)

        #Do planning
        planning = False
        if planner == "CBS":
                an_aircraft_deviated = detect_deviation(aircraft_lst, nodes_dict, t)
                if t in timesteps or an_aircraft_deviated:
                    planning = True
                    planner_start_time = timer.time()
                    for ac in aircraft_lst:
                        if ac.spawntime == t:
                            ac.status = "taxiing"
                            ac.position = nodes_dict[ac.start]["xy_pos"]
                            # print("AC{} has spawned at time {}".format(ac.id, t))

                    current_node = 0
                    current_location_aircraft = [] # this will append the current location of each taxiing agent at the given t
                    for ac in aircraft_lst:
                        if ac.status == "taxiing":
                            id = ac.id
                            position = ac.position
                            # if is_it_a_node(position, nodes_dict) == True:
                            for i in range(1,109):
                                if nodes_dict[i]["xy_pos"] == position:
                                    current_node = nodes_dict[i]["id"]
                            
                            current_location_aircraft.append((current_node, t, id))
                            # print("current_location_aircraft:",current_location_aircraft)
                    
                    if  an_aircraft_deviated and current_location_aircraft != [] and current_location_aircraft is not None:
                        for ac in aircraft_lst:
                            print("AC{} deviation == {}".format(ac.id, ac.deviated))
                        current_location_aircraft = prep_CBS_replan(aircraft_lst, current_location_aircraft)
                    
                    # if an_aircraft_deviated:
                    #     raise Exception("Current_location_aircraft:", current_location_aircraft)
                    
                    

                    cbs = CBSSolver(nodes_dict, edges_dict, aircraft_lst, heuristics, t, starts, goals, current_location_aircraft)
                    paths = cbs.find_solution(constraints, existent_paths, current_location_aircraft, collisions, time_start, original_path_lst, an_aircraft_deviated)
                    
                    if paths is not None:
                        results = paths
                        print("results:\n",results)

                
        if aircraft_lst != []:
            for ac in aircraft_lst:
                if ac.status == "taxiing":
                    ac.move(dt, t)

        # current_time = timer.time()
        t = t + dt # Time update
        
        if planning:
            cpu_time = timer.time() - planner_start_time
            cpu_time_list.append(cpu_time)

        # else:
        #     raise Exception("Planner:", planner, "is not defined.")

        #Move the aircraft that are taxiing

    # =============================================================================
    # DATA COLLECTION 
    # =============================================================================


    length_of_paths_per_agent = []
    for ac in aircraft_lst:
        length_of_paths_per_agent.append(ac.distance_travelled + 2)

    cost_of_replanning = np.array(length_of_paths_per_agent) - np.array(original_path_lst)

    for i in range(len(cost_of_replanning)): # The resulting path length is sometimes negative due to the nature of speed-up deviations
        if cost_of_replanning[i] < 0:
            cost_of_replanning[i] += 1

    # print("The orginial path lengths are: ", original_path_lst)
    # print ("The length of the paths per agent is: ", length_of_paths_per_agent)
    # print ("The cost of replanning is: ", cost_of_replanning)


    collect_data = True
    if collect_data:
        # process the data
        number_of_aircraft = number_of_aircraft
        cpu_mean = statistics.mean(cpu_time_list)
        cpu_variance = statistics.variance(cpu_time_list)
        replanning_cost = sum(cost_of_replanning)
        num_replannings = len(cpu_time_list)


        # make a dataframe with the simulation number as the row index, the number of aircraft as the second column and the rest of the columns as the cpu time list of the simulation
        cpu_times = pd.read_csv('cpu_times.csv')


        # add the data to the dataframe as a row
        cpu_times.loc[-1] = [number_of_aircraft, cpu_mean, cpu_variance, replanning_cost, num_replannings]     # adding a row
        cpu_times.index = cpu_times.index + 1                                                                  # set the index
        cpu_times = cpu_times.sort_index()                                                                     # sort the index

        print(cpu_times)

        # Overwrite the data in the csv file
        cpu_times.to_csv('cpu_times.csv', index=False)

    runnumb += 1


    # %%
