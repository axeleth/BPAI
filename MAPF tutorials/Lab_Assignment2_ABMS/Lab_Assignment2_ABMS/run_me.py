"""
Run-me.py is the main file of the simulation. Run this file to run the simulation.
"""

import os
import pandas as pd
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
simulation_time = 20
planner = "CBS" #choose which planner to use (currently only Independent is implemented)

#Visualization (can also be changed)
plot_graph = False    #show graph representation in NetworkX
visualization = True        #pygame visualization
visualization_speed = 0.05 #set at 0.1 as default (0.1 is very slow now because there are a lot more steps!)

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

#%% RUN SIMULATION
# =============================================================================
# 0. Initialization
# =============================================================================
nodes_dict, edges_dict, start_and_goal_locations = import_layout(nodes_file, edges_file)
graph = create_graph(nodes_dict, edges_dict, plot_graph)
heuristics = calc_heuristics(graph, nodes_dict)

aircraft_lst = []   #List which can contain aircraft agents

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

timesteps = [1,1.5,2,2,2.5]

# LUT_TL = initialise lookup table for ac spawn times and locations HERE
constraints = []  # constraint list init
collisions = []  # collision list init

existent_paths = [[] for i in range(len(timesteps))]    # initialising a list of lists to keep track of the existent paths of each agent
                                                        # for the CBS and the Individual planner


# result = []  # here the final results (in the form of a list of paths for each agent) are going to be stored
starts = []  # the starting node for each agent is stored here
goals = []   # the goal node for each agent is stored here
original_path_lst = []  # the length of the original path (calculated without constraints) for each agent is stored here

id = 0
new_aircraft = False

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

    #Spawn aircraft for this timestep (use for example a random process)
    if t == 1 :
        ac0 = Aircraft(0, 'D', 24,37,1,timesteps[0], nodes_dict) #As an example we will create one aicraft arriving at node 37 with the goal of reaching node 36
        aircraft_lst.append(ac0)
        starts.append(ac0.start)
        goals.append(ac0.goal)

        ac1 = Aircraft(1, 'D', 97,2,2,timesteps[1], nodes_dict)#As an example we will create one aicraft arriving at node 36 with the goal of reaching node 37
        aircraft_lst.append(ac1)
        starts.append(ac1.start)
        goals.append(ac1.goal)

        ac2 = Aircraft(2, 'D', 36,37,1,timesteps[2], nodes_dict)#As an example we will create one aicraft arriving at node 36 with the goal of reaching node 37
        aircraft_lst.append(ac2)
        starts.append(ac2.start)
        goals.append(ac2.goal)

        ac3 = Aircraft(3, 'D', 33,37,1,timesteps[3], nodes_dict)#As an example we will create one aicraft arriving at node 36 with the goal of reaching node 37
        aircraft_lst.append(ac3)
        starts.append(ac3.start)
        goals.append(ac3.goal)

        ac4 = Aircraft(4, 'A', 1,35,2,timesteps[4], nodes_dict)#As an example we will create one aicraft arriving at node 36 with the goal of reaching node 37
        aircraft_lst.append(ac4)
        starts.append(ac4.start)
        goals.append(ac4.goal)
        # aircraft_lst.append(ac1)

    # random spawner:
    # if t <= 10 and t%0.25 == 0: # every 0.25 seconds until t = 10
    #     if rd.random() < 0.2: # 20% chance
    #         id, a_d, start, goal, speed, spawn_time = spawner(t, id)
    #         aircraft_lst.append(Aircraft(id, a_d, start, goal, speed, spawn_time, nodes_dict))

    # this is a super simple deviation example hopefully it will turn into a speed profile...
    DEVIATE = False
    if DEVIATE and t == 2: # at time 5.5

        for ac in aircraft_lst:
            if ac.id == 0:
                ac.speed = 2

    # elif DEVIATE and t == 4: # at time 4
    #     for ac in aircraft_lst:
    #         if ac.id == 3:
    #             ac.speed = 2

    #Do planning
    if planner == "CBS":
            if t in timesteps:
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
                        for i in range(1,109):
                            if nodes_dict[i]["xy_pos"] == position:
                                current_node = nodes_dict[i]["id"]
                        current_location_aircraft.append((current_node, t, id))
                
                

                cbs = CBSSolver(nodes_dict, edges_dict, aircraft_lst, heuristics, t, starts, goals, current_location_aircraft)
                paths = cbs.find_solution(constraints, existent_paths, current_location_aircraft, collisions, time_start, original_path_lst)
                
                if paths is not None:
                    results = paths
            
            # IF DEVIATION OCCURS
            if detect_deviation(aircraft_lst, nodes_dict, t): 
                prep_replan(aircraft_lst, nodes_dict, edges_dict, heuristics, t)
                print("aircraft list:",aircraft_lst,
                      "\nt:",t,
                      "\nstarts:",starts,
                      "\ngoals:",goals,
                      "\ncurrent location aircraft:",current_location_aircraft,
                      "\nconstraints:",constraints,
                      "\nexistent paths:",existent_paths,
                      "\ncollisions:",collisions,
                      "\ntime_start:",time_start)
                raise Exception("Aircraft deviated from path!")
                cbs = CBSSolver(nodes_dict, edges_dict, aircraft_lst, heuristics, t, starts, goals, current_location_aircraft)
                paths = cbs.find_solution(constraints, existent_paths, current_location_aircraft, collisions, time_start, original_path_lst)
                
                if paths is not None:
                    results = paths


    # aircraft_infront(aircraft_lst, t) # if if there is about to be a rear end collision, slow down the faster aircraft
            
    if aircraft_lst != []:
        for ac in aircraft_lst:
            if ac.status == "taxiing":
                ac.move(dt, t)

    t = t + dt
            





    # elif planner == "Independent": # and aircraft_lst != []

    #     # you can change the following condition to trigger replanning at a specified time step!!!! Right now it only plans at timestep 1.

    #     # example:
    #     # if t == 1 OR deviation_detected == True:
    #     #   run_independent_planner()

    #     if t == 1:

    #     # this one is here for the random spawner
    #     # if len(aircraft_lst) == 1: #(Hint: Think about the condition that triggers (re)planning)
    #         # if aircraft_lst[0].spawntime == t: # t == 1 is reserved for the initial planning

    #         run_independent_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t)

    #     if len(aircraft_lst) != 1: # This part is here for when new aircraft are spawned in the middle of the simulation
    #         for ac in aircraft_lst:
    #             if ac.spawntime == t and t != 1: # REMOVE T==1 WHEN USING RANDOM SPAWNER
    #                 print("AC{} has spawned at time {}".format(ac.id, t))
    #                 ac.status = "taxiing" # taxiing
    #                 ac.position = nodes_dict[ac.start]["xy_pos"] # position
    #                 # print("now it has position:", ac.position, "and is", ac.status)
    #                 prep_replan(aircraft_lst, nodes_dict, edges_dict, heuristics, t)
    #                 run_independent_replanner(aircraft_lst, nodes_dict, edges_dict, heuristics, t)
    #                 # for ac in aircraft_lst:
    #                 #     print(ac.path_to_goal)

    #     if detect_deviation(aircraft_lst, nodes_dict, t):  # or new_aircraft == True
    #         prep_replan(aircraft_lst, nodes_dict, edges_dict, heuristics, t)
    #         run_independent_replanner(aircraft_lst, nodes_dict, edges_dict, heuristics, t) # Independent RE-planner
    #         new_aircraft = False

            # raise Exception("Aircraft deviated from path!")



    # elif planner == "Prioritized":
    #     run_prioritized_planner()

    #elif planner == -> you may introduce other planners here
    # else:
    #     raise Exception("Planner:", planner, "is not defined.")

    #Move the aircraft that are taxiing

# =============================================================================
# 2. Implement analysis of output data here
# =============================================================================
#what data do you want to show?

# %%
