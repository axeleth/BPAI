"""
This file contains single agent planner functions  that can be used by other planners.
Consider functions in this file as supporting functions.
"""

import heapq
import numpy as np
import networkx as nx

def calc_heuristics(graph, nodes_dict):
    """
    Calculates the exact heuristic dict (shortest distance between two nodes) to be used in A* search.
    INPUT:
        - graph = networkX graph object
        - nodes_dict = dictionary with nodes and node properties
    RETURNS:
        - heuristics = dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
    """
    
    heuristics = {}
    for i in nodes_dict:
        heuristics[nodes_dict[i]["id"]] = {}
        for j in nodes_dict:
            path, path_length = heuristicFinder(graph, nodes_dict[i]["id"], nodes_dict[j]["id"])
            if path == False:
                pass
            else:
                heuristics[nodes_dict[i]["id"]][nodes_dict[j]["id"]] = path_length
    return heuristics

def heuristicFinder(graph, start_node, goal_node):
    """
    Finds exact distance between start_node and goal_node using the NetworkX graph.
    INPUT:
        - graph = networkX graph object
        - start_node, goal_node [int] = node_ids of start and goal node
    RETURNS:
        - path = list with node_ids that are part of the shortest path
        - path_length = length of the shortest path
    """
    try:
        path = nx.dijkstra_path(graph, start_node, goal_node, weight="weight")
        path_length = nx.dijkstra_path_length(graph, start_node, goal_node)
    except:
        path = False
        path_length = False
        raise Exception('Heuristic cannot be calculated: No connection between', start_node, "and", goal_node)
    return path, path_length


def find_time_step_size(speed, dt=0.01):
    step_size = speed*dt; n_steps = 0; acc_dist = 0
    
    while acc_dist < 0.5:
        n_steps += 1
        acc_dist += step_size

    return n_steps*dt

def build_constraint_table(constraints, agent):
    """Generates a table of constraints for an agent"""
    print("BUILDING CONSTRAINT TABLE")
    return_table = []

    for constraint in constraints:
        # check whether the constraint apploes to the agent. if not, the for loop continues with the 
        # next entry in the array consisting of the constraints (dictionaries)
        if constraint['agent'] != agent:
            continue
        print('constraint {} for agent {}:'.format(constraint, agent))
        time_step = constraint['timestep']

        # preparation for the edge constraints. returns a tuple within tuple ((location 1), (location N))
        if len(constraint['loc']) != 1:
            location_lst = []
            for y in range(len(constraint['loc'])):
                location = constraint['loc'][y]
                location_lst.append(location)
            location_tuple = location_lst

        # preparation for the vertex constraints. returns a tuple with one element (location)
        else:
            location_tuple = [constraint['loc'][0]]

        return_table.append([time_step, location_tuple])

    return_table = np.array(return_table, dtype=object)

    if len (return_table) != 0:
        return_table = return_table[return_table[:,0].argsort()] # this gives the sorted constraint table
    
    print("CONSTRAINT RETURN TABLE:", return_table)
    return return_table

def is_constrained(curr_loc, next_loc, next_time, constraint_table, speed):
    """
    Decides whether a certain movement is constrained or not.
    INPUT:
        - curr_loc = current location of the aircraft (node)
        - next_loc = next location planned for the aircraft (node)
        - next_time = timestep at which the aircraft is intended to be at the next location
        - constraint_table = the constraint table for the agent (aircraft) that intends to move
    RETURNS:
        - switch = a boolean parameter that is "True" if the movement should be constrained and "False" otherwise
    """

    next_loc = next_loc
    curr_loc = curr_loc
    switch = False  # parameter that decides whether the movement should be constrained or not
    for i in constraint_table:
        # Checking vertex constraints.
        if len(i[1])==1:
            time_step = i[0]
            constraint_location = i[1][0]
            # print("looking at vertex constraint:", i)
            if constraint_location == next_loc and next_time == time_step:  # if there exists a vertex constraint for the
                                                                            # next location and next timestep
                switch = True  # then the movement should be constrained
        # Checking edge constraints.
        if len(i[1])!=1:  # if the length of the location of the constraint is different than 1 (namely equal to 2),
                          # then we are dealing with an edge constraint
            # print("Exploring Constraint:", i)
            # print("Time:", i[0], '==', next_time)
            # print('location1:', curr_loc, '==', i[1][0])
            # print('location2:', next_loc, '==', i[1][1])
            # print(speed)
            
            time_step = i[0]
            if time_step != next_time: #  and time_step != next_time - find_time_step_size(speed)
                continue
            constraint_location1 = i[1][0]
            # raise Exception('constraint_location1:', constraint_location1, 'curr_loc:', curr_loc)
            if curr_loc != constraint_location1:
                continue
            constraint_location2 = i[1][1]
            if next_loc == constraint_location2:  # if the aircraft intends to move to a position that is constrained
                                                  # due to an edge collision at the next timestep, the next timestep
                switch = True  # then the movement should be constrained
        if switch == True:
            break
    return switch

def complex_single_agent_astar(nodes_dict, from_node, goal_node, heuristics, time_start, agent, speed, constraints, path_so_far):
    
    ### THIS IS USED FOR CBS PLANNER ###

    constraint_table = build_constraint_table(constraints, agent)

    from_node_id = from_node
    goal_node_id = goal_node
    time_start = time_start

    open_list = []
    closed_list = dict()
    earliest_goal_timestep = time_start
    h_value = heuristics[from_node_id][goal_node_id]
    root = {'loc': from_node_id, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': time_start}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root

    while len(open_list) > 0:
        curr = pop_node(open_list)
        if curr['loc'] == goal_node_id and curr['timestep'] >= earliest_goal_timestep:
            return True, get_path(curr, agent)
        

        possible_positions = nodes_dict[curr['loc']]["neighbors"]   # making a list of neighbors of the current node
                                                                    # these are the nodes to which the agent can move
        possible_positions.add(curr['loc']) # adding the current node to the list of possible positions so the agent has the option of waiting

        for neighbor in possible_positions:
            # print("NEIGHBOR:", neighbor)

            # Check whether the new child nodes violates any constraints set by other agents. We check whether the
            # constraint is true, and if so, we discard this child node and move on to the next node using 'continue'.

            path = path_so_far + get_path(curr,agent)  # the path of the agent that will be used for replanning is composed
                                                       # of the path so far and the part of the path that can be retrieved
                                                       # from get_path (starting from the parent of the current node)

        
            back = 0 # parameter is 1 if the agent moves backwards and 0 if it does not
            for k in range(len(path)-1):
                # print("IN SINGLE AGENT PLANNER FILE!!! time step size: {}".format(find_time_step_size(speed)))
                if path[k][1] == curr['timestep'] - find_time_step_size(speed): # CHECK IF THIS CAUSES AN ERROR
                    if path[k][0]==neighbor and path [k+1][0]==curr['loc']: # the agent had the same position as the neighbour,
                                                                            # but then it moved from that position
                        back = 1 # then it means that it intends to move backwards (back to its previous position, at the previous timestep)


                if path[k][1] == curr['timestep']-1 and path[k+1][0] != neighbor: 
                    if path[k][0] == neighbor: 
                        back = 1

            if back == 1:
                # print("neighbor is behind")
                continue

            # checking that the aircraft doesn't wait for an arbitrary amount of timesteps in a loction and only then
            # wants to move backwards to its previous location

            for k in range(len(path)-1):
                if path[k][0]== neighbor and path[k][1]<curr['timestep']:
                    time_1=path[k][1]
                    time_2=curr['timestep']
                    back = 1
                    position = path[k+1][0]
                    i = 2
                    time_step = time_1 + find_time_step_size(speed) 
                    while time_step < time_2:
                        if path[k+i][0] != position:
                            back = 0
                        i += 1
                        time_step += find_time_step_size(speed) 

            if back == 1:
                # print("waiting for too long")
                continue

            back = 0

            if is_constrained(curr['loc'], neighbor, round(curr['timestep'] + find_time_step_size(speed),2), constraint_table,speed) or back == 1:
                print("Neighbor {} is constrained".format(neighbor))
                continue

            child = {
                    'loc': neighbor,
                    'g_val': curr['g_val'] + 0.5, # this will stay the same since the cost is still the same.
                    'h_val': heuristics[neighbor][goal_node_id],
                    'parent': curr,
                    'timestep': round(curr['timestep'] + find_time_step_size(speed),2)}
            
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

            print("Path found for agent {}: {}".format(agent, get_path(child, agent)))

    print("No path found, "+str(len(closed_list))+" nodes visited")
    return False, [] # Failed to find solutions

def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))

def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr

def compare_nodes(n1, n2):
    """Return true if n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

def get_path(goal_node, agent):
    """Construct path if goal node is reached"""
    path = []
    curr = goal_node
    while curr is not None:
        path.append((curr['loc'], curr['timestep'], agent))
        curr = curr['parent']
    path.reverse()
    #print(path)
    return path




def simple_single_agent_astar(nodes_dict, from_node, goal_node, heuristics, time_start, agent, speed, constraints):
    # def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """
    Single agent A* search. Time start can only be the time that an agent is at a node.
    INPUT:
        - nodes_dict = [dict] dictionary with nodes and node properties
        - from_node = [int] node_id of node from which planning is done
        - goal_node = [int] node_id of node to which planning is done
        - heuristics = [dict] dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
        - time_start = [float] planning start time. 
        - speed = [int] speed of the aircraft (1 or 2)
        - Hint: do you need more inputs? 
    RETURNS:
        - success = True/False. True if path is found and False is no path is found
        - path = list of tuples with (loc, timestep) pairs -> example [(37, 1), (101, 2)]. Empty list if success == False.
    """
    
    # print("from: ", from_node, "\ngoal: ", goal_node, "\nstart time: ", time_start)

    constraint_table = build_constraint_table(constraints, agent)

    from_node_id = from_node
    goal_node_id = goal_node
    time_start = time_start
    
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = time_start
    h_value = heuristics[from_node_id][goal_node_id]
    root = {'loc': from_node_id, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': time_start}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root


    while len(open_list) > 0:
        curr = pop_node(open_list)
        if curr['loc'] == goal_node_id and curr['timestep'] >= earliest_goal_timestep:
            return True, get_path(curr, agent)
        
        possible_positions = nodes_dict[curr['loc']]["neighbors"] # list of possivble locations an agent can move
        possible_positions.add(curr['loc']) # adding the possibilit to wait


        for neighbor in possible_positions:

            path = get_path(curr, agent) # the path of the agent that will be used for replanning is composed
            back = 0
            for k in range(len(path)-1):
                if path [k][0] == neighbor and path[k][0 != path[k+1][0]] and abs(
                        path[k][1] - (round(curr['timestep'] + find_time_step_size(speed),2)) < 4):
                    back = 1


            if is_constrained(curr['loc'], neighbor, round(curr['timestep'] + find_time_step_size(speed),2), constraint_table,speed) or back == 1:
                continue

            child = {'loc': neighbor,
                    'g_val': curr['g_val'] + 0.5, # this will stay the same since the cost is still the same.
                    'h_val': heuristics[neighbor][goal_node_id],
                    'parent': curr,
                    'timestep': round(curr['timestep'] + find_time_step_size(speed),2)} # The time step is determined by the speed of the aircraft
            
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)

    print("No path found, "+str(len(closed_list))+" nodes visited")
    return False, [] # Failed to find solutions
    