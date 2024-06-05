"""
This file contains single agent planner functions  that can be used by other planners.
Consider functions in this file as supporting functions.
"""

import heapq
import networkx as nx
import numpy as np

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


def build_constraint_table(constraints, agent):
    """
    Generates the table of constraints for an agent.
    INPUT:
        - constraints = list of all the constraints that have been generated
        - agent = the id of the agent for which we want to store the constraints
    RETURNS:
        - return_table = list of constraints for the agent, in the form of tuples containing the time step and the
                        location of the constraint
    """
    return_table = []

    for i in constraints:
        # Check whether the constraint applies to the agent. If not, the for loop continues with the next entry in the
        # array consisting of the constraints (dictionaries).
        if i['agent'] != agent:
            continue

        time_step = i['timestep']

        # Preparation for the edge constraints. Returns a tuple within tuple ((location 1), (location N))
        if len(i['loc'])!=1:
            location_lst = []
            for y in range(len(i['loc'])):
                location = i['loc'][y]
                location_lst.append(location)
            location_tup = location_lst

        # Preparation for the vertex constraints
        else:
            location_tup = [i['loc'][0]]

        # Appends [time_step, location] to the array for agent N. Location may be a tuple consisting of multiple entries
        # or just a single entry.
        return_table.append([time_step, location_tup])

    return_table = np.array(return_table)

    if len(return_table) != 0:
        return_table = return_table[return_table[:, 0].argsort()]  # this gives the sorted constraint table

    return return_table

def is_constrained(curr_loc, next_loc, next_time, constraint_table):
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
            if constraint_location == next_loc and next_time == time_step:  # if there exists a vertex constraint for the
                                                                            # next location and next timestep
                switch = True  # then the movement should be constrained
        # Checking edge constraints.
        if len(i[1])!=1:  # if the length of the location of the constraint is different than 1 (namely equal to 2),
                          # then we are dealing with an edge constraint
            time_step = i[0]
            if time_step != next_time:
                continue
            constraint_location1 = i[1][0]
            if curr_loc != constraint_location1:
                continue
            constraint_location2 = i[1][1]
            if next_loc == constraint_location2:  # if the aircraft intends to move to a position that is constrained
                                                  # due to an edge collision at the next timestep, the next timestep
                switch = True  # then the movement should be constrained
        if switch == True:
            break
    return switch

def simple_single_agent_astar(nodes_dict, from_node, goal_node, heuristics, time_start, agent, constraints):
    """
    #### This version of the a_star algorithm is used for Prioritized planning ####
    Single agent A* search. Time start can only be the time that an agent is at a node.
    INPUT:
        - nodes_dict = [dict] dictionary with nodes and node properties
        - from_node = [int] node_id of node from which planning is done
        - goal_node = [int] node_id of node to which planning is done
        - heuristics = [dict] dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first
                              dict is from node and key in second dict is to node.
        - time_start = [float] planning start time.
        - agent = the id of the agent for which the planning needs to be (re)done
        - constraints = list of constraints
    RETURNS:
        - success = True/False. True if path is found and False is no path is found
        - path = list of tuples with (loc, timestep, agent) pairs -> example [(37, 1.5, 1), (101, 2, 1)]. Empty list if success == False.
    """

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
            return True, get_path(curr,agent)


        possible_positions =  nodes_dict[curr['loc']]["neighbors"]  # Making a list of all possible locations to which the
                                                                    # agent can move (its neighbors)
        possible_positions.add(curr['loc'])  # Adding the possibility for the agent to wait in its current location

        for neighbor in possible_positions:

            # Check whether the new child nodes violates any constraints set by other agents. We check whether the
            # constraint is true, and if so, we discard this child node and move on to the next node using 'continue'.

            path = get_path(curr,agent)
            back = 0
            for k in range(len(path) - 1):
                if path[k][0] == neighbor and path[k][0] != path[k + 1][0] and abs(
                        path[k][1] - (curr['timestep']+0.5)) < 4:  # Ensuring that the aircraft is not moving backwards
                    back = 1

            # If the movement is constrained by a constraint or if it results in a backward movement, then we continue
            if is_constrained(curr['loc'], neighbor, curr['timestep']+0.5, constraint_table) or back == 1:
                continue

            # Generating the child node
            child = {'loc': neighbor,
                     'g_val': curr['g_val'] + 0.5,
                     'h_val': heuristics[neighbor][goal_node_id],
                     'parent': curr,
                     'timestep': curr['timestep'] + 0.5}

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


def complex_single_agent_astar(nodes_dict, from_node, goal_node, heuristics, time_start, agent, constraints, path_so_far):
    """
    #### This version of the a_star algorithm is used for CBS planning ####

    Single agent A* search. Time start can only be the time that an agent is at a node.

    INPUT:
        - nodes_dict = [dict] dictionary with nodes and node properties
        - from_node = [int] node_id of node from which planning is done
        - goal_node = [int] node_id of node to which planning is done
        - heuristics = [dict] dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
        - time_start = [float] planning start time.
        - agent = the id of the agent for which the planning needs to be (re)done
        - constraints = list of constraints
        - path_so_far = part of the path of the agent that has been completed so far (all past elements and the current element of the path)
    RETURNS:
        - success = True/False. True if path is found and False is no path is found
        - path = list of tuples with (loc, timestep, agent) pairs -> example [(37, 1.5, 1), (101, 2, 1)]. Empty list if success == False.
    """


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
            return True, get_path(curr,agent)


        possible_positions =  nodes_dict[curr['loc']]["neighbors"]  # Making a list of all possible locations to which the
                                                                    # agent can move (its neighbors)
        possible_positions.add(curr['loc'])  # Adding the possibility for the agent to wait in its current location

        for neighbor in possible_positions:

            # Check whether the new child nodes violates any constraints set by other agents. We check whether the
            # constraint is true, and if so, we discard this child node and move on to the next node using 'continue'.

            path = path_so_far + get_path(curr,agent)  # the path of the agent that will be used for replanning is composed
                                                       # of the path so far and the part of the path that can be retrieved
                                                       # from get_path (starting from the parent of the current node)


            back = 0  # parameter is 1 if the agent moves backwards and 0 if it does not
            for k in range(len(path)-1):
                if path[k][1]==curr['timestep']-0.5:   # if at the previous timestep
                    if path[k][0]==neighbor and path[k+1][0]!=neighbor:  # the agent had the same position as the neighbour,
                                                                         # but then it moved from that position
                        back = 1  # then it means that it intends to move backwards (back to its previous position, at the previous timestep)


                if path[k][1]==curr['timestep']-1 and path[k+1][0]==curr['loc']:
                    if path[k][0] == neighbor:  # the same as above, but for two timesteps behind (ensuring that the aircraft doesn't
                                                # wait for two timesteps at a location and then moves backwards)
                        back = 1


            if back == 1:
                continue

            # checking that the aircraft doesn't wait for an arbitrary amount of timesteps in a loction and only then
            # wants to move backwards to its previous location

            for k in range(len(path) - 1):
                if path[k][0]==neighbor and path[k][1]<curr['timestep']:
                    time_1=path[k][1]
                    time_2=curr['timestep']
                    back = 1
                    position = path[k+1][0]
                    i=2
                    time_step = time_1 + 0.5
                    while time_step < time_2:
                        if path[k+i][0]!=position:  # if the aircraft is just waiting and doesn't actually move for
                                                    # multiple tiemsteps, but also doesn't have the intention to move backwards
                            back=0
                        i=i+1
                        time_step = time_step+0.5

            if back==1:
                continue

            # from here onwards, we do the same as we did for the simple single agent a_star
            if is_constrained(curr['loc'], neighbor, curr['timestep']+0.5, constraint_table) or back == 1:
                continue

            child = {'loc': neighbor,
                     'g_val': curr['g_val'] + 0.5,
                     'h_val': heuristics[neighbor][goal_node_id],
                     'parent': curr,
                     'timestep': curr['timestep'] + 0.5}

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


def complex_single_agent_astar_1(nodes_dict, from_node, goal_node, heuristics, time_start, agent, constraints, path_so_far):
    """
    #### This version of the a_star algorithm is used for Individual planning ####

    Single agent A* search. Time start can only be the time that an agent is at a node.

    INPUT:
        - nodes_dict = [dict] dictionary with nodes and node properties
        - from_node = [int] node_id of node from which planning is done
        - goal_node = [int] node_id of node to which planning is done
        - heuristics = [dict] dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
        - time_start = [float] planning start time.
        - agent = the id of the agent for which the planning needs to be (re)done
        - constraints = list of constraints
        - path_so_far = part of the path of the agent that has been completed so far (all past elements and the current element of the path)
    RETURNS:
        - success = True/False. True if path is found and False is no path is found
        - path = list of tuples with (loc, timestep, agent) pairs -> example [(37, 1.5, 1), (101, 2, 1)]. Empty list if success == False.
        """

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
            return True, get_path(curr,agent)


        possible_positions =  nodes_dict[curr['loc']]["neighbors"]
        possible_positions.add(curr['loc'])

        for neighbor in possible_positions:

            # Check whether the new child nodes violates any constraints set by other agents. We check whether the
            # constraint is true, and if so, we discard this child node and move on to the next node using 'continue'.

            path = path_so_far + get_path(curr,agent)  # we once again need the path_so_far in combination with the part of
                                                       # path that can be retrieved from get_path (starting from the parent
                                                       # of the current node)that is provided


            back = 0
            for k in range(len(path) - 1):
                if path[k][0] == neighbor and path[k][0] != path[k + 1][0]:  # we check if the aircraft is not moving backwards
                    back = 1

            if is_constrained(curr['loc'], neighbor, curr['timestep']+0.5, constraint_table) or back == 1:
                continue

            child = {'loc': neighbor,
                     'g_val': curr['g_val'] + 0.5,
                     'h_val': heuristics[neighbor][goal_node_id],
                     'parent': curr,
                     'timestep': curr['timestep'] + 0.5}

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
    return path


