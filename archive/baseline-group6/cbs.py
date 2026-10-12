"""
CBS planner

This code is part of the assignment of the course AE4422-20 Agent-based Modelling and Simulation in Air Transport (2021/22)

This is the work of group 6

Student names and student numbers: Florina Sirghi   (4648579)
                                   Vincent van Gorp (4777697)
"""

import heapq
import copy
from single_agent_planner import get_sum_of_cost, complex_single_agent_astar

def detect_collision(path1, path2, agent1, agent2, t):
    """
        This detects the first collision happening between two agents.
        INPUT:
            - path1 = path of the first agent (list of elements of the form: (location, timestep, ac,id))
            - path2 = path of the second agent (list of elements of the form: (location, timestep, ac,id))
            - agent1 = id of the first agent
            - agent2 = id of the second agent
            - t = current timestep
        RETURNS:
            - the first identified collision, in the form of a dictionary {'a1': agent, 'a2':agent, 'loc':location, 'timestep':t}
    """

    # Vertex collisions
    for i in range(len(path1)):
        for j in range(len(path2)):
            # determining when a vertex collision is detected
            if path1[i][1]==path2[j][1] and path1[i][0]==path2[j][0]:
                return {'a1': agent1, 'a2': agent2, 'loc': [path1[i][0]], 'timestep': path1[i][1]}

    # Edge collisions
            if path1[i][1]>=1 and path2[j][1]>=1:
                if path1[i][1]==path2[j][1] and path1[i-1][0] == path2[j][0] and path1[i][0] == path2[j-1][0]:
                    # determining when an edge collision is detected
                    return {'a1': agent1,'a2': agent2, 'loc': [path1[i-1][0], path1[i][0]], 'timestep': path1[i][1]}


    # Two aircraft using the same runway
    if t==path1[-1][1]-0.5:  # if the aircraft are one node away from reaching the runway
        if path1[-1][1]==path2[-1][1] and ((path1[-1][0]==1 or path1[-1][0]==2) and (path2[-1][0]==1 or path2[-1][0]==2)):
            # Determining when two aircraft intend to use the same runway, at the same time
            if agent1<agent2: # we choose which aircraft should get a constraint by comparing their ids
                return {'a1': agent2, 'a2': agent2, 'loc': [path2[-1][0]], 'timestep': path2[-1][1]}
            else:
                return {'a1': agent1, 'a2': agent1, 'loc': [path1[-1][0]], 'timestep': path1[-1][1]}


    # Dead lock at the gates

    # checking whether for the two agents whose paths we are considering, one agent is an arriving aircraft and it is
    # 2 nodes away from its goal gate, and the other agent is a departing aircraft that was just spawned at the same gate
    if len(path1)==2 and len(path2)>2 and path1[-1][0]==path2[0][0] and (path1[-1][0]==97 or path1[-1][0]==34 or path1[-1][0]==35 or path1[-1][0]==36 or path1[-1][0]==98) and (path2[0][0]==1 or path2[0][0]==2):
        if t==path1[-3][1]:  # checking if the time step is equal to that when the arriving aircraft is at two nodes away from the gate
            return {'a1': agent2, 'a2': agent2, 'loc': [path2[1][0]], 'timestep': path1[-3][1]}

    elif len(path2)==2 and len(path1)>2 and path2[-1][0]==path1[0][0] and (path2[-1][0]==97 or path2[-1][0]==34 or path2[-1][0]==35 or path2[-1][0]==36 or path2[-1][0]==98) and (path1[0][0]==1 or path1[0][0]==2):
        if t==path2[-3][1]:
            return {'a1': agent1, 'a2': agent1, 'loc': [path1[1][0]], 'timestep': path2[-3][1]}


    return None

def detect_collisions(paths,t):
    """
        This function detects all the first collisions happening between all pairs of agents that are taxiing.
        INPUT:
            - paths = list of paths for all the agents
            - t = current timestep
        RETURNS:
            - first_collisions = list of dictionaries representing the first collisions between all pairs of agents
    """
    first_collisions = []  # Initialising list of first collisions

    for agent1 in range(len(paths)):
        for agent2 in range(agent1, len(paths)):
            if agent1 != agent2 and len(paths[agent1])>0 and len(paths[agent2])>0:
                if detect_collision(paths[agent1], paths[agent2], paths[agent1][0][2], paths[agent2][0][2],t):
                    # Appending the collisions that were identified;
                    # paths[agent1][0][2] and paths[agent2][0][2] were used to extract the correct id of the agents
                    # that are involved in the collisions, as the elements in teh path of each agent are of the form:
                    # (node, timestep, ac.id)
                    first_collisions.append(detect_collision(paths[agent1], paths[agent2], paths[agent1][0][2], paths[agent2][0][2],t))

    return first_collisions

def standard_splitting(collision):
    """
        This function transforms a collision into a set of corresponding constraints
        INPUT:
            - collision = dictionary in the form: {'a1': agent, 'a2':agent, 'loc':location, 'timestep':t}
        RETURNS:
            - return_list = list of dictionaries representing the constraints formulated following the collision that
                            was sent as input
    """

    return_list = []
    agent1 = collision.get('a1')
    agent2 = collision.get('a2')
    timestep = collision.get('timestep')

    # Vertex collision
    if len(collision.get('loc')) == 1:  # if the location is only one element long, then we are dealing with a vertex collision
        location = collision['loc']
        return_list.append({'agent': agent1, 'loc': location, 'timestep': timestep})
        return_list.append({'agent': agent2, 'loc': location, 'timestep': timestep})

    # Edge collision
    if len(collision.get('loc')) == 2:  # if the location is two elements long, then we are dealing with an edge collision
        location = collision['loc']

        return_list.append({'agent': agent1, 'loc': location, 'timestep': timestep})
        return_list.append({'agent': agent2, 'loc': [location[-1], location[0]], 'timestep': timestep})

    return return_list



class CBSSolver(object):
    """The high-level search of CBS."""

    def __init__(self, nodes_dict, edges_dict, aircraft_lst, heuristics, t, starts, goals, current_location_aircraft):

        self.nodes_dict = nodes_dict
        self.t = t
        self.aircraft_lst = aircraft_lst
        self.heuristics = heuristics
        self.edges_dict = edges_dict
        self.startslst = starts
        self.goalslst   = goals
        self.current_location_aircraft = current_location_aircraft

        self.num_of_generated = 0
        self.num_of_expanded = 0
        self.CPU_time = 0

        self.open_list = []


    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_of_generated, node))
        #print("Generate node {}".format(self.num_of_generated))
        self.num_of_generated += 1

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        #print("Expand node {}".format(id))
        self.num_of_expanded += 1
        return node

    def find_solution(self, constraints, existent_paths, current_location_aircraft, collisions, time_start, original_path_lst):
        """
            This function performs CBS planning.
            INPUT:
                - constraints = list of constraints
                - existent_paths = list that stores the paths that have been previously planned for the aircraft, from their
                                   starting position to their goal position
                - current_location_aircraft = a list that contains all the current locations of all taxiing aircraft, with
                                              elements in the form (position, timestep, ac.id)
                - collisions = list of dictionaries that stores all collisions that have been encountered so far
                - time_start = time of the start of the current run
                - original_path_lst = list that stores the taxiing time originally planned for all aircraft
            RETURNS:
                - new_paths = a list of paths of all aircraft, planned from their starting position until their goal position,
                              without any collisions
        """

        paths_to_go = [[] for i in range(len(existent_paths))]  # Initialising a list of the path that still needs to be
                                                                # completd by each agent, from next timestep onwards
        # Initialising the root node,
        root = {'cost': 0,
                'constraints': [],
                'paths':  [],
                'collisions': []}

        for ac in self.aircraft_lst:
            if ac.spawntime == self.t:  # spawning the aircraft if the timestep is equal to the spawntime of an aircraft
                ac.status = "taxiing"
                ac.position = self.nodes_dict[ac.start]["xy_pos"]
                agent = ac.id
                start_node = ac.start  # node from which planning should be done
                goal_node = ac.goal  # goal node, towards which planning should be done
                path_so_far = []  # initialising a list to later store the path completed by the agent until and including the
                                  # current timestep; this list will be used to keep track of the entire path followed by the agent
                ac.plan_independent(self.nodes_dict, self.edges_dict, self.heuristics, self.t, ac) # first, plan the paths without
                                                                                                   # any constraints
                original_path = ac.path_to_goal  # store the path_to_goal calculated with no constraints
                original_path_lst.append((len(original_path) + 1) / 2 - 0.5)  # use it to compute the initially planned taxiing time
                                                                              # for the aircraft, and append that to the original path list

                # Storing in "path" the initial path of the aircraft, which is computed without any constraints (as root['constraints'] is
                # still empty at this point in time
                success, path = complex_single_agent_astar(self.nodes_dict, start_node, goal_node, self.heuristics, self.t, agent, root['constraints'], path_so_far)

                if success:
                    ac.path_to_goal = path[1:]  # Update the path_to_goal for the aircraft
                    next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                    ac.from_to = [path[0][0], next_node_id]
                    ac.last_node = path[0][0]  # Initialising the last node of the aircraft with the starting position
                    paths_to_go[agent] = path  # Storing the computed path for the aircraft in the corresponding
                                               # paths_to_go element
                    existent_paths[agent] = path  # Storing the path of teh aircraft in the corresponding element in the
                                                  # existent_paths list, which will be used to keep track of the entire
                                                  # path followed by the agent
                    root['paths']  = paths_to_go  # Storing the "paths to go" in the root, as these are the paths that could
                                                  # still contain collisions and, therefore, may need to be replanned

                else:
                    raise Exception("No solution found for", ac.id)

                # Check the path
                if path[0][1] != self.t:
                    raise Exception("Something is wrong with the timing of the path planning")

                if path is None:
                    raise BaseException('No solutions')


        for ac in self.aircraft_lst:
            if ac.status == 'taxiing' and ac.spawntime!=self.t:
                paths_to_go[ac.id]= [(ac.from_to[0], ac.from_to[1], ac.id)] + ac.path_to_goal  # For all aircraft that are actively
                # taxiing, but have been been spawned in the past, we add their paths_to_go, by first manually adding the element
                # corresponding to the starting position (as this is not included in path_to_goal)


        root['paths'] = paths_to_go  # Then, we store all the paths_to_go in the root node
        root['cost'] = get_sum_of_cost(root['paths'])  # We compute the cost of the current solution/set of paths
        root['collisions'] = detect_collisions(root['paths'],self.t)  # Then, we store the collisons that take place in the paths
        collisions.append(root['collisions'])
        self.push_node(root)  # and we add the root node to the list

        while len(self.open_list) > 0:  # while we still have nodes in the open list

            P = self.pop_node()  # we extract the node with the smallest cost from the list

            if len(P['collisions']) == 0  or P['collisions'] is None: # if we have found a list of paths without collisons, so a solution

                current_paths = P['paths']  # these are the "paths to go"

                new_paths=[[] for i in range(len(existent_paths))]  # initialising a list of new paths, in which we will store the current
                                                                    # solutions for each agent

                # First, we will store all the elements from the past which are found in the "existent_paths" list
                for i in range(len(existent_paths)):
                    for element in existent_paths[i]:
                        if element[1]<=self.t:  # we also include the element at the current timestep
                            new_paths[i].append(element)



                for ac in self.aircraft_lst:
                    if ac.status == "taxiing":  # for all taxiing aircraft, we update their path_to_goal and we prepare the next move

                        ac.path_to_goal = current_paths[ac.id][1:]
                        next_node_id = ac.path_to_goal[0][0]  # next node is first node in path_to_goal
                        ac.from_to = [current_paths[ac.id][0][0], next_node_id]
                        ac.last_node = [current_paths[ac.id][0][0]]

                        # We also append to the new_paths all the elements in the "current_paths"
                        # More precisely, we are now adding the future elements in the paths of the aircraft, after the
                        # path elements from the past and the element representing the present, which have already been
                        # included before, to obtain the complete planned path for the agent at this point in time
                        new_paths[ac.id].append(current_paths[ac.id][0])
                        for i in range(1,len(current_paths[ac.id])):
                             new_paths[ac.id].append(current_paths[ac.id][i])

                return new_paths

            collision = P['collisions'][0]  # getting a collision from P, namely the first one
            collisions.append(collision)

            constraints_new = standard_splitting(collision)  # generating the constraints, based on the collision that was identified

            for constraint in constraints_new:  # for each constraint
                constraints.append(constraint)

                # Initialising node Q
                Q = {'cost': 0,
                    'constraints': [],
                    'paths': [],
                    'collisions': []}

                Q['constraints'] = copy.deepcopy(P['constraints'])  # we make a copy of the constraints from the parent node P
                                                                    # copy.deepcopy had to be used, as otherwise the two
                                                                    # dictionaries would become linked to each another
                if constraint not in Q['constraints']:
                    Q['constraints'].append(constraint)  # we only append a constraint if it was not already present in P

                Q['paths'] =  copy.deepcopy(P['paths'])  # we store the paths from P into the Q
                agent = constraint['agent']

                # Computing the new starting location for the agent from which the rest of its path should be replanned
                # This new starting location is actually the current location of the aircraft
                start = 0
                for current_location in current_location_aircraft:
                    if current_location[1]==self.t and current_location[2]==agent:
                        start = current_location[0]

                for ac in self.aircraft_lst:
                    if agent == ac.id:
                        if start!=0:
                            path_so_far = []
                            for element in existent_paths[agent]:
                                if element[1] < self.t:
                                    path_so_far.append(element) # creating a list with the path that was completed so far by the agent,
                                                                # to use it for replanning purposes
                            success, path = complex_single_agent_astar(self.nodes_dict, start, self.goalslst[agent], self.heuristics, self.t, agent, Q['constraints'], path_so_far)


                            if path is not None and len(path)>0: # if we succesfully obtain a new path after replanning
                                path_so_far = [] # we are once again generating the path_so_far for the agent
                                for element in existent_paths[agent]:
                                    if element[1]<self.t:
                                        path_so_far.append(element)

                                existent_paths[agent] = path_so_far + path  # and then we are updating the existent_paths list with the
                                                                            # path performed by the agent so far, to which we add the path
                                                                            # that is planned for the aircraft to complete from this
                                                                            # timestep onwards, until it reaches its goal
                                paths_to_go[agent] = path  # we are storing the path that was replanned and still needs to be completed
                                                           # by the aircraft in its corresponding element in paths_to_go

                                Q['paths'][agent] = path  # we are updating the path of the agent in Q with the newly replanned path
                                Q['collisions'] = detect_collisions(Q['paths'],self.t)  # then we compute the collisions for the new set of paths
                                Q['cost'] = get_sum_of_cost(Q['paths'])  # we calculate the new cost for the new set of paths

                                self.push_node(Q) # then, we add the node Q to the list of nodes

        return "No solutions"





