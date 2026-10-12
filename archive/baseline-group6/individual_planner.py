"""
Individual/distributed planner

This code is part of the assignment of the course AE4422-20 Agent-based Modelling and Simulation in Air Transport (2021/22)

This is the work of group 6

Student names and student numbers: Florina Sirghi   (4648579)
                                   Vincent van Gorp (4777697)
"""

from single_agent_planner import complex_single_agent_astar_1
from cbs import detect_collisions, standard_splitting
import time as timer


class Individual_Planner(object):
    """
    Individual Planner.
    INPUT:
        - nodes_dict = [dict] dictionary with nodes and node properties
        - edges_dict = [dict] dictionary with edges
        - heuristics = [dict] dict with shortest path distance between nodes.
                       Dictionary in a dictionary. Key of first dict is fromnode
                       and key in second dict is tonode.
        - Aircraft_lst = [list] List containing all properties for every aircraft
        - t = current time step
        - constraints = [list] List containing all constraints for all agents
    RETURNS:
    Suboptimal_solution_for_collision = True/ False.
        This will check whether a collision was solved by the algorithm in a suboptimal way
        (for example by making the aircraft wait for too long at a certain position or even,
        very rarely, let a collision happen if it doesn't find another solution)

    """

    def __init__(self, nodes_dict, edges_dict, heuristics, aircraft_lst, t, constraints):
        self.nodes_dict = nodes_dict
        self.edges_dict = edges_dict
        self.heuristics = heuristics
        self.t = t
        self.aircraft_lst = aircraft_lst
        self.constraints = constraints

    def planner(self, existent_paths, time_start):
        """
        Planner used for individual planning of aircraft agents
        Input:
            - existant_paths = [List] List of all paths all aircraft have travelled
                                up until the current time step
            - time_start = start of the simulation. Is used in order to break the
                            algorithm in case of a deadlock situation.
        Returns:
            Suboptimal_solution_for_collision = True/ False.
            This will check whether a collision was solved by the algorithm in a suboptimal way
            (for example by making the aircraft wait for too long at a certain position or even,
            very rarely, let a collision happen if it doesn't find another solution)
        """

        # Lists
        newconstraints_lst = []  # List of the new constraints that should be imposed on an airraft
        # for every 0.5 seconds
        current_location_aircraft = []  # List containing the current locations of all aircraft
        suboptimal_solution_for_collision = "False"

        # Global radar
        if self.t % 0.5 == 0:  # Only check all aircraft positions at 0.5 seconds, when they are on a node
            for ac in self.aircraft_lst:
                if ac.status == "taxiing":
                    id = ac.id
                    position = ac.position
                    for i in range(1, 109):  # Check for all nodes if an aircraft is on this node
                        if self.nodes_dict[i]['xy_pos'] == position:
                            current_node = self.nodes_dict[i]['id']
                    current_location_aircraft.append((current_node, self.t, id))  # Append the current node
                    # timestep and flight id
                    existent_paths[ac.id].append((current_node, self.t, id))  # Add this to existent path
                    # In order to check the path
                    # an aircraft travelled

                    # Replan the aircraft if it is close to the departure runway
                    # Its goal is to departure and it will take the shortest route to the runawy
                    if len(ac.path_to_goal) == 6 and (ac.goal == 1.0 or ac.goal == 2.0):
                        if ac.heading == 0:  # If an aircraft is moving downwards, its goal will become 2.0
                            ac.goal = 2.0
                            xy_location = ac.position
                            for i in range(1, 109):
                                if self.nodes_dict[i]['xy_pos'] == xy_location:
                                    curnode = self.nodes_dict[i]['id']
                            path_so_far = []

                            for element in existent_paths[ac.id]:
                                if element[1] < self.t:
                                    path_so_far.append(element)
                            success, path = complex_single_agent_astar_1(self.nodes_dict, curnode, ac.goal,
                                                                         self.heuristics, self.t, ac.id,
                                                                         newconstraints_lst, path_so_far)
                            if success:
                                ac.path_to_goal = path[1:]
                                next_node_id = ac.path_to_goal[0][0]
                                ac.from_to = [path[0][0], next_node_id]

                        if ac.heading == 180:  # If an aircraft is moving upwards, its goal will become 1.0
                            ac.goal = 1.0
                            xy_location = ac.position
                            for i in range(1, 109):
                                if self.nodes_dict[i]['xy_pos'] == xy_location:
                                    curnode = self.nodes_dict[i]['id']
                            path_so_far = []
                            for element in existent_paths[ac.id]:
                                if element[1] < self.t:
                                    path_so_far.append(element)
                            success, path = complex_single_agent_astar_1(self.nodes_dict, curnode, ac.goal,
                                                                         self.heuristics, self.t, ac.id,
                                                                         newconstraints_lst, path_so_far)
                            if success:
                                ac.path_to_goal = path[1:]
                                next_node_id = ac.path_to_goal[0][0]
                                ac.from_to = [path[0][0], next_node_id]

            # Implementation of local radar

            radar_dict = dict()  # This list will contain per 0.5 seconds,
            # which aircraft can see other aircraft.
            for ac in self.aircraft_lst:
                if ac.status == "taxiing":
                    radar_list = []
                    x_pos = ac.position[0]
                    x_pos_min = x_pos - 1.5  # Create left bottom corner position for box radar view
                    y_pos = ac.position[1]
                    y_pos_min = y_pos - 1.5  # Create left bottom corner position for box radar view

                    for a in range(1, 109):  # Check where the current aircraft is
                        if self.nodes_dict[a]['xy_pos'] == ac.position:
                            current_node = self.nodes_dict[a]['id']

                    # This is the radar for a square view
                    for x in range(7):  # Range 7 is needed as a the box has a 'radius' of 3 nodes
                        for y in range(7):  # Range 7 is needed as a the box has a 'radius' of 3 nodes
                            xy_location = (x_pos_min + 0.5 * x, y_pos_min + 0.5 * y)
                            for i in range(1, 109):  # Check if a node exist
                                if self.nodes_dict[i]['xy_pos'] == xy_location:
                                    node_local = self.nodes_dict[i]['id']
                                    radar_list.append((node_local, self.t))

                    # Check if an aircraft is one of the nodes checked in the radar view
                    for i in range(len(current_location_aircraft)):
                        for j in range(len(radar_list)):
                            if current_location_aircraft[i][:2] == radar_list[j]:
                                if current_location_aircraft[i][2] != ac.id:
                                    if ac.id in radar_dict:
                                        radar_dict[ac.id].append((current_location_aircraft[i]))
                                    else:
                                        radar_dict[ac.id] = [(current_location_aircraft[i])]
                                        # Create dictionary with all aircraft that are in the range of the current aircraft

            # If an aircraft sees another aircraft, check for possible collisions
            if len(radar_dict) != 0:
                replanning_list = list(
                    radar_dict.keys())  # List of all aircraft that need see an aircraft. They could have possible collisions

                # Make a possible replanning list of which aircraft sees which aircraft
                for i in range(len(radar_dict)):
                    if i in replanning_list:
                        replanning_list_local = [i]
                        aircraft_radarlist = radar_dict.get(i)
                        for j in range(len(aircraft_radarlist)):
                            replanning_list_local.append(aircraft_radarlist[j][2])
                        paths = []
                        for m in range(len(replanning_list_local)):
                            for aircraft in self.aircraft_lst:
                                if aircraft.id == replanning_list_local[m]:
                                    paths.append(
                                        aircraft.path_to_goal)  # Append path to paths list in order to check for possible
                                    # collisions

                        # Check for collisions using detect_collision from CBS
                        collision = detect_collisions(paths, self.t)

                        # If a collision will occur start the replanning process
                        if len(collision) != 0:
                            counter = 0  # Counter will break the loop if a deadlock situation arises
                            current_time = timer.time()  # Needed to break the loop if a deadlock situation arises

                            while len(collision) != 0:  # If a collision occurs start replanning until no collision
                                # occur anymore
                                counter += 1
                                collision = []
                                paths2 = []  # List containing the old and new paths, a new path will overwrite the old
                                # path

                                # Check if a collision occurs or is still occuring
                                for q in range(len(replanning_list_local)):
                                    for aircraft1 in self.aircraft_lst:
                                        if aircraft1.id == replanning_list_local[q]:
                                            paths2.append(aircraft1.path_to_goal)
                                collision = detect_collisions(paths2, self.t)

                                # If a collision occurs for the old and new paths
                                if len(collision) != 0:
                                    single_collision = collision[0]
                                    return_list = standard_splitting(single_collision)
                                    agent1 = single_collision.get('a1')
                                    agent2 = single_collision.get('a2')

                                    if len(single_collision.get('loc')) == 2:  # Append constraint for edge collision,
                                        # both aircraft get the constraint
                                        newconstraints_lst.append(return_list[0])
                                        newconstraints_lst.append(return_list[1])
                                        agentx = return_list[0].get('agent')
                                        agenty = return_list[1].get('agent')
                                        for ac5 in self.aircraft_lst:
                                            if ac5.id == agentx:  # Replan for the first agent
                                                xy_location = ac5.position
                                                for q in range(1, 109):
                                                    if self.nodes_dict[q]['xy_pos'] == xy_location:
                                                        curnode = self.nodes_dict[q]['id']

                                                path_so_far = []
                                                for element in existent_paths[ac5.id]:
                                                    if element[1] < self.t:
                                                        path_so_far.append(element)

                                                success, path3 = complex_single_agent_astar_1(self.nodes_dict, curnode,
                                                                                              ac5.goal, self.heuristics,
                                                                                              self.t, agentx,
                                                                                              newconstraints_lst,
                                                                                              path_so_far)
                                                if success:
                                                    ac5.path_to_goal = path3[1:]
                                                    next_node_id = ac5.path_to_goal[0][0]
                                                    ac5.from_to = [path3[0][0], next_node_id]

                                            if ac5.id == agenty:  # Replan for the second agent
                                                xy_location = ac5.position
                                                for q in range(1, 109):
                                                    if self.nodes_dict[q]['xy_pos'] == xy_location:
                                                        curnode = self.nodes_dict[q]['id']

                                                path_so_far = []
                                                for element in existent_paths[ac5.id]:
                                                    if element[1] < self.t:
                                                        path_so_far.append(element)
                                                success, path3 = complex_single_agent_astar_1(self.nodes_dict, curnode,
                                                                                              ac5.goal, self.heuristics,
                                                                                              self.t, agenty,
                                                                                              newconstraints_lst,
                                                                                              path_so_far)
                                                if success:
                                                    ac5.path_to_goal = path3[1:]
                                                    next_node_id = ac5.path_to_goal[0][0]
                                                    ac5.from_to = [path3[0][0], next_node_id]


                                    # Determine which aircraft will get the constraint based on the time it needs to reach its goal,
                                    # otherwise the aircraft that spawned first (lowest ID) will get preference
                                    else:
                                        for ac2 in self.aircraft_lst:
                                            if agent1 == ac2.id:
                                                path1 = ac2.path_to_goal
                                                time_to_reach_goal1 = len(path1) / 2

                                            if agent2 == ac2.id:
                                                path2 = ac2.path_to_goal
                                                time_to_reach_goal2 = len(path2) / 2

                                        if time_to_reach_goal1 > time_to_reach_goal2:  # Highest time to reach goal gets preference
                                            constraint = return_list[1]
                                            newconstraints_lst.append(constraint)
                                            agent123 = constraint.get('agent')

                                            for ac3 in self.aircraft_lst:
                                                if ac3.id == agent123:
                                                    xy_location = ac3.position
                                                    for q in range(1, 109):
                                                        if self.nodes_dict[q]['xy_pos'] == xy_location:
                                                            curnode = self.nodes_dict[q]['id']
                                                    path_so_far = []
                                                    for element in existent_paths[ac3.id]:
                                                        if element[1] < self.t:
                                                            path_so_far.append(element)
                                                    success, path1 = complex_single_agent_astar_1(self.nodes_dict,
                                                                                                  curnode,
                                                                                                  ac3.goal,
                                                                                                  self.heuristics,
                                                                                                  self.t,
                                                                                                  agent123,
                                                                                                  newconstraints_lst,
                                                                                                  path_so_far)
                                                    if success:
                                                        ac3.path_to_goal = path1[1:]
                                                        next_node_id = ac3.path_to_goal[0][0]
                                                        ac3.from_to = [path1[0][0], next_node_id]

                                        elif time_to_reach_goal2 >= time_to_reach_goal1:  # Highest time to reach goal gets preference
                                            constraint = return_list[0]
                                            newconstraints_lst.append(constraint)
                                            agent2 = constraint.get('agent')
                                            for ac4 in self.aircraft_lst:
                                                if ac4.id == agent2:
                                                    xy_location = ac4.position
                                                    for q in range(1, 109):
                                                        if self.nodes_dict[q]['xy_pos'] == xy_location:
                                                            curnode = self.nodes_dict[q]['id']
                                                    path_so_far = []
                                                    for element in existent_paths[ac4.id]:
                                                        if element[1] < self.t:
                                                            path_so_far.append(element)
                                                    success, path2 = complex_single_agent_astar_1(self.nodes_dict,
                                                                                                  curnode,
                                                                                                  ac4.goal,
                                                                                                  self.heuristics,
                                                                                                  self.t,
                                                                                                  agent2,
                                                                                                  newconstraints_lst,
                                                                                                  path_so_far)
                                                    if success:
                                                        ac4.path_to_goal = path2[1:]
                                                        next_node_id = ac4.path_to_goal[0][0]
                                                        ac4.from_to = [path2[0][0], next_node_id]

                                if counter > 30:  # Break if a deadlock occurs
                                    suboptimal_solution_for_collision = "True"
                                    break

        if suboptimal_solution_for_collision == "True":
            return ("True")
        else:
            return ("False")
