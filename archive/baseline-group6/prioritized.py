"""
Prioritized planner

This code is part of the assignment of the course AE4422-20 Agent-based Modelling and Simulation in Air Transport (2021/22)

This is the work of group 6

Student names and student numbers: Florina Sirghi   (4648579)
                                   Vincent van Gorp (4777697)
"""

from single_agent_planner import simple_single_agent_astar

def run_prioritized_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t, constraints, timesteps, result, original_path_lst):
    """
    This function performs prioritized planning.
    INPUT:
        - aircraft_lst = list of all Aircraft objects (all agents)
        - nodes_dict = [dict] dictionary with nodes and node properties
        - edge_dict = [dict] edges_dict with current edge weights
        - heuristics = [dict] dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first
                              dict is from node and key in second dict is to node.
        - t = [int] current timestep
        - constraints = list of constraints
        - timesteps = list of all the timesteps when the aircraft are spawned
        - result = list of paths
        - original_path_lst = list that stores the original taxiing time of each agent
    RETURNS:
        - result = list of paths with no collisions (solution)
    """
    for ac in aircraft_lst:
        if ac.spawntime == t and len(result) <= ac.id:
            ac.status = "taxiing"
            ac.position = nodes_dict[ac.start]["xy_pos"]
            ac.plan_independent(nodes_dict, edges_dict, heuristics, t, ac)  # for each aircraft that is being spawned at timestep t
                                                                            # we first compute a path without imposing any constraints,
                                                                            # using plan_independent; this is the direct path from the
                                                                            # starting position to the goal position of the agent
            original_path = ac.path_to_goal  # here we store the path to goal for the aircraft, which does not include the first element
                                             # of the agent's path
            original_path_lst.append((len(original_path) + 1) / 2 - 0.5)  # we calcualte the taxiing time originally planned for the aircraft
                                                                          # we add 1 here, as the original path does not include the first
                                                                          # element (start node, spawn time, ac.id) of the agent's path
            ac.plan_prioritized(nodes_dict, edges_dict, heuristics, t, constraints, ac)  # calculating the path for each agent, taking into
                                                                                         # account the constraints that have been imposed for
                                                                                         # the agent
            agent = ac.id

            start_node = ac.start  # node from which planning should be done
            goal_node = ac.goal  # node to which planning should be done

            success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t, agent,
                                                      constraints) # saving the path of the agent


            # Adding Vertex constraints.
            for y in range(len(path)):

                for future_agent in range(agent + 1, len(timesteps)):
                    if [path[y][0]] != [34] and [path[y][0]] != [35] and [path[y][0]] != [36] and [path[y][0]] != [97] \
                            and [path[y][0]] != [98]:  # Multiple aircraft are allowed to be at the gates at the same time,
                                                       # therefore, we ignore these locations when imposing vertex constraints
                        constraint = {'agent': future_agent, 'loc': [path[y][0]], 'timestep': path[y][1]}
                        constraints.append(constraint)

                    if [path[y][0]] == [1.0]:
                        constraint = {'agent': future_agent, 'loc': [2.0], 'timestep': path[y][
                            1]}  # Restricting the aircraft from using the departure runway at the same time
                        constraints.append(constraint)
                    elif [path[y][0]] == [2.0]:
                        constraint = {'agent': future_agent, 'loc': [1.0], 'timestep': path[y][
                            1]}  # Restricting the aircraft from using the departure runway at the same time
                        constraints.append(constraint)

            # Adding Edge constraints.
            for k in range(len(path)):

                for future_agent in range(agent + 1, len(timesteps)):
                    if path[k][0] != 34 and path[k][0] != 35 and path[k][0] != 36 and path[k][0] != 97 \
                            and path[k][0] != 98:  # Multiple aircraft are allowed to be at the gates at the same time,
                                                   # therefore, we ignore these locations and nodes around them such as
                                                   # 37, 38, 92, 93, 94, 99, 100 when imposing edge constraints

                        constraint = {'agent': future_agent, 'loc': [path[k][0], path[k - 1][0]],
                                      'timestep': path[k][1]}
                        constraints.append(constraint)

                    if (path[0][0] == 37 or path[0][0] == 38) and (
                            [path[k][0]] == [92] or [path[k][0]] == [93] or [path[k][0]] == [94] or [path[k][0]] == [
                        99] or [path[k][0]] == [100]):  # Solving deadlock
                        constraint1 = {'agent': future_agent, 'loc': [path[k + 1][0], path[k][0]],
                                       'timestep': path[k][1] + 0.5}
                        constraint2 = {'agent': future_agent, 'loc': [path[k + 1][0], path[k][0]],
                                       'timestep': path[k][1]}

                        constraints.append(constraint1)
                        constraints.append(constraint2)


            if path is None:
                raise BaseException('No solutions')
            result.append(path)

    return result


