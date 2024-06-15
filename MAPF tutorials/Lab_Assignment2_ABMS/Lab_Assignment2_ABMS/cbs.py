
import heapq
import copy
from single_agent_planner import get_sum_of_cost, complex_single_agent_astar

def detect_collision(path1:list, path2:list, agent1:int, agent2:int, t:int, ac_list):
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

    ac1 = ac_list[agent1]
    ac2 = ac_list[agent2]
    
    for i in range(len(path1)):
        for j in range(len(path2)):

            if ac1.speed == ac2.speed: # if the agents have the same speed
                
                # Vertex 1 – Same speed, same location, same time
                if path1[i][0] == path2[j][0] and path1[i][1] == path2[j][1]:
                    print("VERTEX 1")
                    # raise Exception("VERTEX 1")
                    return {'a1': agent1, 'a2': agent2, 'loc': [path1[i][0]], 'timestep': path1[i][1], 'type': 'vertex1'}

                # Edge 1 – Same speed and same edge, agents trade nodes and reach their respective new nodes at the same time
                if path1[i][1]>1 and path2[j][1]>1:
                    if path1[i][1] == path2[j][1] and path1[i-1][0] == path2[j][0] and path1[i][0] == path2[j-1][0]:
                        print("EDGE 1")
                        # raise Exception("EDGE 1")
                        # {agent1, agent2, [location1, location2], t, edge1} 
                        # note that agent 1 moves from location 1 to location and agent 2 does the opposite
                        return {'a1': agent1, 'a2': agent2, 'loc': [path1[i-1][0], path1[i][0]], 'timestep': path1[i][1], 'type': 'edge1'}
        


            else: # if the agents have different speeds

                if ac1.speed > ac2.speed: # if ac1 is faster
                    
                    if path1[i-1][1] == path2[j-1][1] + 0.25 and path1[i][1] == path2[j][1]:
                        # Vertex 2.3.1 – different speed same location (fast agent follows slow agent and catches up)
                        if path1[i-1][0] == path2[j-1][0] and path1[i][0] == path2[j][0]:
                            print("VERTEX 2.3.1 (agent 1 is faster than agent 2)")
                            # raise Exception("VERTEX 2.3.1 (agent 1 is faster than agent 2)")
                            return {'a1': agent1, 'a2': agent2, 'loc1': [path2[j-1][0]], 'loc2':[path2[j][0]], 'timestep': path1[i][1], 'type': 'vertex2.3.1'}
                        
                        # Vertex 2.1.1 – different speed, same location, same time (fast agent not following)
                        if path1[i][0] == path2[j][0] and path1[i-1][0] != path2[j-1][0]:
                            if path1[i][0] == path2[j][0] and path1[i-1][0] != path2[j-1][0]: # the previous step is not the same but the next one is
                                print("VERTEX 2.1.1 (agent 2 is faster than agent 1)")
                                # raise Exception("VERTEX 2.1.1 (agent 2 is faster than agent 1)")
                                return {'a1': agent1, 'a2': agent2, 'loc': [path1[i][0]], 'timestep': path1[i][1], 'type': 'vertex2.1.1'}
                        
                        # Vertex 2.2.1 – different speed same location (slow agent waiting at location and fast passes through)
                        if path2[j][0] == path2[j+1][0] == path1[i][0]:
                            if path2[j][1] == path1[i][1]-0.25 and path2[j+1][1] == path1[i][1]+0.25: # if agent 1 is waiting and agent 2 is passing through
                                print("VERTEX 2.2.1 (agent 1 is faster than agent 2)")
                                # raise Exception("VERTEX 2.2.1 (agent 1 is faster than agent 2)")
                                return {'a1': agent1, 'a2': agent2, 'loc': [path1[i][0]], 'timestep': path1[i][1], 'type': 'vertex2.2.1'}
                            
                    # Edge 2.1 – different speed and same edge
                    if path1[i][1]>1 and path2[j][1]>1:
                        if path1[i][1] == path2[j][1] and path1[i-1][0] == path2[j][0] and path1[i][0] == path2[j-1][0]:
                            print("EDGE 2.1 (agent 1 is faster than agent 2)")
                            # raise Exception("EDGE 2.1 (agent 1 is faster than agent 2)")
                            return {'a1': agent1, 'a2': agent2, 'loc': [path1[i-1][0], path1[i][0]], 'timestep': path1[i][1], 'type': 'edge2.1'}

        
                if ac1.speed < ac2.speed: # if ac2 is faster
                    
                    if path1[i-1][1] == path2[j-1][1] - 0.25 and path1[i][1] == path2[j][1]: # if time is the same and the last step of agent 1 is 0.25 less than the last step of agent 2
                        # Vertex 2.3.2 – different speed same location (fast agent follows slow agent and catches up)
                        if path1[i-1][0] == path2[j-1][0] and path1[i][0] == path2[j][0]: # if the last 2 steps are the same.
                            print("VERTEX 2.3.2 (agent 2 is faster than agent 1)")
                            # raise Exception("VERTEX 2.3.2 (agent 2 is faster than agent 1)")
                            return {'a1': agent1, 'a2': agent2, 'loc1': [path1[i-1][0]], 'loc2':[path1[i][0]], 'timestep': path1[i][1], 'type': 'vertex2.3.2'}

                        # Vertex 2.1.2 – different speed, same location, same time (fast agent not following)
                        if path1[i][0] == path2[j][0] and path1[i-1][0] != path2[j-1][0]: # the previous step is not the same but the next one is
                            print("VERTEX 2.1.2 (agent 2 is faster than agent 1)")
                            # raise Exception("VERTEX 2.1.2 (agent 2 is faster than agent 1)")
                            return {'a1': agent1, 'a2': agent2, 'loc': [path1[i][0]], 'timestep': path1[i][1], 'type': 'vertex2.1.2'}

                        # Vertex 2.2.2 – different speed same location (slow agent waiting at location and fast passes through)
                        if path1[i][0] == path1[i+1][0] == path2[j][0]:
                            if path1[i][1] == path2[j][1]-0.25 and path1[i+1][1] == path2[j][1]+0.25: # if agent 1 is waiting and agent 2 is passing through
                                print("VERTEX 2.2.2 (agent 2 is faster than agent 1)")
                                # raise Exception("VERTEX 2.2.2 (agent 2 is faster than agent 1)")
                                return {'a1': agent1, 'a2': agent2, 'loc': [path1[i][0]], 'timestep': path1[i][1], 'type': 'vertex2.2.2'}

                    # Edge 2.2 – different speed; reaching opposite nodes at the same time
                    if (path1[i-1][1] == path2[j-1][1]-0.25 and path1[i][1] == path2[j][1] and      # if the agents leave their start nodes at different
                        path1[i-1][0] == path2[j][0] and path1[i][0] == path2[j-1][0]):             # times but reach their goal nodes simultaneously
                        print("EDGE 2.2 (agent 2 is faster than agent 1)")
                        # raise Exception("EDGE 2.2 (agent 2 is faster than agent 1)")
                        return {'a1': agent1, 'a2': agent2, 'loc1': [path1[i][0], path2[j][0]], 'loc2':[path2[j-2][0],path1[i][0]], 'timestep1': path1[i][1], 'timestep2':path1[i][1]-0.25, 'type': 'edge2.2'}

                    # Edge 3.2 – different speed and same edge
                    if (path1[i-1][1] == path2[j-1][1] and path1[i][1] == path2[j][1]+0.25 and      # if the agents leave their start nodes simultaneously
                        path1[i-1][0] == path2[j][0] and path1[i][0] == path2[j-1][0]):             # but reach their goal nodes at different times
                        # raise Exception("EDGE 3.2 (agent 2 is faster than agent 1)")
                        return {'a1': agent1, 'a2': agent2, 'loc': [path2[j][0], path1[i][0]], 'timestep1': path1[i][1], 'timestep2':path2[j][1], 'type': 'edge3.2'}

    # for i in range(len(path1)):
    #     for j in range(len(path2)):
    #         # if path1[i][1]==path2[j][1] and path1[i][0]==path2[j][0]:                
    #         #     if ac1.speed == ac2.speed:
    #         #         print("vertex collision1")
    #         #         return {'a1': agent1, 'a2': agent2, 'loc': [path1[i][0]], 'timestep': path1[i][1]}
    #         if path1[i][1] >= path2[j][1] > path1[i-1][1] and path1[i][0] == path2[j][0]:
    #             if path1[i][1] == path2[j][1]:
    #                 print("vertex collision 2.1")    
    #             elif path1[i][1] > path2[j][1]:
    #                 print("vertex collision 2.2")
    #             # raise Exception(path1[i][1], ">", path2[j][1], ">", path1[i-1][1])
    #             # if ac1.speed != ac2.speed: # this should be obvious, but we need to check that the speeds are different
    #             return {'a1': agent1, 'a2': agent2, 'loc': [path1[i][0]], 'timestep': path1[i][1]}
    #         elif path2[j][1] >= path1[i][1] > path2[j-1][1] and path1[i][0] == path2[j][0]:
    #             print("vertex collision3")
    #             raise Exception("AC1 gets to ac2's location while AC2 is waiting")

    # # Edge collisions
    #         if path1[i][1]>1 and path2[j][1]>1: # THIS USED TO BE >=1 ON BOTH SIDES BUT I CHANGED IT SO THEY DON'T SEE THE FIRST STEP AS AN EDGE COLLISION WHEN THEIR GOALS AND STARTS ARE INVERSELY THE SAME
    #             if path1[i][1]==path2[j][1] and path1[i-1][0] == path2[j][0] and path1[i][0] == path2[j-1][0]:
    #                 print("edge collision1")
    #                 return {'a1': agent1,'a2': agent2, 'loc': [path1[i-1][0], path1[i][0]], 'timestep': path1[i][1]}

    #             # Speed-based edge collisions
    #             else:
    #                 # raise Exception("Making speed-based edge collisions")
    #                 if ac1.speed > ac2.speed:
    #                     if path1[i][1]+0.25==path2[j][1] and path1[i-1][0] == path2[j][0] and path1[i][0] == path2[j-1][0]:
    #                     # raise Exception("Making speed-based edge collisions where ac1 is faster than ac2")
    #                         print("edge collision2")
    #                         return {'a1': agent1,'a2': agent2, 'loc': [path1[i-1][0], path1[i][0]], 'timestep': path1[i][1]}
    #                 elif ac1.speed < ac2.speed:
    #                     if path1[i][1]-0.25==path2[j][1] and path1[i-1][0] == path2[j][0] and path1[i][0] == path2[j-1][0]:
    #                         print("edge collision3")
    #                         return {'a1': agent1,'a2': agent2, 'loc': [path1[i-1][0], path1[i][0]], 'timestep': path2[j][1]}

    # # # Two aircraft using the same runway
    # if path1[-1][1]==path2[-1][1] and ((path1[-1][0]==1 or path1[-1][0]==2) and (path2[-1][0]==1 or path2[-1][0]==2)):
        
    #     # Determining when two aircraft intend to use the same runway, at the same time
    #     if agent1<agent2:

    #         return {'a1': agent2, 'a2': agent2, 'loc': [path2[-1][0]], 'timestep': path2[-1][1]}
    #     else:
    #         return {'a1': agent1, 'a2': agent1, 'loc': [path1[-1][0]], 'timestep': path1[-1][1]}


    # Dead lock at the gates
    # checking whether for the two agents whose paths we are considering, one agent is an arriving aircraft and it is
    # 2 nodes away from its goal gate, and the other agent is a departing aircraft that was just spawned at the same gate
    if len(path1)==2 and len(path2)>2 and path1[-1][0]==path2[0][0] and (path1[-1][0]==97 or path1[-1][0]==34 or path1[-1][0]==35 or path1[-1][0]==36 or path1[-1][0]==98) and (path2[0][0]==1 or path2[0][0]==2):
        if t==path1[-3][1]:  # checking if the time step is equal to that when the arriving aircraft is at two nodes away from the gate
            return {'a1': agent2, 'a2': agent2, 'loc': [path2[1][0]], 'timestep': path1[-3][1]}

    elif len(path2)==2 and len(path1)>2 and path2[-1][0]==path1[0][0] and (path2[-1][0]==97 or path2[-1][0]==34 or path2[-1][0]==35 or path2[-1][0]==36 or path2[-1][0]==98) and (path1[0][0]==1 or path1[0][0]==2):
        if t==path2[-3][1]:
            return {'a1': agent1, 'a2': agent1, 'loc': [path1[1][0]], 'timestep': path2[-3][1]}


def detect_collisions(paths,t,ac_list):
    """
        This function detects all the first collisions happening between all pairs of agents that are taxiing.
        INPUT:
            - paths = list of paths for all the agents
            - t = current timestep
        RETURNS:
            - first_collisions = list of dictionaries representing the first collisions between all pairs of agents
    """
    print("**DETECTING COLLISIONS**")

    first_collisions = []  # Initialising list of first collisions

    for agent1 in range(len(paths)):
        for agent2 in range(agent1, len(paths)):
            if agent1 != agent2 and len(paths[agent1])>0 and len(paths[agent2])>0:
                print("detecting collision between agent {} and agent {}".format(agent1, agent2))
                if detect_collision(paths[agent1], paths[agent2], paths[agent1][0][2], paths[agent2][0][2],t, ac_list):
                    # Appending the collisions that were identified;
                    # paths[agent1][0][2] and paths[agent2][0][2] were used to extract the correct id of the agents
                    # that are involved in the collisions, as the elements in teh path of each agent are of the form:
                    # (node, timestep, ac.id)
                    print("!!!Collision detected between agent {} and agent {}".format(agent1, agent2))
                    first_collisions.append(detect_collision(paths[agent1], paths[agent2], paths[agent1][0][2], paths[agent2][0][2],t, ac_list))
            else:
                if agent1 != agent2: print("No collision detected between agent {} and agent {}".format(agent1, agent2))

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
    print("**STANDARD SPLITTING**")
    return_list = []
    agent1 = collision.get('a1')
    agent2 = collision.get('a2')
    collision_type = collision.get('type')
    
    if collision_type not in ['edge2.1','edge2.2','edge3.1','edge3.2']:  # Edge collisions of type 2 & 3 have 2 separate time steps.
       timestep = collision.get('timestep')
        
    # Vertex collisions
        # Agents have the same speed

    if collision_type == 'vertex1': # Vertex 1
        location = collision['loc']
        return_list.append({'agent': agent1, 'loc': location, 'timestep': timestep})
        return_list.append({'agent': agent2, 'loc': location, 'timestep': timestep})
        raise Exception("VERTEX 1 collision detected :: constraints generated: {}".format(return_list))

        # Agent 1 is faster than agent 2

    elif collision_type == 'vertex2.1.1': # Vertex 2.1.1
        location = collision['loc']
        return_list.append({'agent': agent1, 'loc': location, 'timestep': timestep})
        return_list.append({'agent': agent2, 'loc': location, 'timestep': timestep})
        raise Exception("VERTEX 2.1.1 collision detected :: constraints generated: {}".format(return_list))
    
    elif collision_type == 'vertex2.2.1': # Vertex 2.2.1
        pass
    
    elif collision_type == 'vertex2.3.1': # Vertex 2.3.1
        location1 = collision['loc1']; location2 = collision['loc2']
        return_list.append({'agent': agent2, 'loc': location1, 'timestep': timestep}) # Agent 2 cannot wait for agent 1 to pass through
        return_list.append({'agent': agent1, 'loc': location2, 'timestep': timestep}) # Agent 1 cannot be at agent2's final location
        raise Exception("VERTEX 2.3.1 collision detected :: constraints generated: {}".format(return_list))
    
        # Agent 2 is faster than agent 1

    elif collision_type == 'vertex2.1.2': # Vertex 2.1.2
        location = collision['loc']
        return_list.append({'agent': agent1, 'loc': location, 'timestep': timestep})
        return_list.append({'agent': agent2, 'loc': location, 'timestep': timestep})
        raise Exception("VERTEX 2.1.2 collision detected :: constraints generated: {}".format(return_list))
    
    elif collision_type == 'vertex2.2.2': # Vertex 2.2.2
        pass
    
    elif collision_type == 'vertex2.3.2': # Vertex 2.3.2
        location1 = collision['loc1']; location2 = collision['loc2']
        return_list.append({'agent': agent1, 'loc': location1, 'timestep': timestep}) # Agent 1 cannot wait for agent 2 to pass through
        return_list.append({'agent': agent2, 'loc': location2, 'timestep': timestep}) # Agent 2 cannot be at agent1's final location
        raise Exception("VERTEX 2.3.2 collision detected :: constraints generated: {}".format(return_list))

    # Edge collisions
        # Agents have the same speed
    elif collision_type == 'edge1': # Edge 1
        location = collision['loc']
        return_list.append({'agent': agent1, 'loc': location, 'timestep': timestep})
        return_list.append({'agent': agent2, 'loc': [location[-1], location[0]], 'timestep': timestep})
        raise Exception("EDGE 1 collision detected :: constraints generated: {}".format(return_list))

        # Agent 1 is faster than agent 2
    elif collision_type == 'edge2.1': # Edge 2.1
        pass

    elif collision_type == 'edge3.1': # Edge 3.1
        pass

        # Agent 2 is faster than agent 1
    elif collision_type == 'edge2.2': # Edge 2.2
        location1 = collision['loc1']; location2 = collision['loc2']
        timestep1 = collision['timestep1']; timestep2 = collision['timestep2']
        return_list.append({'agent': agent2, 'loc': location1, 'timestep': timestep1}) # Agent 2 cannot go to Agent1's starting location at t
        return_list.append({'agent': agent2, 'loc': location2, 'timestep': timestep2}) # Agent 2 cannot go to Agent1's final location at t-0.25
        raise Exception("EDGE 2.2 collision detected :: constraints generated: {}".format(return_list))
    
    elif collision_type == 'edge3.2': # Edge 3.2
        location = collision['loc']
        timestep1 = collision['timestep1']; timestep2 = collision['timestep2']
        return_list.append({'agent': agent1, 'loc': location, 'timestep': timestep1})                       # Agent 1 cannot go to Agent2's starting location at t[agent1]
        return_list.append({'agent': agent2, 'loc': [location[-1], location[0]], 'timestep': timestep2})    # Agent 2 cannot go to Agent1's starting location at t[agent2] –– t[agent2] == t[agent1] - 0.25
        raise Exception("EDGE 3.2 collision detected :: constraints generated: {}".format(return_list))




    # Vertex collision
    # if len(collision.get('loc')) == 1:  # if the location is only one element long, then we are dealing with a vertex collision
    #     # print("Vertex collision detected")
    #     location = collision['loc']
    #     return_list.append({'agent': agent1, 'loc': location, 'timestep': timestep})
    #     return_list.append({'agent': agent2, 'loc': location, 'timestep': timestep})

    # # Edge collision
    # if len(collision.get('loc')) == 2:  # if the location is two elements long, then we are dealing with an edge collision
    #     # print("Edge collision detected")
    #     location = collision['loc']
    #     if ac1.speed > ac2.speed:
    #         return_list.append({'agent': agent1, 'loc': location, 'timestep': timestep})
    #     elif ac1.speed < ac2.speed:
    #         return_list.append({'agent': agent2, 'loc': [location[-1], location[0]], 'timestep': timestep})
    #     else:
    #         return_list.append({'agent': agent1, 'loc': location, 'timestep': timestep})
    #         return_list.append({'agent': agent2, 'loc': [location[-1], location[0]], 'timestep': timestep}) # reverse edge

    print("Generated constraints:", return_list)
    return return_list

class CBSSolver(object):
    """The high-evel search of CBS –––– Taken from the MAPF exercises"""

    def __init__(self, nodes_dict, edges_dict, aircraft_lst, heuristics, t, starts, goals, current_location_aircraft):
        
        self.nodes_dict = nodes_dict
        self.edges_dict = edges_dict
        self.aircraft_lst = aircraft_lst
        self.heuristics = heuristics
        self.t = t
        self.starts = starts
        self.goals = goals
        self.current_location_aircraft = current_location_aircraft

        self.num_nodes_generated = 0
        self.num_nodes_expanded = 0
        self.CPU_time = 0

        self.open_list = []
    
    def push_node(self, node):
        heapq.heappush(self.open_list, (node['cost'], len(node['collisions']), self.num_nodes_generated, node))
        print("Generate node: {}".format(self.num_nodes_generated))
        print("Open list:")
        for i in range(len(self.open_list)):
            print("Node", self.open_list[i][2], ":", self.open_list[i])
        self.num_nodes_generated += 1
        if self.num_nodes_generated >= 1000:
            raise Exception("Too many nodes: {}".format(self.num_nodes_generated))

    def pop_node(self):
        _, _, id, node = heapq.heappop(self.open_list)
        print("Expand node: {}".format(id))
        print(node)
        self.num_nodes_expanded += 1
        return node
    
    def find_solution(self, constraints, existent_paths, current_location_aircraft, collisions, time_start, original_path_lst):

        paths_to_go = [[] for i in range(len(existent_paths))]  # Initialising a list of the path that still needs to be
                                                                # completd by each agent, from next timestep onwards
        
        # init root node
        root = {'cost': 0, 'constraints': [], 'paths': [], 'collisions': []}

        for ac in self.aircraft_lst:
            if ac.spawntime == self.t:
                ac.status = "taxiing" 
                ac.position = self.nodes_dict[ac.start]["xy_pos"]
                agent = ac.id
                start_node = ac.start
                goal_node = ac.goal
                path_so_far = []    # initialising a list to later store the path completed by the agent until and including the
                                    # current timestep; this list will be used to keep track of the entire path followed by the agent

                ac.plan_independent(self.nodes_dict, self.edges_dict, self.heuristics, self.t)  # first, plan the paths without
                                                                                                # any constraints
                original_path = ac.path_to_goal
                original_path_lst.append((len(original_path) + 1)/2 - 0.5) # THIS MIGHT NEED THE TIME STEP SIZE CALCULATOR!
                # root['paths'].append(ac.path_to_goal)
                # storing in "path" the initial path of the ac, which is computed without any constraints
                # root['constraints'] is still empty at this point.
                success, path = complex_single_agent_astar(self.nodes_dict, start_node, goal_node, self.heuristics, self.t, agent, ac.speed, root['constraints'], path_so_far)

                if success:
                    ac.path_to_goal = path[1:]
                    next_node_id = ac.path_to_goal[0][0] #next node is first node in path_to_goal
                    ac.from_to = [path[0][0], next_node_id]
                    ac.last_node = path[0][0]
                    
                    # storing the path of the agent in the list of paths that still need to be completed
                    # print("agent:", agent)
                    paths_to_go[agent] = path
                    # print("paths to go:",paths_to_go, "\nLength of paths to go:", len(paths_to_go))
                    existent_paths[agent] = path
                    root['paths'] = paths_to_go

                else:
                    raise Exception("No solution found for", ac.id)

                # check path
                if path[0][1] != self.t:
                    raise Exception("Something is wrong with the timing of the path planning")
                
                if path is None:
                    raise Exception("No solutions")
                
        for ac in self.aircraft_lst:
            if ac.status == 'taxiing' and ac.spawntime != self.t:
                paths_to_go[ac.id]= [(ac.from_to[0], ac.from_to[1], ac.id)] + ac.path_to_goal

                
        
        
        root['paths'] = paths_to_go  # Then, we store all the paths_to_go in the root node
        root['cost'] = get_sum_of_cost(root['paths'])  # We compute the cost of the current solution/set of paths
        root['collisions'] = detect_collisions(root['paths'],self.t,self.aircraft_lst)  # Then, we store the collisons that take place in the paths
        collisions.append(root['collisions'])
        self.push_node(root)  # and we add the root node to the list

        # print("Root node pushed to open list:", root)

        while len(self.open_list) > 0: # while there are still nodes in the open list
            P = self.pop_node()  # we pop the node with the lowest cost from the open list

            if len(P['collisions']) == 0 or P['collisions'] is None:  # if there are no collisions in the paths of the agents
                current_paths = P['paths']  # then we have found a solution

                new_paths = [[] for i in range(len(current_paths))]    # initialising a list of new paths, in which we will store the current
                                                                        # solutions for each agent
                                            
                # first, we will store all the elements from the past which are fund in the "existent_paths" list
                for i in range(len(existent_paths)):  # for each agent
                    for element in current_paths[i]:  # for each element in the path of the agent
                        if element[1] <= self.t:  # if the timestep of the element is less than or equal to the current timestep
                            new_paths[i].append(element)

                

                for ac in self.aircraft_lst:
                    if ac.status == 'taxiing':

                        ac.path_to_goal = current_paths[ac.id][1:]
                        next_node_id = ac.path_to_goal[0][0]  #next node is first node in path_to_goal
                        ac.from_to = [current_paths[ac.id][0][0], next_node_id]
                        ac.last_node = current_paths[ac.id][0][0]
                        
                        new_paths[ac.id].append(current_paths[ac.id][0])
                        for i in range(1, len(current_paths[ac.id])):
                                new_paths[ac.id].append(current_paths[ac.id][i])
                
                return new_paths
            
            collision = P['collisions'][0]
            # print("collision:", collision)  
            collisions.append(collision) 

            constraints_new = standard_splitting(collision)  # we split the collision into constraints

            # print("constraints_new:", constraints_new)

            for constraint in constraints_new:

                constraints.append(constraint)

                # initiailisng the node Q
                Q = {'cost': 0, 'constraints': [], 'paths': [], 'collisions': []}

                Q['constraints'] = copy.deepcopy(P['constraints'])


                if constraint not in Q['constraints']:
                    Q['constraints'].append(constraint)

                Q['paths'] = copy.deepcopy(P['paths'])
                agent = constraint['agent']



                # THIS IS WHERE YOU'LL NEED TO CHANGE THE PATHS TO THE PREDICTED PATHS
                start = 0
                for current_location in current_location_aircraft: 
                    # print("current_location:", current_location, "agent:", agent, "t:", self.t)
                    if current_location[1] == self.t and current_location[2] == agent:
                        start = current_location[0]    
                        # print("start:", start)


                for ac in self.aircraft_lst:
                    if agent == ac.id:
                        if start != 0:
                            path_so_far = []
                            for element in existent_paths[agent]:
                                if element[1] < self.t:
                                    path_so_far.append(element) # creating a list with the path that was completed so far by the agent,
                                                                # to use it for replanning purposes
                            
                            # print("running complex astar")
                            success, path = complex_single_agent_astar(self.nodes_dict, start, self.goals[agent], self.heuristics, self.t, agent, ac.speed, Q['constraints'], path_so_far)


                            if path is not None and len(path)>0:
                                path_so_far = []
                                for element in existent_paths[agent]:
                                    if element[1] < self.t:
                                        path_so_far.append(element)

                                existent_paths[agent] = path_so_far + path  # and then we are updating the existent_paths list with the
                                                                            # path performed by the agent so far, to which we add the path
                                                                            # path performed by the agent so far, to which we add the path
                                                                            # that is planned for the aircraft to complete from this
                                                                            # timestep onwards, until it reaches its goal
                                paths_to_go[agent] = path  # we are storing the path that was replanned and still needs to be completed
                                                           # by the aircraft in its corresponding element in paths_to_go

                                Q['paths'][agent] = path
                                Q['collisions'] = detect_collisions(Q['paths'],self.t,self.aircraft_lst)
                                Q['cost'] = get_sum_of_cost(Q['paths'])

                                self.push_node(Q)

        return "No Solutions"
    




def run_CBS(nodes_dict:dict, edges_dict:dict, aircraft_lst:list, heuristics:dict, t:int):
    """
    VARIABLE PROPERTIES:
    nodes_dict: {key:int :: {int:id:int, x_pos:float, y_pos:float, xy_pos:tuple, 'type':str, neighbors:set(of ids)}}
    edges_dict: {key:tuple(nodeID,nodeID) :: {id:tuple, from:int, to:int, length:float, weight:float, start_end_pos:tuple(of tuples)}}
    aircraft_lst: [Aircraft, Aircraft, ...]
    heuristics: {key:int(fromNodeID) :: {key:int(toNodeID) :: heuristic}}
    t: int          # TIME??
    """
    

    # done = True

    # if done:
    #     raise Exception("**CBS not finished yet**")
    #     return "No solution found"
    
    
    # else:  
    #     raise Exception("**CBS not finished yet**")
