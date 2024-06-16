import random as rd

"""
MAIN FUNCTIONS
"""

def detect_deviation(ac_list, nodes_dict, t):
    """
    Detects if an aircraft deviates from the planned path.
    INPUT:
        - aircraft_lst = [Aircraft, Aircraft, ...]
        - nodes_dict = {key:int :: {int:id:int, x_pos:float, y_pos:float, xy_pos:tuple, 'type':str, neighbors:set(of ids)}}
        - t = [int] current timestep
    RETURNS:
        - deviation = True if deviation is detected, False if no deviation is detected
    """
    is_waiting(ac_list, nodes_dict) # check if there are aircraft waiting at nodes
    for ac in ac_list:
        if ac.status == 'taxiing' and ac.waiting == False:
            next_node_pos = nodes_dict[ac.path_to_goal[0][0]]["xy_pos"] # position of next node in the plan    
            if ac.position == next_node_pos and ac.path_to_goal[0][1] != t: # at node but not at the right time
                print("AC{} too early".format(ac.id)) 
                
                return True
            elif ac.position != next_node_pos and ac.path_to_goal[0][1] == t: # not at the node when it is time to be there
                print("AC{} too late".format(ac.id))
                
                return True
            
    return False # no deviation detected


def prep_replan(ac_list, nodes_dict, edges_dict, heuristics, t):
    """
    INPUT:
        - ac_list = [list] list of aircraft objects
        - nodes_dict = [dict] dictionary with nodes and node properties
        - t = [int] current timestep
        ****
        - edges_dict = [dict] dictionary with edges and edge properties
        - heuristics = [dict] dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
        ****

    RETURNS:
        - ac_list with modified start positions and times.
    """

    # new_list = [] # the list that will be populated and returned

    for ac in ac_list: 
        if ac.status == 'taxiing':
            # new_list.append(ac) # This never ended up being used...
    
            if is_it_a_node(ac.position,nodes_dict): # if the aircraft is at a node in the graph
                ac.replan_time = t
                print("AC{} is at a node at location {}".format(ac.id, ac.position))
                
            else:
                print("AC{} is between nodes at location {}".format(ac.id, ac.position))
                next_node = ac.path_to_goal[0][0] # Get ID of the next planned node 
                distance_to_next_node = calculate_distance(ac.position, nodes_dict[next_node]["xy_pos"]) # Calculate distance between agent's current position and the next node
                time_to_next_node = distance_to_next_node / ac.speed # Calculate the time it will take for agent to reach the next node
                arrival_time = t + time_to_next_node # Calculate the arrival time at the next node
                ac.replan_time = arrival_time
                print("AC{} will reach node {} at location {} at time {}".format(ac.id, next_node, nodes_dict[next_node]["xy_pos"], arrival_time))
                print("which is the same as its replan time {}".format(ac.replan_time))    


def run_independent_replanner(ac_list,nodes_dict, edges_dict, heuristics, t):
    """
    Just like the independent planner, but uses the internal replan time instead of t.
    """
    print("**Running Independent Re-planner**")
    for ac in ac_list:
        if ac.status == 'taxiing': # if the agent is taxiing
            ac.replanning = True
            if ac.spawntime == t:
                ac.replanning == False
            ac.plan_independent(nodes_dict, edges_dict, heuristics, t) # plan the path
            ac.replanning = False


def aircraft_infront(ac_list, t):
    """
    INPUT:
        - ac_list = [list] list of aircraft objects
        - nodes_dict = [dict] dictionary with nodes and node properties
        - t = [int] current timestep
    RETURNS:
        - Modifies an agent's speed if it detects that there is an aircraft 1 node in front of it.

    This is a sort of speed profile that applies universally to the agents. It is a form of local adjustment
    to avoid rear-end collisions in a semi-realistic way.
    """
    for ac1 in ac_list:
        for ac2 in ac_list:
            if ac1.status == 'taxiing' and ac2.status == 'taxiing':
                if ac1.speed > ac2.speed: # if ac1 is faster than ac2
                    if calculate_distance(ac1.position, ac2.position) <= 0.5: # we check that the distance between them is <= 0.5
                        ac1.speed = 1
                        print("AC{} is slowing down to speed {} to match AC{}".format(ac1.id, ac1.speed, ac2.id))
                        # raise Exception("AC{} is faster than AC{} and is too close".format(ac1.id, ac2.id))

                elif ac1.speed < ac2.speed: # if ac2 is faster than ac1
                    if calculate_distance(ac1.position, ac2.position) <= 0.5: # we check that the distance between them is <= 0.5
                        # we also need to check that ac2 is BEHIND but one step at a time.
                        ac2.speed = 1
                        print("AC{} is slowing down to speed {} to match AC{}".format(ac2.id, ac2.speed, ac1.id))
                        # raise Exception("AC{} is faster than AC{} and is too close".format(ac2.id, ac1.id))

            
            
            
"""
UTILITY FUNCTIONS
"""

def is_waiting(ac_list, nodes_dict): 
    """
    INPUT:
        - ac_list = [list] list of aircraft objects
        - nodes_dict = [dict] dictionary with nodes and node properties
    RETURNS:   
        - None
    
    this fucntion checks if there is an aircraft waiting in place. if so, then the 
    aircraft's waiting attribute is set to True. If it is no longer waiting, the 
    aircraft's internal move function will reset this attribute to False.
    """
    for ac in ac_list:
        if ac.status == 'taxiing':
            from_node = ac.from_to[0]
            to_node = ac.from_to[1]
            xy_from = ac.nodes_dict[from_node]["xy_pos"] #xy position of from node
            xy_to = ac.nodes_dict[to_node]["xy_pos"] #xy position of to node
            x = xy_to[0]-xy_from[0]
            y = xy_to[1]-xy_from[1]

            if x==0 and y == 0:
                ac.waiting = True
        

def is_it_a_node(position, nodes_dict, retrieve_node=False):
    """
    INPUT:
        - position of agent
        - nodes_dict = [dict] dictionary with nodes and node properties
    RETURNS:
        - BOOL: True if agent is at a node, False if agent is not at a node
        - node: if retrieve_node is True, returns the node at which the agent is located
    """

    for node in nodes_dict:
        if position == nodes_dict[node]["xy_pos"]:
            if retrieve_node == True:
                return node
            return True
    return False


def calculate_distance(ac_position, next_node_position):
    """
    INPUT:
        - ac_position = [tuple] (x,y) position of aircraft
        - next_node_position = [tuple] (x,y) position of next node
    RETURNS:
        - distance = [float] distance between ac_position and next_node_position
    """

    distance = ((ac_position[0] - next_node_position[0])**2 + (ac_position[1] - next_node_position[1])**2)**0.5
    return distance
                    

def spawner(t, id=0):
    # rd.seed(1)              # seed
    spawn = False           # spawn boolean
    arr_nodes = [1,2,37,38]
    dep_nodes = [97,34,34,36,98]

    id += 1
    a_d = "A" if rd.random() < 0.5 else "D"
    if a_d == "A":
        start = rd.choice(arr_nodes)
        goal = rd.choice(dep_nodes)
    else:
        start = rd.choice(dep_nodes)
        goal = rd.choice(arr_nodes)
    speed = 1 if rd.random() < 0.5 else 2
    spawn_time = t

    return id, a_d, start, goal, speed, spawn_time
