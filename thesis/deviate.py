import random as rd

"""
MAIN FUNCTIONS
"""

def detect_deviation(ac_list, nodes_dict, t):
    """
    INPUT:
        - aircraft_lst = [Aircraft, Aircraft, ...]
        - nodes_dict = {key:int :: {int:id:int, x_pos:float, y_pos:float, xy_pos:tuple, 'type':str, neighbors:set(of ids)}}
        - t = [int] current timestep
    RETURNS:
        - deviation = True if deviation is detected, False if no deviation is detected
    
    This function detects if an aircraft deviates from the planned path.
    """
    is_waiting(ac_list, nodes_dict) # check if there are aircraft waiting at nodes. This is here to
                                    # avoid an error where a waiting aircraft is seen as a deviation.
                                    # It happens when the position matches the next step but the time doesn't.

    switch = False # switch to detect deviation

    for ac in ac_list:
        if ac.status == 'taxiing' and ac.waiting == False:
            next_node_pos = nodes_dict[ac.path_to_goal[0][0]]["xy_pos"] # position of next node in the plan    
            if ac.position == next_node_pos and ac.path_to_goal[0][1] != t: # at node but not at the right time – SPEED UP! 
                print("AC{} too early".format(ac.id)) 
                ac.deviated = True
                ac.deviation_type = "early"
                switch = True
                # return True

            elif ac.position != next_node_pos and ac.path_to_goal[0][1] == t: # not at the node when it is time to be there – SLOW DOWN! 
                print("AC{} too late".format(ac.id))
                ac.deviated = True
                ac.deviation_type = "late"
                switch = True
                # return True
            
    return switch # return the switch


def prep_CBS_replan(ac_list, current_location_aircraft):
    """
    INPUT:
        - ac the aircraft in question
        - current_location_aircraft = [list] list with current locations of aircraft
    RETURNS:
        - ac_loctions_list with modified start positions and times.

        This function makes sure that all aircraft positions are primed for CBS replanning in case of a deviation.
    """

    replan_locations_list = [] # the list that will be populated and returned

    for ac in ac_list:
        if ac.status == "taxiing":
            for location in current_location_aircraft:

                if ac.deviated == True: # if it has deviated

                    if ac.speed == 2:
                        if location[2] == ac.id and location[0] == ac.path_to_goal[0][0]:
                            replan_location = location
                            replan_locations_list.append(replan_location)

                            # raise Exception("AC{} gets the replan location {}".format(ac.id, replan_location))

                    # if it has slowed down
                    elif ac.speed == 1:
                        if location[2] == ac.id:
                            if is_it_a_node(ac.position, ac.nodes_dict) == True:
                                _,replan_node = is_it_a_node(ac.position, ac.nodes_dict, retrieve_node=True)
                                replan_time = location[1]
                                replan_location = (replan_node, replan_time, ac.id)
                                replan_locations_list.append(replan_location)

                                # raise Exception("AC{} gets the replan location {}".format(ac.id, replan_location))

                            else:
                                replan_node = ac.path_to_goal[0][0]
                                time_to_next_node = 0.25
                                replan_time = location[1] + time_to_next_node
                                replan_location = (replan_node, replan_time, ac.id)
                                replan_locations_list.append(replan_location)

                                # raise Exception("AC{} gets the replan location {}".format(ac.id, replan_location))
                    else:
                        raise Exception("AC{} has no speed or speed is not 1 or 2".format(ac.id))

                elif ac.deviated == False:
                    
                    if ac.speed == 2:
                    
                        if location[2] == ac.id: 
                    
                            replan_location = location
                            replan_locations_list.append(replan_location)

                            # raise Exception("AC{} has not deviated and gets its current location {}".format(ac.id, location))

                    elif ac.speed == 1:
                        
                        if location[2] == ac.id:
                        
                            if is_it_a_node(ac.position, ac.nodes_dict) == True:
                        
                                _,replan_node = is_it_a_node(ac.position, ac.nodes_dict, retrieve_node=True)
                                replan_time = location[1]
                                replan_location = (replan_node, replan_time, ac.id)
                                replan_locations_list.append(replan_location)

                                # raise Exception("AC{} has not deviated and gets the replan location {}".format(ac.id, replan_location))

                            else:
                                replan_node = ac.path_to_goal[0][0]
                                time_to_next_node = 0.25
                                replan_time = location[1] + time_to_next_node
                                replan_location = (replan_node, replan_time, ac.id)
                                replan_locations_list.append(replan_location)
                                # raise Exception("AC{} has not deviated and gets the replan location {}".format(ac.id, replan_location))
                    else:
                        raise Exception("AC{} has no speed or speed is not 1 or 2".format(ac.id))
                else: 
                    raise Exception("AC{} has no deviation attribute".format(ac.id))
                
    return replan_locations_list


def check_deviation(ac):
    """
    INPUT:
        - ac = [Aircraft] aircraft object
    RETURNS:
        - None : allows aircraft to deviate by changing their speeds

        This function checks the speed profiles of an agent and determines whether it will deviate from
        its planned path at the current time. If it is supposed to deviate, its speed is adjusted accordingly.
    """
    
    if len(ac.path_to_goal) > 0:
        if ac.profile == "cS": # Cautious Start  –– only used for departing agents
            if ac.distance_travelled >= len(ac.path_to_goal) / 3: 
                ac.speed = 2
                ac.profile = "fast" # reset to end deviation behaviour.
                print("AC{} has travelled a third of its path and is now speeding up".format(ac.id))

        elif ac.profile == "cE": # Cautious End –– only used for arriving agents
            if ac.distance_travelled >= len(ac.path_to_goal) / 1.34: # 3/4 of the way there!
                ac.speed = 1
                ac.profile = "slow"

        elif ac.profile == "fS": # Fast start –– only used for arriving agents
            if ac.distance_travelled >= len(ac.path_to_goal) / 4: # 1/4 of the way there!
                ac.speed = 1
                ac.profile = "slow"
                print("AC{} has travelled 4 units and is now slowing down".format(ac.id))

        elif ac.profile in ["fast", "slow"]: # Continuous fast or slow
            pass



            
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









"""
vv **SCRAPYARD** vv
"""

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
