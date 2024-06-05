"""
Implement CBS here

R is the root node
R.constraints = []
R.paths = [] # list of paths for each agent, found using A* search
R.collisions = list of collisions for each agent, found using detect_collisions(R.paths)
R.cost = get_sum_of_costs(R.paths) 
insert R into OPEN # OPEN is the open list of nodes that need to be explored

while OPEN is not empty:

    ### we explore the node with the lowest cost
    N = node from open with the LOWEST COST                     # remember to remove this node from the open list!
    if N.collisions is empty:                                   # here we have used 'detect_collisions(N.paths)' to get N.collisions
        return N.paths                                          # The paths in N are now an optimal solution and N is a GOAL NODE
        
    collision = N.collisions[0]                                 # get the first collision in N.collisions (any collision)
    constraints = standard splitting(collision)                 # turn the collision into 2 constraints. This is a function that you need to implement!

    ### create new nodes for each new constraint
    for constraint in constraints:
        N_new = N.copy                                          # make a copy of N //// (or make a new node)
        N_new.constraints = N.constraints AND [constraint]      # add the constraint to the new node
        N_new.paths = N.paths                                   # copy the paths from N to N_new
        a_i = the agent in question                             # each constraint is connected to an agent.
        path = a_star(a_i, N_new.constraints)                   # find a new path for the agent in question, given the constraints
        
        ### 
        if path is not emtpy:
            replace the path fo agent a_i in N_new.paths with path
            N_new.collisions = detect_collisions(N_new.paths)
            N_new.cost = get_sum_of_costs(N_new.paths)
            insert N_new into OPEN

return "No solution found"
"""

import networkx as nx

def run_CBS(nodes_dict:dict, edges_dict:dict, aircraft_lst:list, heuristics:dict, t:int):
    """
    VARIABLE PROPERTIES:
    nodes_dict: {key:int :: {int:id:int, x_pos:float, y_pos:float, xy_pos:tuple, 'type':str, neighbors:set(of ids)}}
    edges_dict: {key:tuple(nodeID,nodeID) :: {id:tuple, from:int, to:int, length:float, weight:float, start_end_pos:tuple(of tuples)}}
    aircraft_lst: [Aircraft, Aircraft, ...] # I think
    heuristics: {key:int(fromNodeID) :: {key:int(toNodeID) :: heuristic}}
    t: int          # TIME??
    """
    
    print("**Running CBS**")
    
    # print("Pritning variable types: ")
    # print('nodes_dict: ', type(nodes_dict))
    # print('edges_dict: ', type(edges_dict))
    # print('aircraft_lst: ', aircraft_lst)
    # print('heuristics: ', type(heuristics))
    # print('t: ', type(t))
    
    # YOU MIGHT WANT THESE LATER!
    num_nodes_generated = 0
    num_nodes_expanded = 0

    # create an empty root node
    root = {'cost': 0,'constraints': [],'paths': {},'collisions': []}


    for aircraft in aircraft_lst: 
        if aircraft.spawntime == t:
            aircraft.status = "taxiing" 
            aircraft.position = nodes_dict[aircraft.start]["xy_pos"]
            aircraft.plan_independent(nodes_dict, edges_dict, heuristics, t)
            root['paths'][aircraft.id] = aircraft.path_to_goal
           
            # aircraft = aircraft_lst[i]
            # print("Aircraft: ", aircraft.id)
            # print("Planning")
            # aircraft.plan_independent(nodes_dict, edges_dict, heuristics, t)
            # print("plan finished")
            # print("Path: ", aircraft.path_to_goal)
            # root['paths'].append(aircraft.path_to_goal)


    print("Root node: ", root)
    # print("Root node paths: ", root['paths'])
    # print("Root node paths length: ", len(root['paths']))
    # print("Root node collisions: ", root['collisions'])


    # raise Exception("**CBS not finished yet**")
    return

def generate_paths():
    pass

class CBSSolver(object):
    """The high-evel search of CBS –––– Taken from the MAPF exercises"""

    def __init__(self, nodes_dict, edges_dict, aircraft_lst, heuristics, t, starts, goals, current_location_aircraft):
        pass
