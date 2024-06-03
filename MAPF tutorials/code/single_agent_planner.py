import heapq

def move(loc, dir):
    directions = [(0, -1), (1, 0), (0, 1), (-1, 0), (0, 0)]
    return loc[0] + directions[dir][0], loc[1] + directions[dir][1]


def get_sum_of_cost(paths):
    rst = 0
    for path in paths:
        rst += len(path) - 1
    return rst


def compute_heuristics(my_map, goal):
    # Use Dijkstra to build a shortest-path tree rooted at the goal location
    open_list = []
    closed_list = dict()
    root = {'loc': goal, 'cost': 0}
    heapq.heappush(open_list, (root['cost'], goal, root))
    closed_list[goal] = root
    while len(open_list) > 0:
        (cost, loc, curr) = heapq.heappop(open_list)
        for dir in range(4):
            child_loc = move(loc, dir)
            child_cost = cost + 1
            if child_loc[0] < 0 or child_loc[0] >= len(my_map) \
               or child_loc[1] < 0 or child_loc[1] >= len(my_map[0]):
               continue
            if my_map[child_loc[0]][child_loc[1]]:
                continue
            child = {'loc': child_loc, 'cost': child_cost}
            if child_loc in closed_list:
                existing_node = closed_list[child_loc]
                if existing_node['cost'] > child_cost:
                    closed_list[child_loc] = child
                    # open_list.delete((existing_node['cost'], existing_node['loc'], existing_node))
                    heapq.heappush(open_list, (child_cost, child_loc, child))
            else:
                closed_list[child_loc] = child
                heapq.heappush(open_list, (child_cost, child_loc, child))

    # build the heuristics table
    h_values = dict()
    for loc, node in closed_list.items():
        h_values[loc] = node['cost']
    return h_values


def build_constraint_table(constraints, agent):
    ##############################
    # Task 1.2/1.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.

    # max_timestep = -1
    # for constraint in constraints:
    #     if constraint['agent'] == agent:
    #         max_timestep = max(max_timestep, constraint['timestep'])

    # constraint_table = [[] for _ in range(max_timestep + 1)]

    # for constraint in constraints:
    #     if constraint['agent'] == agent:
    #         constraint_table[constraint['timestep']].append({'loc':constraint['loc']})

    # return constraint_table

    positive = []
    negative = []
    max_timestep = -1

    for constraint in constraints:
        if constraint['positive']:
            if constraint['agent'] == agent:
                positive.append(constraint)
            else: 
                negative.append(constraint)
            max_timestep = max(max_timestep, constraint['timestep'])

        elif constraint['agent'] == agent:
            negative.append(constraint)
            max_timestep = max(max_timestep, constraint['timestep'])

    constraint_table = [[] for _ in range(max_timestep + 1)]
    for constraint in positive:
        if len(constraint['loc']) == 1: # positive vertex constraint
            constraint_table[constraint['timestep']].append({'loc': [constraint['loc'][0]], 'positive': True})
        else: # positive edge constraint
            constraint_table[constraint['timestep'] - 1].append({'loc': [constraint['loc'][0]], 'positive': True})
            constraint_table[constraint['timestep']].append({'loc': [constraint['loc'][0]], 'positive': True})

    for constraint in negative:
        if len(constraint['loc']) == 1: #vertex constraint
            constraint_table[constraint['timestep']].append({'loc': constraint['loc'], 'positive': False})
        elif constraint['positive']:  # positive edge constraint for other agents
            constraint_table[constraint['timestep'] - 1].append({'loc': [constraint['loc'][0]], 'positive': False})
            constraint_table[constraint['timestep']].append({'loc': [constraint['loc'][1]], 'positive': False})
            constraint_table[constraint['timestep']].append({'loc': [constraint['loc'][1], constraint['loc'][0]], 'positive': False})
        else: # negative edge constraint
            constraint_table[constraint['timestep']].append({'loc': constraint['loc'], 'positive': False})

    
    # print(constraint_table)
    return constraint_table

            
    # you can continue this when you come to it

def get_location(path, time):
    if time < 0:
        return path[0]
    elif time < len(path):
        return path[time]
    else:
        return path[-1]  # wait at the goal location


def get_path(goal_node):
    path = []
    curr = goal_node
    while curr is not None:
        path.append(curr['loc'])
        curr = curr['parent']
    path.reverse()
    return path


def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 1.2/1.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    # if len(constraint_table) <= next_time:
    #     return False
    
    # for constraint in constraint_table[next_time]:
    #     # print(constraint, curr_loc, next_loc, next_time)
    #     if constraint['loc'] == [next_loc] or constraint['loc'] == [curr_loc, next_loc]:
    #         return True
    #     else:
    #         if constraint['loc'] == [curr_loc, next_loc]:
    #             return True
        
        
    # return False

    if len(constraint_table) <= next_time:
        return False
    
    for constraint in constraint_table[next_time]:
        if constraint['positive']: # positive constraint
            if constraint['loc'][0] != next_loc:
                return True
        else: # negative constraint
            if len(constraint['loc']) == 1: # vertex constraint
                if constraint['loc'][0] == next_loc:
                    return True
            else: # edge constraint
                if constraint['loc'] == [curr_loc, next_loc]:
                    return True
                
    return False


def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))


def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr


def compare_nodes(n1, n2):
    """Return true is n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']


def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """ my_map      - binary obstacle map
        start_loc   - start position
        goal_loc    - goal position
        agent       - the agent that is being re-planned
        constraints - constraints defining where robot should or cannot go at each timestep
    """

    ##############################
    # Task 1.1: Extend the A* search to search in the space-time domain
    #           rather than space domain, only.

    # print("constraints: ", constraints)
    constraint_table = build_constraint_table(constraints, agent)
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = 0
    h_value = h_values[start_loc]
    root = {'loc': start_loc, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': 0} # Added timestep = 0
    push_node(open_list, root)
    closed_list[((root['loc']),root['timestep'])] = root # Added timestep
    while len(open_list) > 0:
        curr = pop_node(open_list)
        #############################
        # Task 1.4: Adjust the goal test condition to handle goal constraints
        if curr['loc'] == goal_loc and curr['timestep'] >= earliest_goal_timestep:
            found = True
            if curr['timestep'] + 1 < len(constraint_table):
                for t in range(curr['timestep'] + 1, len(constraint_table)):
                    if is_constrained(goal_loc, goal_loc, t, constraint_table):
                        found = False
                        break
            if found:
                return get_path(curr)
        for dir in range(5):
            child_loc = move(curr['loc'], dir)
            if my_map[child_loc[0]][child_loc[1]] or is_constrained(curr['loc'], child_loc, curr['timestep'] + 1, constraint_table):
                continue
            child = {'loc': child_loc,
                    'g_val': curr['g_val'] + 1,
                    'h_val': h_values[child_loc],
                    'parent': curr,
                    'timestep': curr['timestep'] + 1} # Added time
            if ((child['loc']), child['timestep']) in closed_list:
                existing_node = closed_list[((child['loc']),child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[((child['loc']),child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[((child['loc']),child['timestep'])] = child
                push_node(open_list, child)

    return None  # Failed to find solutions
