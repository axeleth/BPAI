import time as timer
from single_agent_planner import compute_heuristics, a_star, get_sum_of_cost


class PrioritizedPlanningSolver(object):
    """A planner that plans for each robot sequentially."""

    def __init__(self, my_map, starts, goals):
        """my_map   - list of lists specifying obstacle positions
        starts      - [(x1, y1), (x2, y2), ...] list of start locations
        goals       - [(x1, y1), (x2, y2), ...] list of goal locations
        """

        self.my_map = my_map
        self.starts = starts
        self.goals = goals
        self.num_of_agents = len(goals)

        self.CPU_time = 0

        # compute heuristics for the low-level search
        self.heuristics = []
        for goal in self.goals:
            self.heuristics.append(compute_heuristics(my_map, goal))

    def find_solution(self):
        """ Finds paths for all agents from their start locations to their goal locations."""

        start_time = timer.time()
        result = []
        constraints = []
        max_path_length = 0

        for i in range(self.num_of_agents):  # Find path for each agent
            upper_bound = len(self.my_map) * len(self.my_map[0]) + max_path_length
            print("upper bound: ", upper_bound)
            path = a_star(self.my_map, self.starts[i], self.goals[i], self.heuristics[i],
                          i, constraints)
            print("path length: ", len(path))
            if path is None or len(path)>=upper_bound:
                raise BaseException('No solutions')
            result.append(path)
            max_path_length = max(max_path_length, len(path))

            ##############################
            # Task 2: Add constraints here
            #         Useful variables:
            #            * path contains the solution path of the current (i'th) agent, e.g., [(1,1),(1,2),(1,3)]
            #            * self.num_of_agents has the number of total agents
            #            * constraints: array of constraints to consider for future A* searches

            for t in range(len(path)):
                for k in range(i, self.num_of_agents):
                    # add vertex constraint
                    constraints.append({'agent': k,
                                        'loc': [path[t]],
                                        'timestep': t,
                                        'positive': False})
                    # add edge constraint
                    if t > 0:
                        constraints.append({'agent': k,
                                            'loc': [path[t], path[t - 1]],
                                            'timestep': t,
                                            'positive': False})

            # upper bound T=height*width+max(path_length)
            for t in range(len(path), upper_bound+1):
                for k in range(i, self.num_of_agents):
                    # add vertex constraint
                    constraints.append({'agent': k,
                                        'loc': [path[-1]],
                                        'timestep': t,
                                        'positive': False})

            ##############################

        self.CPU_time = timer.time() - start_time

        print("\n Found a solution! \n")
        print("CPU time (s):    {:.2f}".format(self.CPU_time))
        print("Sum of costs:    {}".format(get_sum_of_cost(result)))
        print(result)
        return result
