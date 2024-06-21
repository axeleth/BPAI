from single_agent_planner import simple_single_agent_astar
from deviate import is_it_a_node
import math

class Aircraft(object):
    """Aircraft class, should be used in the creation of new aircraft."""

    def __init__(self, flight_id, a_d, profile, start_node, goal_node, spawn_time, nodes_dict):
        """
        Initalisation of aircraft object.
        INPUT:
            - flight_id: [int] unique id for this aircraft
            - a_d: [str] "a" for arrival flight and "d" for departure flight
            - start_node: node_id of start node
            - goal_node: node_id of goal node
            - spawn_time: spawn_time of a/c 
            - nodes_dict: copy of the nodes_dic
        """
        # Deviation related (placed up here because it needs to be initialised before speed)
        self.profile = profile              # the speed profile of the agent: determines deviation behaviour.
        
        #Fixed parameters
        self.speed = self.init_speed()      #how much a/c moves per unit of t; Determined by speed profile
        self.id = flight_id                 #flight_id
        self.type = a_d                     #arrival or departure (A/D)
        self.spawntime = spawn_time         #spawntime
        self.start = start_node             #start_node_id
        self.goal = goal_node               #goal_node_id
        self.nodes_dict = nodes_dict        #keep copy of nodes dict
        
        #Route related
        self.status = None 
        self.path_to_goal = []              #planned path left from current location
        self.from_to = [0,0]
        self.path_so_far = []               #path that has been taken so far

        #State related
        self.heading = 0
        self.position = (0,0)               #xy position on map
        
        # Deivation related
        self.deviated = False               #has this agent deviated from its path?
        self.deviation_type = None          #what type of deviation has this agent experienced?
        self.waiting = False                # is the agent waiting for a slot?
        self.distance_travelled = 0 
        
        # Replanning related –––– OBS; not used anymore!
        self.replanning = False             # is this agent replanning or spawning?
        self.replan_time = None             # what time does the agent plan start?

    def init_speed(self):
        """
        INPUT:
            - self = Aircraft object
        RETURNS:
            - speed = initial speed of aircraft based on profile
        """

        if self.profile == "cS":        return 1 # Start slow
        elif self.profile == "fS":      return 2 # Start fast
        elif self.profile == "cE":      return 2 # ––––||––––
        
        elif self.profile == "fast":    return 2 # Fast the whole time
        elif self.profile == "slow":    return 1 # Slow the whole time
        

        else: raise Exception("Invalid speed profile '{}' for AC{}".format(self.profile, self.id))
        

    def get_heading(self, xy_start, xy_next):
        """
        Determines heading of an aircraft based on a start and end xy position.
        INPUT:
            - xy_start = tuple with (x,y) position of start node
            - xy_next = typle with (x,y) position of next node
        RETURNS:
            - heading = heading of aircraft in degrees
        """

        if xy_start[0] == xy_next[0]: #moving up or down
            if xy_start[1] > xy_next[1]: #moving down
                heading = 180
            elif xy_start[1] < xy_next[1]: #moving up
                heading = 0
            else:
                heading=self.heading

        elif xy_start[1] == xy_next[1]: #moving right or left
            if xy_start[0] > xy_next[0]: #moving left
                heading = 90
            elif xy_start[0] < xy_next[0]: #moving right
                heading = 270
            else:
                heading=self.heading
        else: 
            raise Exception("Invalid movement")
    
        self.heading = heading
      
    def move(self, dt, t):   
        """
        Moves an aircraft between from_node and to_node and checks if to_node or goal is reached.
        INPUT:
            - dt = 
            - t = 
        """
        
        #Determine nodes between which the ac is moving
        from_node = self.from_to[0]
        to_node = self.from_to[1]
        xy_from = self.nodes_dict[from_node]["xy_pos"] #xy position of from node
        xy_to = self.nodes_dict[to_node]["xy_pos"] #xy position of to node
        distance_to_move = self.speed*dt #distance to move in this timestep
    
        #Update position with rounded values
        x = xy_to[0]-xy_from[0]
        y = xy_to[1]-xy_from[1]

        if x == 0 and y!=0:
            x_normalized = 0
            y_normalized = y / math.sqrt(x ** 2 + y ** 2)
            self.waiting = False
        elif x == 0 and y==0: # stand still
            x_normalized = 0
            y_normalized = 0
            self.waiting = True
        elif x!=0 and y==0:
            x_normalized = x / math.sqrt(x ** 2 + y ** 2)
            y_normalized = 0
            self.waiting = False
        elif x!=0 and y!=0:
            x_normalized = x / math.sqrt(x ** 2 + y ** 2)
            y_normalized = y / math.sqrt(x ** 2 + y ** 2)
            self.waiting = False

        distance_to_next_node = round(math.sqrt((xy_to[0] - self.position[0])**2 + (xy_to[1] - self.position[1])**2),2)

        posx = round(self.position[0] + x_normalized * distance_to_move ,2) #round to prevent errors –––––> Current position + unit vector * distance of next step
        posy = round(self.position[1] + y_normalized * distance_to_move ,2) #round to prevent errors
        # if self.id == 2: print("AC{} location just before step: {}".format(self.id, self.position))
        self.position = (posx, posy)  
        self.get_heading(xy_from, xy_to)	

        # if self.id == 2: print("AC2 position: ", self.position, " xy_to: ", xy_to, " path_to_goal: ", self.path_to_goal[0], " t: ", t, " dt: ", dt)
        if self.position == xy_to and self.path_to_goal[0][1] == t+dt: #If with this move its current to node is reached
            # print(" ")
            # print("AC: ", self.id, " just stepped to node: ", self.path_to_goal[0][0], " (note that the time has not been updated yet and is still: ", t, ")")
            # print("position: ", self.position)
            # print(" ")
            if self.position == self.nodes_dict[self.goal]["xy_pos"]: #if the final goal is reached
                self.status = "arrived"

            else:  #current to_node is reached, update the remaining path
                remaining_path = self.path_to_goal
                self.path_to_goal = remaining_path[1:]
                
                new_from_id = self.from_to[1] #new from node
                new_next_id = self.path_to_goal[0][0] #new to node

                if new_from_id != self.from_to[0]:
                    self.last_node = self.from_to[0]
                
                self.from_to = [new_from_id, new_next_id] #update new from and to node

                self.distance_travelled += 1


        # elif detect_deviation(self.position, xy_to, self.path_to_goal, t, dt):
        #     raise Exception("Aircraft deviated from path")
        
    def plan_independent(self, nodes_dict, edges_dict, heuristics, t):
        """
        Plans a path for taxiing aircraft assuming that it knows the entire layout.
        Other traffic is not taken into account.
        INPUT:
            - nodes_dict: copy of the nodes_dict
            - edges_dict: edges_dict with current edge weights
        """

        if self.status == "taxiing": # Planning only occurs when an aircraft is in the taxiing state.
            if self.replanning == False: # if this is the first time planning
            

                start_node = self.start #node from which planning should be done (just the node id)
                goal_node = self.goal #node to which planning should be done (just the node id)
                agent = self.id
                constraints = []
                
                success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t, agent, self.speed,
                                                          constraints) 
                if success:
                    self.path_to_goal = path[1:]
                    next_node_id = self.path_to_goal[0][0] #next node is first node in path_to_goal
                    self.from_to = [path[0][0], next_node_id]
                    # print("Path AC", self.id, ":", path)
                else:
                    raise Exception("No solution found for", self.id)
                
                #Check the path
                if path[0][1] != t:
                    raise Exception("Something is wrong with the timing of the path planning")
        
            elif self.replanning == True: # if this planning is as a result of deviation
                if t == self.replan_time: # if the agent is at a node. (This is the time to start the new plan from)
                    print("AC",self.id, ": t == replan_time")
                    current_node_id = is_it_a_node(self.position, nodes_dict, retrieve_node=True)
                    print("from_to[1]: ", self.from_to[1], "curr_node_id: ", current_node_id)
                    # start_node = self.from_to[1] # this needs to be exactly the node that the next node
                    start_node = current_node_id
                    goal_node = self.goal

                    success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, t, self.speed) # I added speed but this is not being used yet in the function
                    if success:
                        self.path_to_goal = path[1:] # The path EXCLUDES the first (current) node
                        next_node_id = self.path_to_goal[0][0] #next node is first node in path_to_goal
                        self.from_to = [path[0][0], next_node_id]
                        print("Path AC", self.id, ":", path)
                    else:
                        raise Exception("No solution found for", self.id)

                elif t != self.replan_time: # if the agent is between nodes. This is the time to start the new plan from
                    print("AC",self.id, ": t != replan_time")
                    old_start = self.from_to[0]
                    start_node = self.from_to[1]
                    goal_node = self.goal

                    print("AC{} in the middle has replan time {}".format(self.id, self.replan_time))

                    success, path = simple_single_agent_astar(nodes_dict, start_node, goal_node, heuristics, self.replan_time, self.speed) # I added speed but this is not being used yet in the function
                    if success:
                        self.path_to_goal = path # The path INCLUDES the first (next) node
                        next_node_id = self.path_to_goal[0][0] #next node is first node in path_to_goal
                        self.from_to = [old_start, next_node_id]
                        print("Path AC", self.id, ":", path)
                    else:
                        raise Exception("No solution found for", self.id)
                


    


                
