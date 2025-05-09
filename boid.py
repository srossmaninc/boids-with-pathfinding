import numpy as np
import random
import time
import math

WIDTH = 1920
HEIGHT = 1080

MAX_SPEED = 5
MIN_SPEED = 2

END_GOAL_RADIUS = 30

"""
This program is based off of Craig Reynold's paper
"Flocks, herds and schools: A distributed behavioral model"

I'd like to thank the nice ECE department at Cornell for simplifying scary vector math
https://people.ece.cornell.edu/land/courses/ece4760/labs/s2021/Boids/Boids.html#:~:text=Boids%20is%20an%20artificial%20life,very%20simple%20set%20of%20rules.
"""

class ProblemDef:
    def __init__(self, action_cost):
        self.action_cost = action_cost
        # actions has the form (dx delta, dy delta)
        self.actions = [
            (1, 1), # move right, down
            (1, -1), # move left, up
            (-1, 1), # move left, down
            (-1, -1) # move right, up
        ]

class Boid:
    center_x = 0
    center_y = 0
    xvelocity = 0
    yvelocity = 0

    center_points = []

    # Per the paper,
    #   random heading
    #   random velocity
    def __init__(self, init_x, init_y, boid_id, wall_coords):
        print(f"spawning boid id={boid_id}...")
        self.start_time = time.time()

        self.problem_def = ProblemDef(1)

        self.center_x = init_x
        self.center_y = init_y
        self.boid_id = boid_id

        self.path_pts = [(self.center_x, self.center_y)]

        self.color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))

        self.xvelocity = random.randint(MIN_SPEED, MAX_SPEED) # 5
        self.yvelocity = random.randint(MIN_SPEED, MAX_SPEED) # 5
        self.size = 7

        # 'radius' of sensitivty circle
        self.sense_radius = 80

        self.wall_coords = wall_coords

        self.H = {} # (state (to) cost estimate)
        self.results = {} # ( (s, a) to s' )
        self.current_s = ( init_x, init_y )
        self.prev_s = None # initially set to None
        self.a = (0, 0) # initially set to None


    # # # # # # # # # # # # # # # # # # # # 
    # COLLISION AVOIDANCE
    # # # # # # # # # # # # # # # # # # # # 

    def avoid_walls(self):
        turn_factor = 1
        had_to_avoid = False

        for top_left_xy, width_height in self.wall_coords:
            # print(f"top_left = {top_left}, bottom_right = {bottom_right}")
            horizontal_span = (top_left_xy[0], top_left_xy[0] + width_height[0])
            vertical_span = (top_left_xy[1], top_left_xy[1] + width_height[1])

            # check if we will be in obstacle
            future_xloc = self.center_x + (self.xvelocity*5)
            future_yloc = self.center_y + (self.yvelocity*5)

            if future_xloc >= horizontal_span[0] and future_xloc <= horizontal_span[1] and future_yloc >= vertical_span[0] and future_yloc <= vertical_span[1]:
                # with x range
                if self.center_y < vertical_span[0]:
                    # boid below
                    # print("1")
                    self.yvelocity -= turn_factor
                elif self.center_y > vertical_span[1]:
                    # print("2")
                    self.yvelocity += turn_factor

                if self.center_x < horizontal_span[0]:
                    # print("3")
                    self.xvelocity -= turn_factor
                elif self.center_x > horizontal_span[1]:
                    # print("4")
                    self.xvelocity += turn_factor

                had_to_avoid = True

        return had_to_avoid

        # self.xvelocity += x_accumulate
        # self.yvelocity += y_accumulate

        return True
    
    def avoid_borders(self):

        wall_distance = 80
        turn_factor = 0.5

        # issue with boid going directly straight vertical or horizontal
        if self.yvelocity == 0:
            self.yvelocity -= 1
        if self.xvelocity == 0:
            self.xvelocity += 1

        # left wall
        if self.center_x - wall_distance < 0:
            # rotated 'up' or flat
            self.xvelocity += turn_factor

        # right wall
        if self.center_x + wall_distance >= WIDTH:
            self.xvelocity -= turn_factor

        # top
        if self.center_y + wall_distance >= HEIGHT:
            self.yvelocity -= turn_factor

        # bottom
        if self.center_y - wall_distance < 0:
            self.yvelocity += turn_factor

        return True
    
    def collision_avoidance(self, neighborhood):
        bump_dist = 20
        avoid_force_x = 0
        avoid_force_y = 0

        for neighbor in neighborhood:
            diff_x = self.center_x - neighbor.center_x
            diff_y = self.center_y - neighbor.center_y

            squared_dist = diff_x**2 + diff_y**2

            if squared_dist < bump_dist**2:
                # print("hit!")
                # Generate a force pushing AWAY from the neighbor
                avoid_force_x += diff_x / squared_dist  # Subtract the difference to move in the opposite direction
                avoid_force_y += diff_y / squared_dist

        curr_speed = math.sqrt( self.xvelocity ** 2 + self.yvelocity ** 2)
        self.xvelocity += avoid_force_x *2#* curr_speed
        self.yvelocity += avoid_force_y *2#* curr_speed
    
    # # # # # # # # # # # # # # # # # # # # 
    # FLOCK CENTERING
    # # # # # # # # # # # # # # # # # # # # 

    def stay_centered(self, num_neighbhors, neighbhors_x_sum, neighbhors_y_sum):
        # find average 'center' of flock
        center_flock_x = neighbhors_x_sum / num_neighbhors
        center_flock_y = neighbhors_y_sum / num_neighbhors

        diff_x = center_flock_x - self.center_x
        diff_y = center_flock_y - self.center_y
        # print(f"curr_heading_relative={center_heading_relative} boid_heading={self.heading}")
        # print(f"diff_x {diff_x} diff_y {diff_y}")
        self.center_points.append( (center_flock_x, center_flock_y) )

        return diff_x, diff_y

    # # # # # # # # # # # # # # # # # # # # 
    # MATCH VELOCITY
    # # # # # # # # # # # # # # # # # # # # 

    def match_velocity(self, num_neighbors, xvelocity_sum, yvelocity_sum):
        avg_xvelocity = xvelocity_sum / num_neighbors
        avg_yvelocity = yvelocity_sum / num_neighbors
        return (avg_xvelocity - self.xvelocity), (avg_yvelocity - self.yvelocity)

    # # # # # # # # # # # # # # # # # # # # 
    # A-STAR
    # # # # # # # # # # # # # # # # # # # # 
        # A* base code
        # Q <- ordered list containing just the initial state
        # Loop
        #   if Q is empty then return failure
        #   Node <- Pop(Q)
        #   if Node is a goal,
        #       return Node (or path to it)
        #   else
        #       Children <- Expand(Node)
        #       Merge Children into Q, keep sorted by f(n)
    # RTA* psuedocode
        # keep has table of h values for visited states
        # 1. for each neighbor of current state s
        # 2. either find h in table or do some lookahead
        # 3. add edge cost to get f
        # 4. update h(s) to second-best f value
        # 5. move to best neighbhor
    # LRTA* (LSS-LRTA*)
        # 1. single A* lookahead (LSS)
        # 2. update all h values in LSS
        # 3. move to frontier
        # pseudocode:
        #   1. returns an action
        #   2. persistent -> result (a table mapping (s,a)->s') initially empty
        #      H, a tale mapping s to a cost estimate, initially empty
        # LRTA*-Agent(problem, s', h)
            # if is_goal(s') then return stop
            # if s' is a new state (NOT in H) then H[s'] <- h(s')
            # if s is not null then
            #   result[state, action] <- s'
            #   H[s] <- min( LRTA*-Cost( s, b, result[s, b], H ) ) for b in Action(s)
            # a <- argmin LRTA*-Cost( problem, s', b, result[s', b], H ) for b in actions
            # s <- s'
            # return a
        # LRTA*-Cost(problem, s, a, s', H) returns a cost estimate
        #   if s' is undefinted then return h(s)
        #   else return problem.Action-Cost(s, a, s') + H[s']
        #       ^^^ I will let Action-Cost be equal to 1 (for 1 timestep)

        # | s   |<- previous state
        # | s'  |<- current state
        # | s'' |<- next state
    def lrta_star_agent(self, s, objective_coords):
        # NOTE: define current state here
        #        ^^^ get from above
        prev_a = None

        if self.current_s == objective_coords:
            return ""
        
        if self.H.get(self.current_s) == None:
            self.H[self.current_s] = self.h(self.current_s, objective_coords)
        if self.prev_s != None:
            self.results[(self.prev_s, prev_a)] = self.current_s

            temp_H_val = 10000
            # NOTE: FIX ME!!!
            for b1 in self.problem_def.actions:
                temp_cost = self.lrta_star_cost(self.prev_s, b1, self.current_s, objective_coords)
                if temp_cost < temp_H_val:
                    temp_H_val = temp_cost
            self.H[self.prev_s] = temp_H_val


        # REPALCE EVERYTGHING WITH ITERATOR MAP CALLS



        temp_a_h = 10000
        temp_a = None
        # stuff with s''
        for b2 in self.problem_def.actions:
            applied_b2 = ( self.current_s[0]+b2[0], self.current_s[1]+b2[1] )
            temp_calc = self.lrta_star_cost(self.current_s, b2, applied_b2, objective_coords)
            if temp_calc < temp_a_h:
                temp_a = b2

        self.prev_s = self.current_s

        return temp_a
    
    def h(self, state_in_question, objective_coords):
        return math.sqrt( (objective_coords[0] - state_in_question[0])**2 + (objective_coords[1] - state_in_question[1])**2 )

    def lrta_star_cost(self, s1, a, s2, objective_coords):
        # if the s_prime provided isn't in H yet, return h(s)
        if self.H.get(s2) == None:
            # euclidean distance
            print("using old state")
            return self.h(s1, objective_coords)
        else:
            # for now, MAKE ALL ACTION COSTS 1
            print(f"using new state H val -> {self.H[s2]}")
            return self.problem_def.action_cost + self.H[s2]
    
    # # # # # # # # # # # # # # # # # # # # 
    # FOLLOW MOUSE
    # # # # # # # # # # # # # # # # # # # # 

    def follow_mouse(self, mouse_xy):
        # mouse_diff_x = (mouse_xy[0] - self.center_x) / self.center_x
        # mouse_diff_y = (mouse_xy[1] - self.center_y) / self.center_y
        mouse_scalar = 0.05  # Adjust this value

        mouse_diff_x = mouse_xy[0] - self.center_x
        mouse_diff_y = mouse_xy[1] - self.center_y

        distance_to_mouse_sq = mouse_diff_x**2 + mouse_diff_y**2

        if distance_to_mouse_sq > 0:  # Avoid division by zero if mouse is at the boid's center
            distance_to_mouse = distance_to_mouse_sq**0.5
            direction_x = mouse_diff_x / distance_to_mouse
            direction_y = mouse_diff_y / distance_to_mouse

            xvelocity_delta = mouse_scalar * direction_x
            yvelocity_delta = mouse_scalar * direction_y

            # print(f"mouse_diff_x {xvelocity_delta} mouse_diff_y {yvelocity_delta}")

            self.xvelocity += xvelocity_delta
            self.yvelocity += yvelocity_delta
        return mouse_diff_x, mouse_diff_y


    # # # # # # # # # # # # # # # # # # # # 
    # GET NEIGHBHORS
    # # # # # # # # # # # # # # # # # # # # 

    def get_neighbhors(self, boids):
        neighbors = []
        for b in boids:
            if b.boid_id != self.boid_id:
                dist_squared = (self.center_x - b.center_x)**2 + (self.center_y - b.center_y)**2
                if dist_squared < self.sense_radius**2:
                    neighbors.append( (b) )
        return neighbors

    # # # # # # # # # # # # # # # # # # # # 
    # PILOT
    # # # # # # # # # # # # # # # # # # # # 

    def pilot(self, boids, mouse_xy, objective_coords):

        # findining boid neighbhors
        neighbhors = self.get_neighbhors(boids=boids)
        # accumulation vals
        num_neighbhors = len(neighbhors)

        neighbhors_x_sum = 0
        neighbhors_y_sum = 0

        xvelocity_sum = 0
        yvelocity_sum = 0

        for neighbor in neighbhors:
            neighbhors_x_sum += neighbor.center_x
            neighbhors_y_sum += neighbor.center_y
            xvelocity_sum += neighbor.xvelocity
            yvelocity_sum += neighbor.yvelocity

        # NOTE: PRECENDENCE LEVELS
        mouse_scalar = 1
        # mouse_diff_x, mouse_diff_y = self.follow_mouse(mouse_xy)
        # mouse_diff_x, mouse_diff_y = 0, 0

        # self.xvelocity += mouse_scalar*mouse_diff_x
        # self.yvelocity += mouse_scalar*mouse_diff_y


        # wall avoidance
        had_to_avoid = self.avoid_walls()
        self.avoid_borders()


        if num_neighbhors > 0:
            # self.xvelocity *= mouse_diff_x
            # self.xvelocity *= mouse_diff_y

            self.collision_avoidance(neighborhood=neighbhors)

            # if we had to avoid a wall, skip the rest as it could make boids 'clip' through walls

            if not had_to_avoid:

                # flock centering
                centering_scalar = 0.0002
                diff_x1, diff_y1 = self.stay_centered(num_neighbhors=num_neighbhors, neighbhors_x_sum=neighbhors_x_sum, neighbhors_y_sum=neighbhors_y_sum)

                # velocity matching
                match_v_scalar = 0.2
                diff_vx, diff_vy = self.match_velocity(num_neighbhors, xvelocity_sum=xvelocity_sum, yvelocity_sum=yvelocity_sum)

                # # with mouse
                # self.xvelocity += mouse_scalar*mouse_diff_x + centering_scalar*diff_x1 + match_v_scalar*diff_vx
                # self.yvelocity += mouse_scalar*mouse_diff_y + centering_scalar*diff_y1 + match_v_scalar*diff_vy
                # without
                self.xvelocity += centering_scalar*diff_x1 + match_v_scalar*diff_vx
                self.yvelocity += centering_scalar*diff_y1 + match_v_scalar*diff_vy

        lrta_next_act = self.lrta_star_agent(self.prev_s, objective_coords)
        lrta_scalar = 0.01
        self.a = lrta_next_act
        self.xvelocity += lrta_scalar*lrta_next_act[0]
        self.yvelocity += lrta_scalar*lrta_next_act[1]
                

            # print(f"diff_x1 {diff_x1} diff_y1 {diff_y1}")
            # print(f"diff vx {diff_vx} diff_vy {diff_vy}")
            # print("------------")

        # collision avoidance

        speed = math.sqrt(self.xvelocity**2 + self.yvelocity**2)

        max_speed = MAX_SPEED # velocity of 10
        min_speed = MIN_SPEED
        if speed > max_speed:
            self.xvelocity = (self.xvelocity/speed) * MAX_SPEED
            self.yvelocity = (self.yvelocity/speed) * MAX_SPEED
        elif speed < min_speed:
            self.xvelocity = (self.xvelocity/speed) * MIN_SPEED
            self.yvelocity = (self.yvelocity/speed) * MIN_SPEED


        self.center_x += self.xvelocity
        self.center_y += self.yvelocity

        self.current_s = ( self.center_x, self.center_y )



        self.path_pts.append((self.center_x, self.center_y))

        if self.check_course_complete(objective_coordx=objective_coords[0], objective_coordy=objective_coords[1]):
            return None
        else:
            return self
    
    def check_course_complete(self, objective_coordx, objective_coordy):
        # checks if a boid is touching 'end' green circle
        # NOTE: euclid distance isn't square rooted for potential speedup
        euclid_dist = (objective_coordx - self.center_x)**2 + (objective_coordy - self.center_y)**2
        if euclid_dist < END_GOAL_RADIUS**2:
            print(f"boid id={self.boid_id} has completed course in {time.time() - self.start_time}!")
            return True
        else:
            return False

    def get_boid(self):
        # Define triangle relative to center as a NumPy array
        points = np.array([
            [self.size, 0],
            [-self.size, -self.size + 4],
            [-self.size, self.size - 4]
        ])

        # Calculate rotation angle using NumPy's atan2
        rad = np.arctan2(self.yvelocity, self.xvelocity)
        cos_rad = np.cos(rad)
        sin_rad = np.sin(rad)

        # Create rotation matrix
        rotation_matrix = np.array([
            [cos_rad, sin_rad],
            [-sin_rad, cos_rad]
        ])

        # Rotate all points at once using matrix multiplication
        rotated_points_relative = points @ rotation_matrix

        # Translate points to the boid's center
        rotated_points = rotated_points_relative + np.array([self.center_x, self.center_y])

        path_to_draw = list(reversed(self.path_pts))[:100]

        return rotated_points.tolist(), self.center_x, self.center_y, self.sense_radius, self.color, path_to_draw