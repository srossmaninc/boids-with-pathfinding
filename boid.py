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

class State:
    nav_mesh_interval = 5

    def __init__(self, xcoord, ycoord, direction, xvelocity, yvelocity, action_to ):
        self.x = xcoord
        self.y = ycoord
        self.xvelocity = xvelocity
        self.yvelocity = yvelocity
        self.direction = direction
        self.action_to = action_to

        self.x_nav = int(xcoord / self.nav_mesh_interval)
        self.y_nav = int(ycoord / self.nav_mesh_interval)

    def get_dir_string(self, direction):
        if direction == 1:
            return "headed bottom-right"
        elif direction == 2:
            return "headed bottom-left"
        elif direction == 3:
            return "headed top-left"
        elif direction == 4:
            return "headed top-right"
        else:
            print("why are you here?")

    def __eq__(self, other):
        return (
            isinstance(other, State) and
            self.x_nav == other.x_nav and
            self.y_nav == other.y_nav and
            self.direction == other.direction
        )

    def __hash__(self):
        return hash((self.x_nav, self.y_nav, self.direction))
    
    def __str__(self):
        return f"x={self.x_nav}, y={self.y_nav}, direction={self.get_dir_string(self.direction)}"

class Boid:
    center_x = 0
    center_y = 0
    xvelocity = 0
    yvelocity = 0

    # Per the paper,
    #   random heading
    #   random velocity
    def __init__(self, init_x, init_y, boid_id, wall_coords):
        print(f"spawning boid id={boid_id}...")
        self.start_time = time.time()

        # self.actions = [
        #     (0.9, 1), # right
        #     (1.1, 1), # left
        #     (1, 0.9), # down
        #     (1, 1.1) # up
        # ]
        self.actions = [0, 45, -45]

        self.center_x = init_x
        self.center_y = init_y
        self.boid_id = boid_id

        self.path_pts = [(self.center_x, self.center_y)]

        self.color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))

        self.xvelocity = 3#random.randint(MIN_SPEED, MAX_SPEED) # 5
        self.yvelocity = 3#random.randint(MIN_SPEED, MAX_SPEED) # 5
        self.size = 7

        # 'radius' of sensitivty circle
        self.sense_radius = 80

        self.wall_coords = wall_coords

        self.H = {} # (state (to) cost estimate)
        self.results = {} # ( (s, a) to s' )
        self.current_s = State(init_x, init_y, self.get_dir(self.xvelocity, self.yvelocity), self.xvelocity, self.yvelocity, None )
        self.prev_s = None # initially set to None
        self.a = (0, 0) # initially set to None


    # # # # # # # # # # # # # # # # # # # # 
    # COLLISION AVOIDANCE
    # # # # # # # # # # # # # # # # # # # # 

    def avoid_walls(self):
        turn_factor = 2
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

        return diff_x, diff_y

    # # # # # # # # # # # # # # # # # # # # 
    # MATCH VELOCITY
    # # # # # # # # # # # # # # # # # # # # 

    def match_velocity(self, num_neighbors, xvelocity_sum, yvelocity_sum):
        avg_xvelocity = xvelocity_sum / num_neighbors
        avg_yvelocity = yvelocity_sum / num_neighbors
        return (avg_xvelocity - self.xvelocity), (avg_yvelocity - self.yvelocity)

    # # # # # # # # # # # # # # # # # # # # 
    # GET DIRECTION FROM VELOCITIES
    # # # # # # # # # # # # # # # # # # # # 
    def get_dir(self, xv, yv):
        if xv > 0 and yv > 0:
            # headed bottom-right
            return 1
        elif xv < 0 and yv > 0:
            # headed bottom-left
            return 2
        elif xv < 0 and yv < 0:
            # headed top-left
            return 3
        elif xv > 0 and yv < 0:
            # headed top-right
            return 4
        print("shouldn't be here!")

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


        # REPALCE EVERYTGHING WITH ITERATOR MAP CALLS


    def rotate_velocity_2d(self, vx, vy, angle_degrees):
        angle_radians = np.deg2rad(angle_degrees)
        cos_theta = np.cos(angle_radians)
        sin_theta = np.sin(angle_radians)

        vx_rotated = vx * cos_theta - vy * sin_theta
        vy_rotated = vx * sin_theta + vy * cos_theta

        return vx_rotated, vy_rotated

    def lrta_star_agent(self, objective_coords):

        if (self.current_s.x, self.current_s.y) == objective_coords:
            return ""
        
        if self.current_s not in self.H:

            # IMPLEMENTING COUNTER!
            self.H[self.current_s] = (self.h(self.current_s, objective_coords), 0)

        if self.prev_s != None:
            self.results[(self.prev_s, self.a)] = self.current_s

            best_cost = float('inf')
            for b_theta1 in self.actions:
                proposed_rotated_vec = self.rotate_velocity_2d(self.prev_s.xvelocity, self.prev_s.yvelocity, b_theta1)
                new_pos_x = self.prev_s.x + proposed_rotated_vec[0]
                new_pos_y = self.prev_s.y + proposed_rotated_vec[1]
                new_dir = self.get_dir(proposed_rotated_vec[0], proposed_rotated_vec[1])
                
                applied_b1 = State( new_pos_x, new_pos_y, new_dir, proposed_rotated_vec[0], proposed_rotated_vec[1], b_theta1 )

                self.results[(self.prev_s, b_theta1)] = applied_b1

                temp_cost = self.lrta_star_cost(self.prev_s, self.results[(self.prev_s, b_theta1)], objective_coords, 0)

                if temp_cost < best_cost:
                    best_cost = temp_cost
            self.H[self.prev_s] = (best_cost, self.H[self.prev_s][1] + 1)

        best_cost2 = float('inf')
        best_action = None
        # stuff with s'' (next state)
        for b_theta2 in self.actions:
            proposed_rotated_vec2 = self.rotate_velocity_2d(self.current_s.xvelocity, self.current_s.yvelocity, b_theta2)
            new_pos_x2 = self.current_s.x + proposed_rotated_vec2[0]
            new_pos_y2 = self.current_s.y + proposed_rotated_vec2[1]
            new_dir2 = self.get_dir(proposed_rotated_vec2[0], proposed_rotated_vec2[1])

            applied_b2 = State( new_pos_x2, new_pos_y2, new_dir2, proposed_rotated_vec2[0], proposed_rotated_vec2[1], b_theta2 )
            self.results[(self.current_s, b_theta2)] = applied_b2

            temp_calc = self.lrta_star_cost(self.current_s, self.results[(self.current_s, b_theta2)], objective_coords, 1)

            # if temp_calc == best_cost2:
            #     print(f"tie at action {best_action} and {b_theta2}")

            if temp_calc < best_cost2:
                best_cost2 = temp_calc
                best_action = b_theta2

        if self.prev_s != None and self.H[self.prev_s][1] > 1:
            print(f"updated best action from {self.prev_s.action_to} -> {self.current_s.action_to}")
        self.prev_s = self.current_s

        # print(f"chosen action -> {best_action}")

        return best_action

    def h1(self, state_in_question, objective_coords):
        dx = (objective_coords[0] - (state_in_question.x))
        dy = (objective_coords[1] - (state_in_question.y))
        # return math.sqrt( dx**2 + dy**2 )
        return abs(dx) + abs(dy)
    
    def h(self, state_in_question, objective_coords):
        dx = (objective_coords[0] - (state_in_question.x))
        dy = (objective_coords[1] - (state_in_question.y))
        return math.sqrt( dx**2 + dy**2 )

    def lrta_star_cost(self, s1, s2, objective_coords, which):
        # if the s_prime provided isn't in H yet, return h(s)
        if s2 not in self.H:
            # euclidean distance
            # print(f"not in H -> {s2}")
            return self.h(s2, objective_coords)
        else:
            # for now, MAKE ALL ACTION COSTS 1
            if self.H[s2][1] > 0:
                print(f"{self.H[s2]} <- {s2}")
            return 20 + self.H[s2][0]


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

    def pilot(self, boids, objective_coords):

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


        # wall avoidance
        had_to_avoid = self.avoid_walls()
        self.avoid_borders()


        if num_neighbhors > 0:

            self.collision_avoidance(neighborhood=neighbhors)

            # if we had to avoid a wall, skip the rest as it could make boids 'clip' through walls

            if not had_to_avoid:

                # flock centering
                centering_scalar = 0.0002
                diff_x1, diff_y1 = self.stay_centered(num_neighbhors=num_neighbhors, neighbhors_x_sum=neighbhors_x_sum, neighbhors_y_sum=neighbhors_y_sum)

                # velocity matching
                match_v_scalar = 0.2
                diff_vx, diff_vy = self.match_velocity(num_neighbhors, xvelocity_sum=xvelocity_sum, yvelocity_sum=yvelocity_sum)

                self.xvelocity += centering_scalar*diff_x1 + match_v_scalar*diff_vx
                self.yvelocity += centering_scalar*diff_y1 + match_v_scalar*diff_vy

        lrta_next_act_theta = self.lrta_star_agent(objective_coords)
        lrta_scalar = 0.01
        self.a = lrta_next_act_theta
        res = self.rotate_velocity_2d(self.xvelocity, self.yvelocity, lrta_next_act_theta)
        self.xvelocity += lrta_scalar*res[0]
        self.yvelocity += lrta_scalar*res[1]
                

        # print(self.h((self.center_x, self.center_y), objective_coords))

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

        # a state is defined in multiples of 20 so,
        self.current_s = State( self.center_x, self.center_y, self.get_dir(self.xvelocity, self.yvelocity), self.xvelocity, self.yvelocity, lrta_next_act_theta )


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
            for k, v in self.H.items():
                if v[1] > 1:
                    print(f"state {k} was visited {v[1]} times")
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

        # path_to_draw = list(reversed(self.path_pts))[:100]
        path_to_draw = self.path_pts

        return rotated_points.tolist(), self.center_x, self.center_y, self.sense_radius, self.color, path_to_draw