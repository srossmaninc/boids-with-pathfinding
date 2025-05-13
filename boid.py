import numpy as np
import random
import time
import math

WIDTH = 1920
HEIGHT = 1080

MAX_SPEED = 5
MIN_SPEED = 2

END_GOAL_RADIUS = 50

"""
This program is based off of Craig Reynold's paper
"Flocks, herds and schools: A distributed behavioral model"

I'd like to thank the nice ECE department at Cornell for simplifying scary vector math
https://people.ece.cornell.edu/land/courses/ece4760/labs/s2021/Boids/Boids.html#:~:text=Boids%20is%20an%20artificial%20life,very%20simple%20set%20of%20rules.
"""

class State:
    nav_mesh_interval = 50

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

    def string_stats(self):
        return f"({self.x_nav}, {self.y_nav})"

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
        return f"x={self.x_nav}, y={self.y_nav}"
    
class H_Entry:
    def __init__(self, h_val, counter):
        self.h_val = h_val
        self.counter = counter

    def string_stats(self):
        return f"{self.h_val} {self.counter}"

    def __str__(self):
        return f"{self.h_val} was updated {self.counter} times"

class Boid:
    center_x = 0
    center_y = 0
    xvelocity = 0
    yvelocity = 0


    def __init__(self, init_x, init_y, boid_id, wall_coords):
        print(f"spawning boid id={boid_id}...")
        self.start_time = time.time()

        self.actions = [45, -45, 0, 90, -90, 135, -135]

        self.center_x = init_x
        self.center_y = init_y
        self.boid_id = boid_id

        self.path_pts = [(self.center_x, self.center_y)]

        self.color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))

        self.xvelocity = 4#random.randint(MIN_SPEED, MAX_SPEED) # 5
        self.yvelocity = 4#random.randint(MIN_SPEED, MAX_SPEED) # 5
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
        turn_factor = 1
        had_to_avoid = False


        # issue with boid going directly straight vertical or horizontal
        vel_check_x = round(self.xvelocity)
        vel_check_y = round(self.yvelocity)
        if vel_check_y == 0:
            if self.yvelocity < 0:
                self.yvelocity *= 1.1
            else:
                self.yvelocity *= 1.1
        if vel_check_x == 0:
            if self.xvelocity < 0:
                self.xvelocity *= 1.1
            else:
                self.xvelocity *= 1.1

        # check if we will be in obstacle
        future_xloc = self.center_x + (self.xvelocity*MAX_SPEED)
        future_yloc = self.center_y + (self.yvelocity*MAX_SPEED)

        x_acummulate = 0
        y_accumulate = 0

        for (x, y), (w, h) in self.wall_coords:
            horizontal_span = (x, x + w)
            vertical_span = (y, y + h)

            if future_xloc >= horizontal_span[0] and future_xloc <= horizontal_span[1] and future_yloc >= vertical_span[0] and future_yloc <= vertical_span[1]:
                # with x range
                if self.center_y <= vertical_span[0]:
                    # boid below
                    # print("1")
                    y_accumulate -= turn_factor
                elif self.center_y >= vertical_span[1]:
                    # print("2")
                    y_accumulate += turn_factor

                if self.center_x <= horizontal_span[0]:
                    # print("3")
                    x_acummulate -= turn_factor
                elif self.center_x >= horizontal_span[1]:
                    # print("4")
                    x_acummulate += turn_factor

                had_to_avoid = True

        self.xvelocity += x_acummulate
        self.yvelocity += y_accumulate
        return had_to_avoid
    
    
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
    def get_dir(self, xv, yv, num_directions=6):
        angle = math.atan2(yv, xv)
        angle_deg = (math.degrees(angle) + 360) % 360  # normalize to [0, 360)

        bucket_size = 360 / num_directions
        direction_id = int(angle_deg // bucket_size)
        return direction_id  # range: [0, num_directions - 1]


    def rotate_velocity_2d(self, vx, vy, angle_degrees):
        angle_radians = np.deg2rad(angle_degrees)
        cos_theta = np.cos(angle_radians)
        sin_theta = np.sin(angle_radians)

        vx_rotated = vx * cos_theta - vy * sin_theta
        vy_rotated = vx * sin_theta + vy * cos_theta

        return vx_rotated, vy_rotated
    
    def lrta_wall_check(self, applied_s):
        # if we are in a wall, this is an awful path so return infinity
        for (x, y), (w, h) in self.wall_coords:
            horizontal_span = (x, x + w)
            vertical_span = (y, y + h)

            if applied_s.x >= horizontal_span[0] and applied_s.x <= horizontal_span[1] and applied_s.y >= vertical_span[0] and applied_s.y <= vertical_span[1]:
                return True
        return False

    def lrta_star_agent(self, objective_coords, num_neighbors):

        if (self.current_s.x, self.current_s.y) == objective_coords:
            return ""
        
        if self.current_s not in self.H:

            self.H[self.current_s] = H_Entry(self.h(self.current_s, objective_coords), 0)

        if self.prev_s != None:
            self.results[(self.prev_s, self.a)] = self.current_s

            best_cost = float('inf')
            for b_theta1 in self.actions:
                proposed_rotated_vec = self.rotate_velocity_2d(self.prev_s.xvelocity, self.prev_s.yvelocity, b_theta1)
                new_pos_x = self.prev_s.x + proposed_rotated_vec[0]
                new_pos_y = self.prev_s.y + proposed_rotated_vec[1]
                new_dir = self.get_dir(proposed_rotated_vec[0], proposed_rotated_vec[1])
                
                applied_b1 = State( new_pos_x, new_pos_y, new_dir, proposed_rotated_vec[0], proposed_rotated_vec[1], b_theta1 )
                if not self.lrta_wall_check(applied_b1):

                    self.results[(self.prev_s, b_theta1)] = applied_b1

                    visit_penalty = 0
                    if applied_b1 in self.H:
                        visit_penalty = self.H[applied_b1].counter*50*(num_neighbors+1)

                    temp_cost = self.lrta_star_cost(self.results[(self.prev_s, b_theta1)], objective_coords) + visit_penalty

                    if temp_cost < best_cost:
                        best_cost = temp_cost
            self.H[self.prev_s].h_val = best_cost
            self.H[self.prev_s].counter += 1

        best_cost2 = float('inf')
        best_action = None
        # stuff with s'' (next state)
        for b_theta2 in self.actions:
            proposed_rotated_vec2 = self.rotate_velocity_2d(self.current_s.xvelocity, self.current_s.yvelocity, b_theta2)
            new_pos_x2 = self.current_s.x + proposed_rotated_vec2[0]
            new_pos_y2 = self.current_s.y + proposed_rotated_vec2[1]
            new_dir2 = self.get_dir(proposed_rotated_vec2[0], proposed_rotated_vec2[1])

            applied_b2 = State( new_pos_x2, new_pos_y2, new_dir2, proposed_rotated_vec2[0], proposed_rotated_vec2[1], b_theta2 )

            if not self.lrta_wall_check(applied_b2):

                self.results[(self.current_s, b_theta2)] = applied_b2

                temp_calc = self.lrta_star_cost(self.results[(self.current_s, b_theta2)], objective_coords)

                if temp_calc == best_cost2:
                    if best_action == self.current_s.action_to:
                        best_action = b_theta2

                if temp_calc < best_cost2:
                    best_cost2 = temp_calc
                    best_action = b_theta2

        self.prev_s = self.current_s

        if best_action == None:
            best_action = 180

        return best_action

    def h(self, state_in_question, objective_coords):
        dx = (objective_coords[0] - (state_in_question.x))
        dy = (objective_coords[1] - (state_in_question.y))
        return abs(dx) + abs(dy)
    
    def h1(self, state_in_question, objective_coords):
        dx = (objective_coords[0] - (state_in_question.x))
        dy = (objective_coords[1] - (state_in_question.y))
        return math.sqrt( dx**2 + dy**2 )

    def lrta_star_cost(self, s2, objective_coords):
            
        # if the s_prime provided isn't in H yet, return h(s)
        if s2 not in self.H:
            # euclidean distance
            return self.h(s2, objective_coords)
        else:
            return 1 + self.H[s2].h_val


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

        # print(f"xv = {self.xvelocity} yv = {self.yvelocity}")

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

        velocity_adjustment_x = 0
        velocity_adjustment_y = 0


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

                velocity_adjustment_x += centering_scalar*diff_x1 + match_v_scalar*diff_vx
                velocity_adjustment_y += centering_scalar*diff_y1 + match_v_scalar*diff_vy
                

        if not had_to_avoid:
            lrta_next_act_theta = self.lrta_star_agent(objective_coords, num_neighbhors)
            lrta_scalar = 0.01
            # if num_neighbhors == 0:
            #     lrta_scalar = 0.01
            # else:
            #     lrta_scalar = 0.005 * num_neighbhors
            self.a = lrta_next_act_theta
            res = self.rotate_velocity_2d(self.xvelocity, self.yvelocity, lrta_next_act_theta)
            velocity_adjustment_x += lrta_scalar*res[0]
            velocity_adjustment_y += lrta_scalar*res[1]

        self.xvelocity += velocity_adjustment_x
        self.yvelocity += velocity_adjustment_y

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

        self.avoid_walls()

        self.center_x += self.xvelocity
        self.center_y += self.yvelocity

        if not had_to_avoid:
            # a state is defined in multiples of 20 so,
            self.current_s = State( self.center_x, self.center_y, self.get_dir(self.xvelocity, self.yvelocity), self.xvelocity, self.yvelocity, lrta_next_act_theta )


        self.path_pts.append((self.center_x, self.center_y))

        # path_to_draw = list(reversed(self.path_pts))[:100]
        path_to_draw = self.path_pts

        if self.check_course_complete(objective_coordx=objective_coords[0], objective_coordy=objective_coords[1]):
            return None, path_to_draw, self.color
        else:
            return self, path_to_draw, self.color
    
    def check_course_complete(self, objective_coordx, objective_coordy):
        # checks if a boid is touching 'end' green circle
        # NOTE: euclid distance isn't square rooted for potential speedup
        euclid_dist = (objective_coordx - self.center_x)**2 + (objective_coordy - self.center_y)**2
        if euclid_dist < END_GOAL_RADIUS**2:
            print(f"boid id={self.boid_id} has completed course in {time.time() - self.start_time}!")
            for k, v in self.H.items():
                if v.counter > 1:
                    print(f"{k.string_stats()} was visited {v.string_stats()} times")
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

        return rotated_points.tolist(), self.center_x, self.center_y, self.sense_radius