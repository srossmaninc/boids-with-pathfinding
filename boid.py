import random
import math

WIDTH = 1920
HEIGHT = 1080

MAX_SPEED = 5
MIN_SPEED = 2

"""
This program is based off of Craig Reynold's paper
"Flocks, herds and schools: A distributed behavioral model"

I'd like to thank the nice ECE department at Cornell for simplifying scary vector math
https://people.ece.cornell.edu/land/courses/ece4760/labs/s2021/Boids/Boids.html#:~:text=Boids%20is%20an%20artificial%20life,very%20simple%20set%20of%20rules.
"""

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


    # # # # # # # # # # # # # # # # # # # # 
    # COLLISION AVOIDANCE
    # # # # # # # # # # # # # # # # # # # # 

    def avoid_walls(self):

        wall_distance = 100 #50
        turn_factor = 0.5

        for top_left, bottom_right in self.wall_coords:
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
    
    def avoid_borders(self):

        wall_distance = 100
        turn_factor = 0.5

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


        # wall avoidance
        # self.avoid_walls()

        # self.avoid_borders()
    
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

    def pilot(self, boids, mouse_xy):
        # flight/pilot module are combined

        # COMBINE ACCELERATION REQUESTS HERE
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


        if num_neighbhors > 0:
            # self.xvelocity *= mouse_diff_x
            # self.xvelocity *= mouse_diff_y

            self.collision_avoidance(neighborhood=neighbhors)

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

        self.avoid_borders()


        self.center_x += self.xvelocity
        self.center_y += self.yvelocity
        # print(f"curr x_velocity {self.xvelocity} y_velocity {self.yvelocity}")

        self.path_pts.append((self.center_x, self.center_y))

        return True, self

    def get_boid(self):
        # Define triangle relative to center
        points = [
            (self.size, 0),
            (-self.size, -self.size+4),
            (-self.size, self.size-4)
        ]

        # Rotate points
        rotated_points = []
        for px, py in points:
            rad = math.atan2(self.yvelocity, self.xvelocity)
            rx = px * math.cos(rad) - py * math.sin(rad)
            ry = px * math.sin(rad) + py * math.cos(rad)
            rotated_points.append((self.center_x + rx, self.center_y + ry))

        path_to_draw = list(reversed(self.path_pts))[:100]
        # path_to_draw = self.path_pts

        # print(self.heading)

        return rotated_points, self.center_x, self.center_y, self.sense_radius, self.color, path_to_draw