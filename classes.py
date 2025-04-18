import random
import math

WIDTH = 1920
HEIGHT = 1080


class Boid:
    center_x = 0
    center_y = 0
    xvelocity = 0
    yvelocity = 0

    center_points = []

    # Per the paper,
    #   random heading
    #   random velocity
    def __init__(self, init_x, init_y, boid_id):
        self.center_x = init_x
        self.center_y = init_y
        self.boid_id = boid_id

        self.path_pts = [(self.center_x, self.center_y)]

        self.color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))

        self.xvelocity = 5#random.randint(3, 10) # 5
        self.yvelocity = 5#random.randint(3, 10) # 5
        self.size = 7

        # 'radius' of sensitivty circle
        self.sense_radius = 100


    # # # # # # # # # # # # # # # # # # # # 
    # COLLISION AVOIDANCE
    # # # # # # # # # # # # # # # # # # # # 

    def avoid_walls(self):

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
            diff_x = neighbor.center_x - self.center_x
            diff_y = neighbor.center_y - self.center_y

            if abs(diff_x) < bump_dist and abs(diff_y) < bump_dist:
                # Generate a force pushing AWAY from the neighbor
                avoid_force_x -= diff_x  # Subtract the difference to move in the opposite direction
                avoid_force_y -= diff_y

        self.xvelocity += avoid_force_x * 0.05
        self.yvelocity += avoid_force_y * 0.05


        # wall avoidance
        self.avoid_walls()
    
    # # # # # # # # # # # # # # # # # # # # 
    # FLOCK CENTERING
    # # # # # # # # # # # # # # # # # # # # 

    def stay_centered(self, num_neighbhors, neighbhors_x_sum, neighbhors_y_sum):
        # find average 'center' of flock

        if num_neighbhors != 0:
            center_flock_x = neighbhors_x_sum / num_neighbhors
            center_flock_y = neighbhors_y_sum / num_neighbhors

            diff_x = center_flock_x - self.center_x
            diff_y = center_flock_y - self.center_y
            # print(f"curr_heading_relative={center_heading_relative} boid_heading={self.heading}")
            # print(f"diff_x {diff_x} diff_y {diff_y}")
            self.center_points.append( (center_flock_x, center_flock_y) )

            # if diff_x > 0:
            #     self.xvelocity += diff_x/(self.sense_radius*5)
            # else:
            #     self.xvelocity -= diff_x/(self.sense_radius*5)

            # if diff_y > 0:
            #     self.yvelocity += diff_y/(self.sense_radius*5)
            # else:
            #     self.yvelocity -= diff_y/(self.sense_radius*5)

            self.xvelocity += diff_x/(self.sense_radius*5)
            self.yvelocity += diff_y/(self.sense_radius*5)

    # # # # # # # # # # # # # # # # # # # # 
    # MATCH VELOCITY
    # # # # # # # # # # # # # # # # # # # # 

    def match_velocity(self, num_neighbors, xvelocity_sum, yvelocity_sum):

        # need to average and match

        if num_neighbors != 0:

            avg_xvelocity = xvelocity_sum / num_neighbors
            avg_yvelocity = yvelocity_sum / num_neighbors
            self.xvelocity += (avg_xvelocity - self.xvelocity)*0.2
            self.yvelocity += (avg_yvelocity - self.yvelocity)*0.2

            # if avg_xvelocity > self.xvelocity:
            #     self.xvelocity += 0.1
            # elif avg_xvelocity < self.xvelocity:
            #     self.xvelocity -= 0.1

            # if avg_yvelocity > self.yvelocity:
            #     self.yvelocity += 0.1
            # elif avg_yvelocity < self.yvelocity:
            #     self.yvelocity -= 0.1

    # # # # # # # # # # # # # # # # # # # # 
    # GET NEIGHBHORS
    # # # # # # # # # # # # # # # # # # # # 

    def get_neighbhors(self, boids):
        neighbhors = []
        for b in boids:
            if b.boid_id != self.boid_id:
                euclid_distance = math.sqrt( (self.center_x - b.center_x)**2 + (self.center_y - b.center_y)**2 )
                # print(f"euclid_dist={euclid_distance} id={self.color}")
                if euclid_distance < self.sense_radius:
                    neighbhors.append( (b) )
        return neighbhors

    # # # # # # # # # # # # # # # # # # # # 
    # PILOT
    # # # # # # # # # # # # # # # # # # # # 

    def pilot(self, boids):
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
        # flock centering
        self.stay_centered(num_neighbhors=num_neighbhors, neighbhors_x_sum=neighbhors_x_sum, neighbhors_y_sum=neighbhors_y_sum)
        # velocity matching
        self.match_velocity(num_neighbhors, xvelocity_sum=xvelocity_sum, yvelocity_sum=yvelocity_sum)
        # collision avoidance

        speed = math.sqrt(self.xvelocity**2 + self.yvelocity**2)

        max_speed = 10 # velocity of 10
        min_speed = 3
        if speed > max_speed:
            self.xvelocity = (self.xvelocity/speed) * max_speed
            self.yvelocity = (self.yvelocity/speed) * max_speed
        elif speed < min_speed:
            self.xvelocity = (self.xvelocity/speed) * min_speed
            self.yvelocity = (self.yvelocity/speed) * min_speed


        self.collision_avoidance(neighborhood=neighbhors)

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

        # path_to_draw = list(reversed(self.path_pts))[:100]
        path_to_draw = self.path_pts

        # print(self.heading)

        return rotated_points, self.center_x, self.center_y, self.sense_radius, self.color, path_to_draw