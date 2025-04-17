import random
import math

WIDTH = 1920
HEIGHT = 1080


class Boid:
    heading = 0
    velocity = None
    center_x = 0
    center_y = 0

    center_points = []

    # Per the paper,
    #   random heading
    #   random velocity
    def __init__(self, init_x, init_y, boid_id):
        self.center_x = init_x
        self.center_y = init_y
        self.boid_id = boid_id

        self.path_pts = [(self.center_x, self.center_y)]

        self.heading = random.randint(0, 359)
        self.color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        self.velocity = 5 # random.randint(0, 15)
        self.size = 7

        # 'radius' of sensitivty circle
        self.sense_radius = 100

    def new_heading(self, neg, distance):
        print(distance)
        distance_ratio = distance / 25
        # print(distance_ratio)
        sensitivity_mag = pow(1.2, -7*distance_ratio)
        # print(f"sensitivy_mag={sensitivity_mag} & distance_ratio={distance_ratio} & distance={distance}")

        angle_delta = 90

        if neg:
            angle_delta = -angle_delta
        else:
            pass

        angle_delta = angle_delta * sensitivity_mag

        heading_angle_delta = self.heading + angle_delta
        header_modulo = heading_angle_delta % 360

        # print(f"heading is {heading_angle_delta} * sens_mag of {sensitivity_mag}")

        if heading_angle_delta > 360:
            # print(f"heading_angle_delta > 360 {heading_angle_delta % 360}")
            return header_modulo
        elif heading_angle_delta < 0:
            # remainder_angle = heading_angle_delta % 360
            # print(f"heading_angle_delta < 0 {(heading_angle_delta % 360) + remainder_angle}")
            # print(f">>> {heading_angle_delta} + {header_modulo} = {heading_angle_delta + header_modulo}")
            return (heading_angle_delta + header_modulo)
        else:
            # print(f"heading_angle_delta {heading_angle_delta}")
            return heading_angle_delta
        
    def avoid_walls(self):

        if self.heading % 90 == 0:
            print("self.heading is multiple of 90 so adding 1")
            self.heading += 1

        # NOTE: ROTATION IS COUNTER-CLOCKWISE

        wall_distance = 50

        # left wall
        if self.center_x - wall_distance < 0:
            # print("1")
            distance = self.center_x
            if self.heading < 270 and self.heading > 180:
                self.heading = self.new_heading(neg=False, distance=distance)
            elif self.heading < 180 and self.heading > 90:
                self.heading = self.new_heading(neg=True, distance=distance)
            else:
                # print("1x")
                return False

        # right wall
        if self.center_x + wall_distance >= WIDTH:
            # print("2")
            distance = (WIDTH - self.center_x)
            if self.heading < 360 and self.heading > 270:
                self.heading = self.new_heading(neg=True, distance=distance)
            elif self.heading < 90 and self.heading > 0:
                self.heading = self.new_heading(neg=False, distance=distance)
            else:
                # print("2x")
                return False

        # top
        if self.center_y + wall_distance >= HEIGHT:
            # print("3")
            distance = (HEIGHT - self.center_y)
            if self.heading < 90 and self.heading > 0:
                self.heading = self.new_heading(neg=True, distance=distance)
            elif self.heading < 180 and self.heading > 90:
                self.heading = self.new_heading(neg=False, distance=distance)
            else:
                # print("3x")
                return False

        # bottom
        if self.center_y - wall_distance < 0:
            # print("4")
            distance = self.center_y
            if self.heading > 270 and self.heading < 360:
                self.heading = self.new_heading(neg=False, distance=distance)
            elif self.heading < 270 and self.heading > 180:
                self.heading = self.new_heading(neg=True, distance=distance)
            else:
                # print("4x")
                return False

        return True
    
    def collision_avoidance(self, neighborhood):
        # get nearby boids
        print(neighborhood)
    
    def pilot(self, boids):
        # flight/pilot module are combined

        # COMBINE ACCELERATION REQUESTS HERE

        # NOTE: PRECENDENCE LEVELS
        # flock centering
        # self.stay_centered(boids)
        # velocity matching
        # collision avoidance
        self.avoid_walls()
        # self.pacman_walls()

        rad = math.radians(self.heading)
        
        self.center_x += self.velocity * math.cos(rad)
        self.center_y += self.velocity * math.sin(rad)

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
            rad = math.radians(self.heading)
            rx = px * math.cos(rad) - py * math.sin(rad)
            ry = px * math.sin(rad) + py * math.cos(rad)
            rotated_points.append((self.center_x + rx, self.center_y + ry))

        # path_to_draw = list(reversed(self.path_pts))[:100]
        path_to_draw = self.path_pts

        # print(self.heading)

        return rotated_points, self.center_x, self.center_y, self.sense_radius, self.color, path_to_draw
    
    def get_neighbhors(self, boids):
        neighbhors = []
        neighbhors_x_sum = 0
        neighbhors_y_sum = 0
        for b in boids:
            euclid_distance = math.sqrt( (self.center_x - b.center_x)**2 + (self.center_y - b.center_y)**2 )
            # print(f"euclid_dist={euclid_distance} id={self.color}")
            if euclid_distance < self.sense_radius:
                neighbhors.append( (b) )
                neighbhors_x_sum += b.center_x
                neighbhors_y_sum += b.center_y

        return neighbhors, neighbhors_x_sum, neighbhors_y_sum

    def stay_centered(self, boids):
        #
        # print("tries to find 'center'")

        # find average 'center' of flock
        neighbhors, neighbhors_x_sum, neighbhors_y_sum = self.get_neighbhors(boids=boids)
        num_neighbhors = len(neighbhors)

        center_flock_x = neighbhors_x_sum / num_neighbhors
        center_flock_y = neighbhors_y_sum / num_neighbhors

        diff_x = self.center_x - center_flock_x
        diff_y = self.center_y - center_flock_y
        center_heading_relative = math.degrees(math.atan2(diff_y, diff_x))

        # print(f"num of neighbhors={num_neighbhors} with center at ({center_flock_x}, {center_flock_y}) and curr boid center at ({self.center_x}, {self.center_y})")
        # print(f"curr_heading_relative={center_heading_relative}")
        if center_heading_relative == 0:
            # no neighbhors so
            pass
        else:
            print(f"curr_heading_relative={center_heading_relative} boid_heading={self.heading}")
            self.center_points.append( (center_flock_x, center_flock_y) )
            if self.heading > center_heading_relative:
                self.heading += 2
            else:
                self.heading -= 2
