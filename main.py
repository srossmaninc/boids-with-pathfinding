import pygame
from boid import Boid, WIDTH, HEIGHT, END_GOAL_RADIUS
from map_build import Map
import random

pygame.init()

# Set up display
ACTUAL_WIDTH, ACTUAL_HEIGHT = 1280, 720
screen = pygame.display.set_mode((ACTUAL_WIDTH, ACTUAL_HEIGHT))
pygame.display.set_caption("Boids")

# Clock to control frame rate
clock = pygame.time.Clock()

# List to hold triangle sprites
boids = []
curr_id = 0

previous_paths = []

def update_surface():
    surface = pygame.Surface((WIDTH, HEIGHT))
    surface.fill((4, 30, 66)) # UNH COLORS

    for top_left_xy, width_height in map_boxes:
        pygame.draw.rect(surface=surface, color=(211, 211, 211), rect=pygame.Rect(top_left_xy, width_height))

    pygame.draw.circle(surface, (36, 252, 3), (objective_coords), END_GOAL_RADIUS)

    new_boids = []

    for (previous_path, previous_color) in previous_paths:
        pygame.draw.lines(surface=surface, color=previous_color, closed=False, points=list(reversed(previous_path)))

    # if only a quarter/less than a quarter of the boids remain, map completed
    if len(boids) <= int(original_num_boids * 0.25):
        return surface, None

    for boid in boids:

        match boid.pilot(boids, objective_coords):
            case (None, path_to_draw, path_color):
                previous_paths.append((path_to_draw, path_color))
            case (self, path_to_draw, path_color):
                #####
                new_boids.append(self)
                rotated_points, center_x, center_y, sense_radius = boid.get_boid()

                pygame.draw.lines(surface=surface, color=path_color, closed=False, points=list(reversed(path_to_draw)))
                pygame.draw.polygon(surface, (255, 255, 0), rotated_points)
                #####

    return surface, new_boids

def create_boid(init_x, init_y):
    global curr_id
    curr_id = curr_id + 1
    return Boid(init_x, init_y, curr_id, wall_coords=map_boxes)

if __name__ == '__main__':

    # map_obj = Map("map1.in")
    # map_obj = Map("map2.in")
    # map_obj = Map("map3.in")
    map_obj = Map("map4.in")
    # map_obj = Map("map5.in")
    map_boxes, objective_coords = map_obj.build_map()

    # mandatory walls
    map_top_left = (-50, -50)
    map_top_right = (WIDTH - 2, -50)
    map_bottom_left = (-50, HEIGHT - 2)
    map_bottom_right = (WIDTH - 2, HEIGHT - 2)
    # left wall (starts at top_left, goes to bottom left)
    map_boxes.append( (map_top_left, (52, HEIGHT + 100)) )
    # top wall (starts at top_left, goes to top_right)
    map_boxes.append( (map_top_left, (WIDTH + 100, 52)) )
    # bottom wall (starts at bottom_left, goes to bottom_right)
    map_boxes.append( (map_bottom_left, (WIDTH + 100, 52)) )
    # right wall (starts at top_right, goes to bottom_right)
    map_boxes.append( (map_top_right, (52, HEIGHT + 100)) )

    RUN_3 = not True
    RUN_6 = True
    RUN_9 = not True

    if RUN_3:
        boids.append( create_boid(100, 100) )
        boids.append( create_boid(150, 100) )
        boids.append( create_boid(180, 130) )
    elif RUN_6:
        boids.append( create_boid(100, 100) )
        boids.append( create_boid(150, 100) )
        boids.append( create_boid(180, 130) )
        boids.append( create_boid(170, 70) )
        boids.append( create_boid(225, 150) )
        boids.append( create_boid(225, 200) )
    elif RUN_9:
        boids.append( create_boid(100, 100) )
        boids.append( create_boid(150, 100) )
        boids.append( create_boid(180, 130) )
        boids.append( create_boid(170, 70) )
        boids.append( create_boid(225, 150) )
        boids.append( create_boid(225, 200) )
        boids.append( create_boid(50, 50) )
        boids.append( create_boid(70, 50) )
        boids.append( create_boid(85, 65) )
    else:
        boids.append( create_boid(100, 100) )


    original_num_boids = len(boids)

    # Game loop
    running = True
    stopped = False
    while running:
        screen.fill((30, 30, 30))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                x, y = pygame.mouse.get_pos()
                x = x*(ACTUAL_WIDTH/WIDTH)
                y = y*(ACTUAL_HEIGHT/HEIGHT)
                if x < WIDTH - 20 and x > 20 and y < HEIGHT - 20 and y > 0:
                    # boids.append(Boid(x, y, curr_id))
                    ptx = random.randint(70, 100)
                    pty = random.randint(70, 100)
                    boids.append(Boid(ptx, pty, boid_id=curr_id, wall_coords=map_boxes))
                    curr_id += 1

        surface = None
        match update_surface():
            case (surface, None):
                boids = []
                for (previous_path, previous_color) in previous_paths:
                    pygame.draw.lines(surface=surface, color=previous_color, closed=False, points=list(reversed(previous_path)))
            case (surface, new_boids):
                boids = new_boids
                
        resized_screen = pygame.transform.smoothscale(surface, (ACTUAL_WIDTH,ACTUAL_HEIGHT)) 
        screen.blit(resized_screen, (0, 0))

        pygame.display.update()
        clock.tick(60)
                

    pygame.quit()
