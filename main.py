import pygame
from boid import Boid, WIDTH, HEIGHT, END_GOAL_RADIUS
from map_build import Map
import random

pygame.init()

# Set up display
ACTUAL_WIDTH, ACTUAL_HEIGHT = 1280, 720
# ACTUAL_WIDTH, ACTUAL_HEIGHT = 1920, 1080
screen = pygame.display.set_mode((ACTUAL_WIDTH, ACTUAL_HEIGHT))
pygame.display.set_caption("Boids")

# Clock to control frame rate
clock = pygame.time.Clock()

# List to hold triangle sprites
boids = []

def update_surface():
    surface = pygame.Surface((WIDTH, HEIGHT))
    surface.fill((4, 30, 66)) # UNH COLORS

    for top_left_xy, width_height in map_boxes:
        pygame.draw.rect(surface=surface, color=(211, 211, 211), rect=pygame.Rect(top_left_xy, width_height))

    pygame.draw.circle(surface, (36, 252, 3), (objective_coords), END_GOAL_RADIUS)

    new_boids = []

    for boid in boids:

        boid_new = boid.pilot(boids, objective_coords)
        #####
        if boid_new != None:
            new_boids.append(boid_new)
            rotated_points, center_x, center_y, sense_radius, color, path_to_draw = boid.get_boid()

            pygame.draw.lines(surface=surface, color=color, closed=False, points=list(reversed(path_to_draw)))
            pygame.draw.polygon(surface, (255, 255, 0), rotated_points)
        #####

    # boids = new_boids

    return surface, new_boids

if __name__ == '__main__':
    curr_id = 0

    # map_obj = Map("map1.in")
    # map_obj = Map("map2.in")
    map_obj = Map("map3.in")
    map_boxes, objective_coords = map_obj.build_map()

    boids.append(Boid(100, 100, curr_id, wall_coords=map_boxes))
    curr_id += 1
    # boids.append(Boid(150, 100, 2, wall_coords=map_boxes))
    # boids.append(Boid(180, 130, 3, wall_coords=map_boxes))
    # boids.append(Boid(170, 70, 4, wall_coords=map_boxes))

    # boids.append(Boid(225, 150, 5, wall_coords=map_boxes))
    # boids.append(Boid(125, 90, 6, wall_coords=map_boxes))

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
            # elif event.type == pygame.MOUSEBUTTONDOWN:
            #     stopped = True

            # if not stopped:
            #     surface, new_boids = update_surface()
            #     boids = new_boids

        surface, new_boids = update_surface()
        boids = new_boids
        
        resized_screen = pygame.transform.smoothscale(surface, (ACTUAL_WIDTH,ACTUAL_HEIGHT)) 
        screen.blit(resized_screen, (0, 0))

        pygame.display.update()
        clock.tick(60)
                

    pygame.quit()
