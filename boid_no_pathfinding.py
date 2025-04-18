import pygame
from classes import Boid, WIDTH, HEIGHT

pygame.init()

# Set up display
ACTUAL_WIDTH, ACTUAL_HEIGHT = 1280, 720
screen = pygame.display.set_mode((ACTUAL_WIDTH, ACTUAL_HEIGHT))
pygame.display.set_caption("Boids")

# Clock to control frame rate
clock = pygame.time.Clock()

# List to hold triangle sprites
boids = []

def update_surface():
    surface = pygame.Surface((WIDTH, HEIGHT))

    new_boids = []

    for boid in boids:

        ret, boid_new = boid.pilot(boids)
        new_boids.append(boid_new)
        # ret = boid.pacman_walls()
        if not ret:
            exit(-1)

        rotated_points, center_x, center_y, sense_radius, color, path_to_draw = boid.get_boid()

        # for pt in boid.center_points:
        #     pygame.draw.circle(surface, (255, 0, 0), pt, 5)

        # pygame.draw.circle(surface, (0, 0, 255), (center_x, center_y), sense_radius)
        # pygame.draw.circle(surface, (30, 30, 30), (center_x, center_y), sense_radius-1)
        pygame.draw.lines(surface=surface, color=color, closed=False, points=list(reversed(path_to_draw)))
        pygame.draw.polygon(surface, (255, 255, 0), rotated_points)

    # boids = new_boids

    return surface, new_boids

curr_id = 0

boids.append(Boid(100, 100, 1))
boids.append(Boid(150, 100, 2))

# Game loop
running = True
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
                boids.append(Boid(60*(curr_id+1), 10*(curr_id+3), boid_id=curr_id))
                curr_id += 1

    surface, new_boids = update_surface()
    boids = new_boids

    resized_screen = pygame.transform.smoothscale(surface, (ACTUAL_WIDTH,ACTUAL_HEIGHT)) 
    screen.blit(resized_screen, (0, 0))



    pygame.display.update()
    clock.tick(60)

pygame.quit()
