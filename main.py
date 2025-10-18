import pygame
import sys
import argparse
import config
from swarm import Swarm

def main(drone_count):
    pygame.init()
    screen = pygame.display.set_mode((config.SCREEN_WIDTH, config.SCREEN_HEIGHT))
    pygame.display.set_caption(f"Drone Hordes - Step 2: {drone_count} Drones (Ideal Gas)")
    clock = pygame.time.Clock()

    swarm = Swarm(n=drone_count)

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                    running = False

        swarm.update()

        screen.fill(config.COLOR_BACKGROUND)
        swarm.draw(screen)
        
        pygame.display.flip()

        clock.tick(config.FPS)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drone hordes simulation.")
    parser.add_argument(
        '-n', '--count', 
        type=int, 
        default=config.DEFAULT_DRONE_COUNT, 
        help='Number of drones.'
    )
    args = parser.parse_args()
    
    main(drone_count=args.count)
