import pygame
import sys
import config
from swarm import Drone

def main():
    pygame.init()
    screen = pygame.display.set_mode((config.SCREEN_WIDTH, config.SCREEN_HEIGHT))
    pygame.display.set_caption("Drone Hordes - Step 1: One Drone")
    clock = pygame.time.Clock()

    drone = Drone()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    running = False

        drone.update()

        screen.fill(config.COLOR_BACKGROUND)
        drone.draw(screen)
        
        pygame.display.flip()

        clock.tick(config.FPS)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
