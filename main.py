import pygame
import sys
import argparse
import config
from swarm import Swarm
from behaviors import GasIdealBehavior, FollowMouseBehavior

def main(drone_count):
    pygame.init()
    screen = pygame.display.set_mode((config.SCREEN_WIDTH, config.SCREEN_HEIGHT))
    pygame.display.set_caption(f"Drone Hordes - {drone_count} Drones | MODE: Ideal Gas")
    clock = pygame.time.Clock()

    swarm = Swarm(n=drone_count)
    
    behaviors = {
        'gas_ideal': GasIdealBehavior(),
        'follow_mouse': FollowMouseBehavior()
    }
    
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                    running = False
                
                if event.key == pygame.K_1:
                    swarm.set_behavior(behaviors['gas_ideal'])
                    swarm.reset_drone_velocities(speed=config.DRONE_SPEED)
                    pygame.display.set_caption(f"Drone Hordes - {drone_count} Drones | MODE: Ideal Gas")
                
                if event.key == pygame.K_2:
                    swarm.set_behavior(behaviors['follow_mouse'])
                    swarm.reset_drone_velocities(speed=0)
                    pygame.display.set_caption(f"Drone Hordes - {drone_count} Drones | MODE: Follow Mouse")

        swarm.update()

        screen.fill(config.COLOR_BACKGROUND)
        swarm.draw(screen)
        
        pygame.display.flip()

        clock.tick(config.FPS)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Drone swarm simulation.")
    parser.add_argument(
        '-n', '--count', 
        type=int, 
        default=config.DEFAULT_DRONE_COUNT, 
        help='Number of drones in the simulation.'
    )
    args = parser.parse_args()
    
    main(drone_count=args.count)
